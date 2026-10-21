"""
Convert one car of a batch: read it, build the SVJ, write the SVJ, GLBs, tire plots and
the per-car log into the car's output folder.

``convert_one`` is a plain top-level function of picklable arguments, so the batch tab
can run it in the calling thread (one car at a time, log lines streamed as they are
produced) or in worker processes (several cars at once).  Nothing here touches tkinter.
"""
from __future__ import annotations

import json
import shutil
import tempfile
import traceback
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from car_archives import ArchiveCar, extract_car
from converter import (CONV_VER, SVJ_VERSION, _car_stem, _clean, build_svj,
                       read_car_directory)


def _ts_long() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _already_converted(car_out: Path, need_glb: bool) -> bool:
    """True when a car's output folder already holds its SVJ (and, when GLBs are
    wanted, at least one GLB beside it or in meshes/)."""
    try:
        if not car_out.is_dir() or not any(car_out.glob("*.svj.json")):
            return False
        if need_glb:
            return any(car_out.glob("*.glb")) or any((car_out / "meshes").glob("*.glb"))
        return True
    except OSError:
        return False


def _write_car_log(car_out: Path, stem: str, car_path: Path,
                   car_log: list[str], bench: dict,
                   include_plots: bool, convert_glb: bool,
                   error: Optional[str] = None) -> None:
    """Write a detailed per-car conversion_log.txt."""
    hdr = "═" * 70
    lines: list[str] = [
        hdr,
        "AC → SVJ  Conversion Log",
        f"Converter : v{CONV_VER}   SVJ spec : {SVJ_VERSION}",
        f"Timestamp : {_ts_long()}",
        f"Car path  : {car_path}",
        hdr,
        "",
    ]

    if error:
        lines += [
            "STATUS: FAILED",
            "",
            "ERROR DETAIL:",
            error,
        ]
    else:
        lines += ["CONVERSION LOG:", ""]
        lines += car_log
        lines += [""]

        # Pacejka fit quality table
        if bench:
            lines += ["", "─" * 70, "PACEJKA FIT QUALITY", "─" * 70]
            lines.append(
                f"  {'Axle':<14}  {'Source':<12}  "
                f"{'Lat R²':>8}  {'Lat RMSE N':>10}  "
                f"{'Long R²':>8}  {'Long RMSE N':>11}  {'Fz0 N':>7}"
            )
            lines.append("  " + "─" * 68)
            for axle_key, br in bench.items():
                p = br.params
                lines.append(
                    f"  {axle_key:<14}  {p.source:<12}  "
                    f"{br.fit_lateral['r2']:>8.4f}  "
                    f"{br.fit_lateral['rmse_N']:>10.2f}  "
                    f"{br.fit_longitudinal['r2']:>8.4f}  "
                    f"{br.fit_longitudinal['rmse_N']:>11.2f}  "
                    f"{p.FZ0:>7.0f}"
                )

        # Output file manifest
        lines += ["", "─" * 70, "OUTPUT FILES", "─" * 70]
        lines.append(f"  {stem}.svj.json")
        lines.append(f"  conversion_log.txt  ← this file")
        if include_plots and bench:
            for axle_key in bench:
                tag = axle_key.replace(" ", "_")
                for suffix in ("lateral", "longitudinal", "mu_vs_fz"):
                    lines.append(f"  {stem}.tires_{tag}_{suffix}.png")
        if convert_glb:
            for sub in ("meshes", ""):
                glb_dir = car_out / sub if sub else car_out
                if glb_dir.is_dir():
                    for g in sorted(glb_dir.glob("*.glb")):
                        lines.append(f"  {sub + '/' if sub else ''}{g.name}")

    lines += ["", hdr, ""]
    (car_out / "conversion_log.txt").write_text(
        "\n".join(lines), encoding="utf-8")


def convert_one(job: dict, emit: Optional[Callable[[str], None]] = None) -> dict:
    """Convert one car.  ``job``: ``src`` (a car folder or an ``ArchiveCar``), ``car_out``
    (its output folder), ``include_plots``, ``convert_glb``, ``include_skins``,
    ``mesh_sub``, ``include_damage``, ``highest_lod_only``.

    Returns ``{status: ok|skipped|error, name, label, stem, ui: [log lines for the UI],
    car_log: [lines of conversion_log.txt], csv: [rows], error: (message, traceback)}``.
    ``emit`` is also called with every UI line as it is produced.
    """
    src = job["src"]
    car_out: Path = job["car_out"]
    include_plots = job["include_plots"]
    convert_glb = job["convert_glb"]
    ui: list[str] = []

    def put(line: str) -> None:
        ui.append(line)
        if emit is not None:
            emit(line)

    zip_tmp: Optional[str] = None
    if isinstance(src, ArchiveCar):
        zip_tmp = tempfile.mkdtemp(prefix="ac_svj_zip_")      # removed right after
        car_path = Path(zip_tmp) / src.name
    else:
        car_path = Path(src)
    label = src.label if isinstance(src, ArchiveCar) else car_path.name
    res: dict = {"status": "error", "name": car_path.name, "label": label, "stem": "",
                 "ui": ui, "car_log": [], "csv": [], "error": None}
    car_out.mkdir(parents=True, exist_ok=True)

    glb_tmp: Optional[str] = None
    glb_dir: Optional[Path] = None
    if convert_glb:
        glb_tmp = tempfile.mkdtemp(prefix="ac_svj_glb_")
        glb_dir = Path(glb_tmp)

    car_log_lines: list[str] = []
    bench: dict = {}
    svj: dict = {}
    success = False
    error_detail = ""
    try:
        if isinstance(src, ArchiveCar):
            put(f"ℹ Extracting {src.zip_path.name} → {src.name}")
            extract_car(src, Path(zip_tmp), job["include_skins"])
        ini_files, cm_meta, ctrl_files, data_dir = read_car_directory(car_path)

        if not ini_files:
            note = (cm_meta or {}).get("_data_source", "")
            msg = f"⊘ Skipped — {note}"
            put(msg)
            car_log_lines = [msg]
            res["car_log"] = car_log_lines
            res["status"] = "skipped"
            _write_car_log(car_out, car_path.name, car_path, car_log_lines, {},
                           include_plots, convert_glb)
            return res

        put(f"ℹ Files found: {', '.join(sorted(ini_files.keys()))}")
        if ctrl_files:
            put(f"ℹ Controllers: {', '.join(sorted(ctrl_files.keys()))}")

        svj, car_log, bench = build_svj(
            ini_files, cm_meta,
            data_dir=data_dir,
            ctrl_files=ctrl_files,
            glb_output_dir=glb_dir,
            include_skins=job["include_skins"],
            meshes_subdir=job["mesh_sub"],
            include_damage_textures=job["include_damage"],
            highest_lod_only=job["highest_lod_only"],
            include_plots=include_plots,
        )
        svj = _clean(svj)
        stem = _car_stem(svj, car_path.name)
        res["stem"] = stem

        for line in car_log:                       # every build_svj log line to the UI
            put(line)
            car_log_lines.append(line)

        (car_out / f"{stem}.svj.json").write_text(
            json.dumps(svj, indent=2), encoding="utf-8")

        mesh_sub = job["mesh_sub"]
        glb_src = ((glb_dir / mesh_sub if mesh_sub else glb_dir) if glb_dir else None)
        if glb_src is not None and glb_src.is_dir():
            mesh_out = car_out / mesh_sub if mesh_sub else car_out
            mesh_out.mkdir(exist_ok=True)
            for g in glb_src.glob("*.glb"):
                shutil.copy2(str(g), str(mesh_out / g.name))

        if include_plots and bench:
            for axle_key, br in bench.items():
                tag = axle_key.replace(" ", "_")
                (car_out / f"{stem}.tires_{tag}_lateral.png").write_bytes(br.lateral_png)
                (car_out / f"{stem}.tires_{tag}_longitudinal.png"
                 ).write_bytes(br.longitudinal_png)
                (car_out / f"{stem}.tires_{tag}_mu_vs_fz.png").write_bytes(br.mu_vs_fz_png)
                if br.mf62_camber_png:
                    (car_out / f"{stem}.tires_{tag}_mf62_camber.png"
                     ).write_bytes(br.mf62_camber_png)
                if br.mf62_lateral_png:
                    (car_out / f"{stem}.tires_{tag}_mf62_lateral.png"
                     ).write_bytes(br.mf62_lateral_png)

                vi = svj.get("vehicle_info", {})
                make = vi.get("make") or ""
                year = vi.get("year") or ""
                drive = vi.get("drive_type") or ""
                parts_ = axle_key.split("_", 1)
                axle_s = parts_[0]
                cmpnd = parts_[1] if len(parts_) > 1 else "0"
                p = br.params
                fz0_f = p.FZ0
                res["csv"].append(
                    f"{car_path.name},{stem},{make},{year},{drive},"
                    f"{cmpnd},{axle_s},"
                    f"{br.fit_lateral['r2']:.4f},"
                    f"{br.fit_lateral['rmse_N']:.2f},"
                    f"{br.fit_longitudinal['r2']:.4f},"
                    f"{br.fit_longitudinal['rmse_N']:.2f},"
                    f"{fz0_f:.0f},"
                    f"{fz0_f:.0f},{p.DY0:.4f},{p.DX0:.4f},"
                    f"OK"
                )
                put(f"✓ Tire fit [{axle_key}]: "
                    f"lat R²={br.fit_lateral['r2']:.4f}  "
                    f"long R²={br.fit_longitudinal['r2']:.4f}")

        glb_note = ""
        if convert_glb:
            glb_ok = (glb_src is not None and glb_src.is_dir()
                      and any(glb_src.glob("*.glb")))
            glb_note = "  ✓ GLB exported" if glb_ok else "  ⚠ no GLB"
        put(f"✓ Done: {car_path.name} — {len(bench)} tire fit(s){glb_note}")
        success = True
        res["status"] = "ok"

    except Exception as ex:                                       # noqa: BLE001
        error_detail = traceback.format_exc()
        put(f"✗ Error: {ex}")
        put(error_detail)
        car_log_lines.append(f"ERROR: {ex}\n{error_detail}")
        res["csv"].append(f"{car_path.name},,,,,,,,,,,,,,ERROR")
        res["error"] = (f"{type(ex).__name__}: {ex}", error_detail)

    finally:
        res["car_log"] = car_log_lines
        if res["status"] != "skipped":
            _write_car_log(
                car_out,
                _car_stem(svj, car_path.name) if success else car_path.name,
                car_path, car_log_lines, bench if success else {},
                include_plots, convert_glb,
                error=error_detail if not success else None)
        if glb_tmp:
            shutil.rmtree(glb_tmp, ignore_errors=True)
        if zip_tmp:
            shutil.rmtree(zip_tmp, ignore_errors=True)
    return res
