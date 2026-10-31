"""SVJ_to_AC -- Gradio UI and orchestrator.

Run with::

    python svj_to_ac.py

Serves a four-tab UI on ``http://127.0.0.1:7860``:

* **Upload SVJ** -- drop an ``.svj.json`` file, get back a Content
  Manager-ready zip.
* **Single SVJ** -- type a path to an SVJ on disk; useful while
  iterating on a hand-edited fixture.
* **Batch SVJ folder** -- point at a folder containing ``.svj.json``
  files; produces one combined zip named
  ``svj_ac_batch_<YYYYMMDD>_conv<UI_VER>.zip`` plus a
  ``batch_summary.csv``.
* **FBX Lab** -- generate the optional ASCII FBX 2014 skeleton from
  either a loaded SVJ or hand-entered dimensions
  (wheelbase / track / overall L*W*H / tyre width).

The UI is built in :func:`build_ui` (separate from :func:`main` so
headless tests can inspect tab structure without launching the server).
Per :mod:`ai_handoff/05_PITFALLS.md` "Gradio file uploads on Windows",
every uploaded path is wrapped in :class:`pathlib.Path` immediately on
entry.
"""

from __future__ import annotations

import csv
import datetime as _dt
import io
import sys
import tempfile
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import svj_loader
import pack_car
import fbx_skeleton

UI_VERSION = "0_1"
APP_TITLE = "SVJ → AC Converter · beta 0.1"


@dataclass
class ConversionResult:
    """Outcome of one SVJ -> AC conversion."""

    success: bool
    zip_path: Path | None
    log: str
    summary: dict[str, Any]


# --------------------------------------------------------------------------- #
# Core convert callbacks (UI-agnostic so they can be tested directly)         #
# --------------------------------------------------------------------------- #


def convert_one(svj_path: str | Path, out_dir: Path) -> ConversionResult:
    """Convert a single SVJ -> AC content folder + zip.

    All paths are normalised through :class:`pathlib.Path` before use.
    Returns a :class:`ConversionResult` regardless of success; on
    failure ``zip_path`` is ``None`` and ``log`` carries the traceback.
    """
    log = io.StringIO()
    src = Path(svj_path)
    log.write(f"Loading SVJ: {src.name}\n")
    try:
        doc = svj_loader.load(src)
    except svj_loader.SvjLoadError as exc:
        log.write(f"FAIL: {exc}\n")
        return ConversionResult(False, None, log.getvalue(), {"error": str(exc)})

    findings = svj_loader.validate(doc)
    for f in findings:
        log.write(f"VALIDATE [{f.severity}] {f.path}: {f.message}\n")

    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        zip_path = pack_car.convert(doc, out_dir)
        log.write(f"OK: wrote {zip_path.name} ({zip_path.stat().st_size} bytes)\n")
    except Exception as exc:  # noqa: BLE001 (we want UI to surface anything)
        log.write("FAIL:\n" + traceback.format_exc())
        return ConversionResult(False, None, log.getvalue(), {"error": str(exc)})

    return ConversionResult(
        True,
        zip_path,
        log.getvalue(),
        {
            "model": pack_car.model_id(doc),
            "brand": doc.vehicle_info.make,
            "name": doc.vehicle_info.model,
            "warnings": len(findings),
            "sha256": pack_car.sha256_of(zip_path),
            "bytes": zip_path.stat().st_size,
        },
    )


def convert_batch(folder: str | Path, out_dir: Path) -> ConversionResult:
    """Convert every ``*.svj.json`` under ``folder`` into one combined zip.

    Output filename is
    ``svj_ac_batch_<YYYYMMDD>_conv<UI_VERSION>.zip`` (matching the
    AC_to_SVJ batch naming convention from spec §11).
    """
    log = io.StringIO()
    src = Path(folder)
    if not src.is_dir():
        msg = f"FAIL: not a directory: {src}\n"
        log.write(msg)
        return ConversionResult(False, None, log.getvalue(), {"error": msg.strip()})

    files = sorted(src.rglob("*.svj.json"))
    if not files:
        msg = f"FAIL: no .svj.json files under {src}\n"
        log.write(msg)
        return ConversionResult(False, None, log.getvalue(), {"error": msg.strip()})

    out_dir.mkdir(parents=True, exist_ok=True)
    batch_root = out_dir / "_batch_staging"
    batch_root.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, Any]] = []
    for path in files:
        log.write(f"--- {path.relative_to(src)} ---\n")
        per_car_dir = batch_root / path.stem
        per_car_dir.mkdir(parents=True, exist_ok=True)
        result = convert_one(path, per_car_dir)
        log.write(result.log)
        row = {"input": str(path.relative_to(src)), "success": result.success}
        row.update(result.summary)
        summary_rows.append(row)

    # batch_summary.csv at batch_root level.
    csv_path = batch_root / "batch_summary.csv"
    fieldnames = sorted({k for row in summary_rows for k in row.keys()})
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)
    log.write(f"\nWrote batch_summary.csv with {len(summary_rows)} rows\n")

    date_tag = _dt.date.today().strftime("%Y%m%d")
    zip_name = f"svj_ac_batch_{date_tag}_conv{UI_VERSION}.zip"
    zip_path = pack_car.zip_folder(batch_root, out_dir / zip_name, arcname_base="batch")
    log.write(f"OK: wrote {zip_path.name} ({zip_path.stat().st_size} bytes)\n")

    return ConversionResult(
        True,
        zip_path,
        log.getvalue(),
        {
            "cars": len(summary_rows),
            "succeeded": sum(1 for r in summary_rows if r.get("success")),
            "failed": sum(1 for r in summary_rows if not r.get("success")),
        },
    )


def make_fbx_only(
    svj_path: str | Path | None,
    out_dir: Path,
    *,
    overall_length: float | None = None,
    overall_width: float | None = None,
    overall_height: float | None = None,
    tyre_width: float | None = None,
    wheelbase: float | None = None,
    track_front: float | None = None,
    track_rear: float | None = None,
) -> ConversionResult:
    """FBX Lab tab: generate an FBX from either an SVJ or manual dims.

    When ``svj_path`` is None we synthesise a minimal :class:`SvjDoc`
    from the manual inputs. The result is a single ``.fbx`` file in
    ``out_dir`` plus accompanying ``lods.ini`` and
    ``persistence.ksanim`` placeholder.
    """
    log = io.StringIO()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        if svj_path:
            doc = svj_loader.load(Path(svj_path))
            log.write(f"Loaded {Path(svj_path).name} for FBX Lab\n")
        else:
            doc = _synthetic_doc(wheelbase, track_front, track_rear)
            log.write("Using manual dimensions for FBX Lab\n")
    except svj_loader.SvjLoadError as exc:
        log.write(f"FAIL: {exc}\n")
        return ConversionResult(False, None, log.getvalue(), {"error": str(exc)})

    fbx_path = out_dir / "car.fbx"
    fbx_skeleton.write(
        doc, fbx_path,
        overall_length=overall_length,
        overall_width=overall_width,
        overall_height=overall_height,
        tyre_width=tyre_width,
    )
    fbx_skeleton.write_lods_ini(out_dir / "lods.ini")
    fbx_skeleton.write_ksanim_placeholder(out_dir / "persistence.ksanim")
    log.write(
        f"OK: wrote {fbx_path.name} ({fbx_path.stat().st_size} bytes), "
        "plus lods.ini and persistence.ksanim\n"
    )

    return ConversionResult(
        True, fbx_path, log.getvalue(),
        {"fbx_bytes": fbx_path.stat().st_size, "model": pack_car.model_id(doc)},
    )


# --------------------------------------------------------------------------- #
# Gradio UI                                                                   #
# --------------------------------------------------------------------------- #


def build_ui():  # type: ignore[no-untyped-def]
    """Build the Gradio Blocks UI. Returns the :class:`gr.Blocks` object.

    Imported lazily so the converter helpers above remain usable on a
    machine without gradio installed.
    """
    import gradio as gr  # noqa: WPS433 -- intentional local import

    with gr.Blocks(title=APP_TITLE) as ui:
        gr.Markdown(f"# {APP_TITLE}")
        gr.Markdown(
            "Inverse of AC_to_SVJ. Drop an SVJ v0.95 file (or a folder of "
            "them) and get a Content Manager-ready car zip back."
        )

        with gr.Tab("1 - Upload SVJ"):
            upload_file = gr.File(label="SVJ file (.svj.json)", file_types=[".json"])
            upload_btn = gr.Button("Convert", variant="primary")
            upload_log = gr.Textbox(label="Log", lines=10, interactive=False)
            upload_zip = gr.File(label="Download AC zip", interactive=False)

            def _on_upload(uploaded):  # noqa: ANN001
                if uploaded is None:
                    return "Drop an .svj.json file first.", None
                with tempfile.TemporaryDirectory() as td:
                    result = convert_one(uploaded.name, Path(td))
                    if result.zip_path:
                        # Copy out of the tmpdir so Gradio can serve it.
                        persistent = Path(tempfile.mkdtemp()) / result.zip_path.name
                        persistent.write_bytes(result.zip_path.read_bytes())
                        return result.log, str(persistent)
                    return result.log, None

            upload_btn.click(_on_upload, inputs=[upload_file], outputs=[upload_log, upload_zip])

        with gr.Tab("2 - Single SVJ (path)"):
            single_path = gr.Textbox(label="Path to .svj.json on disk")
            single_btn = gr.Button("Convert", variant="primary")
            single_log = gr.Textbox(label="Log", lines=10, interactive=False)
            single_zip = gr.File(label="Download AC zip", interactive=False)

            def _on_single(path_str):  # noqa: ANN001
                if not path_str:
                    return "Type a path first.", None
                with tempfile.TemporaryDirectory() as td:
                    result = convert_one(path_str, Path(td))
                    if result.zip_path:
                        persistent = Path(tempfile.mkdtemp()) / result.zip_path.name
                        persistent.write_bytes(result.zip_path.read_bytes())
                        return result.log, str(persistent)
                    return result.log, None

            single_btn.click(_on_single, inputs=[single_path], outputs=[single_log, single_zip])

        with gr.Tab("3 - Batch SVJ folder"):
            batch_folder = gr.Textbox(label="Folder containing .svj.json files")
            batch_btn = gr.Button("Convert all", variant="primary")
            batch_log = gr.Textbox(label="Log", lines=14, interactive=False)
            batch_zip = gr.File(label="Download batch zip", interactive=False)

            def _on_batch(folder_str):  # noqa: ANN001
                if not folder_str:
                    return "Type a folder path first.", None
                with tempfile.TemporaryDirectory() as td:
                    result = convert_batch(folder_str, Path(td))
                    if result.zip_path:
                        persistent = Path(tempfile.mkdtemp()) / result.zip_path.name
                        persistent.write_bytes(result.zip_path.read_bytes())
                        return result.log, str(persistent)
                    return result.log, None

            batch_btn.click(_on_batch, inputs=[batch_folder], outputs=[batch_log, batch_zip])

        with gr.Tab("4 - FBX Lab"):
            gr.Markdown(
                "Generate a placeholder ASCII FBX 2014 from a loaded SVJ "
                "or hand-entered dimensions. KsEditor can compile it to "
                "`.kn5`."
            )
            fbx_svj_path = gr.Textbox(
                label="Optional: path to .svj.json (overrides manual inputs)"
            )
            with gr.Row():
                fbx_wb = gr.Number(label="Wheelbase (m)", value=2.5)
                fbx_tf = gr.Number(label="Track front (m)", value=1.5)
                fbx_tr = gr.Number(label="Track rear (m)", value=1.5)
            with gr.Row():
                fbx_len = gr.Number(label="Overall length (m)", value=4.0)
                fbx_wid = gr.Number(label="Overall width (m)", value=1.8)
                fbx_hgt = gr.Number(label="Overall height (m)", value=1.3)
                fbx_tw = gr.Number(label="Tyre width (m)", value=0.225)
            fbx_btn = gr.Button("Generate FBX", variant="primary")
            fbx_log = gr.Textbox(label="Log", lines=8, interactive=False)
            fbx_file = gr.File(label="Download .fbx", interactive=False)

            def _on_fbx(svj_path, wb, tf, tr, le, wi, he, tw):  # noqa: ANN001
                with tempfile.TemporaryDirectory() as td:
                    result = make_fbx_only(
                        svj_path or None,
                        Path(td),
                        overall_length=le, overall_width=wi,
                        overall_height=he, tyre_width=tw,
                        wheelbase=wb, track_front=tf, track_rear=tr,
                    )
                    if result.zip_path:  # zip_path is the .fbx in this tab
                        persistent = Path(tempfile.mkdtemp()) / result.zip_path.name
                        persistent.write_bytes(result.zip_path.read_bytes())
                        return result.log, str(persistent)
                    return result.log, None

            fbx_btn.click(
                _on_fbx,
                inputs=[fbx_svj_path, fbx_wb, fbx_tf, fbx_tr,
                        fbx_len, fbx_wid, fbx_hgt, fbx_tw],
                outputs=[fbx_log, fbx_file],
            )

    return ui


# --------------------------------------------------------------------------- #
# Synthetic SvjDoc for FBX Lab manual-input mode                              #
# --------------------------------------------------------------------------- #


def _synthetic_doc(
    wheelbase: float | None,
    track_front: float | None,
    track_rear: float | None,
):
    """Build a minimally valid :class:`SvjDoc` from a few scalars."""
    from svj_loader import (
        SvjBrakes,
        SvjChassis,
        SvjCorner,
        SvjDoc,
        SvjMetadata,
        SvjPowertrain,
        SvjSuspension,
        SvjTires,
        SvjVehicleInfo,
    )

    wb = float(wheelbase or 2.5)
    tf = float(track_front or 1.5)
    tr = float(track_rear or 1.5)
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("Manual", "FBX_Lab", 2026, "test", "RWD"),
        chassis=SvjChassis(1000.0, wb, tf, tr, [0.0, 0.0, -0.4]),
        suspension=SvjSuspension(corners={
            "FL": SvjCorner("FL", [0.0, -tf / 2.0, -0.3]),
            "FR": SvjCorner("FR", [0.0, tf / 2.0, -0.3]),
            "RL": SvjCorner("RL", [-wb, -tr / 2.0, -0.3]),
            "RR": SvjCorner("RR", [-wb, tr / 2.0, -0.3]),
        }),
        tires=SvjTires(sets={"tire_front_1": {"dimensions": {"overall_diameter": 0.62}}}),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.6, "fixed"),
        metadata=SvjMetadata(version="0.95"),
    )


# --------------------------------------------------------------------------- #
# Entry point                                                                 #
# --------------------------------------------------------------------------- #


def main(argv: list[str] | None = None) -> int:
    """Launch the Gradio server on ``127.0.0.1:7860``.

    Args
    ----
    argv
        Optional CLI args. Recognised: ``--share`` (Gradio's public URL),
        ``--no-browser`` (don't auto-open browser).
    """
    args = list(argv if argv is not None else sys.argv[1:])
    share = "--share" in args
    inbrowser = "--no-browser" not in args
    ui = build_ui()
    ui.launch(
        server_name="127.0.0.1",
        server_port=7860,
        share=share,
        inbrowser=inbrowser,
        show_error=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "APP_TITLE",
    "ConversionResult",
    "UI_VERSION",
    "build_ui",
    "convert_batch",
    "convert_one",
    "main",
    "make_fbx_only",
]
