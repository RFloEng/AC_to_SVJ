"""Tests for the per-car batch conversion (``batch_convert.convert_one``), alone and in
worker processes.  Uses the synthetic ``test_car`` folder only."""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import batch_convert as B

HERE = Path(__file__).parent


def _job(src: Path, out: Path, **kw) -> dict:
    job = {"src": src, "car_out": out, "include_plots": False, "convert_glb": False,
           "include_skins": False, "mesh_sub": "meshes", "include_damage": False,
           "highest_lod_only": False}
    job.update(kw)
    return job


def _svj_of(out: Path) -> dict:
    (f,) = list(out.glob("*.svj.json"))
    d = json.loads(f.read_text(encoding="utf-8"))
    d.pop("_metadata", None)                   # carries the run's timestamp
    return d


def test_convert_one_writes_svj_and_log_and_streams_lines():
    if not (HERE / "test_car").is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        out = Path(t) / "out" / "car"
        seen: list[str] = []
        r = B.convert_one(_job(HERE / "test_car", out), emit=seen.append)
        assert r["status"] == "ok" and r["stem"] and r["error"] is None
        assert seen == r["ui"] and any(l.startswith("✓ Done") for l in seen)
        assert (out / f"{r['stem']}.svj.json").is_file()
        log = (out / "conversion_log.txt").read_text(encoding="utf-8")
        assert "CONVERSION LOG" in log and r["stem"] in log


def test_convert_one_reports_a_car_without_data_and_an_error():
    with tempfile.TemporaryDirectory() as t:
        empty = Path(t) / "nothing"
        empty.mkdir()
        r = B.convert_one(_job(empty, Path(t) / "o1"))
        assert r["status"] == "skipped" and r["name"] == "nothing"
        assert (Path(t) / "o1" / "conversion_log.txt").is_file()
        # a job that cannot run: the error is returned, not raised
        bad = _job(empty, Path(t) / "o2")
        del bad["mesh_sub"]
        if (HERE / "test_car").is_dir():
            bad["src"] = HERE / "test_car"
            r = B.convert_one(bad)
            assert r["status"] == "error" and r["error"][0].startswith("KeyError")
            assert r["csv"] and r["csv"][0].endswith("ERROR")


_POOL = """
import sys, json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
sys.path.insert(0, {here!r})
import batch_convert as B

if __name__ == "__main__":
    out = Path({out!r})
    jobs = [dict(src=Path({car!r}), car_out=out / n, include_plots=False, convert_glb=False,
                 include_skins=False, mesh_sub="meshes", include_damage=False,
                 highest_lod_only=False) for n in ("a", "b")]
    with ProcessPoolExecutor(max_workers=2) as pool:
        res = list(pool.map(B.convert_one, jobs))
    print(json.dumps([r["status"] for r in res]))
"""


def test_worker_processes_give_the_same_output_as_one_car_at_a_time():
    car = HERE / "test_car"
    if not car.is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        seq = Path(t) / "seq"
        B.convert_one(_job(car, seq))
        pool = Path(t) / "pool"
        code = _POOL.format(here=str(HERE), out=str(pool), car=str(car))
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                           encoding="utf-8", timeout=300)
        assert p.returncode == 0, p.stderr[-800:]
        assert json.loads(p.stdout.strip().splitlines()[-1]) == ["ok", "ok"]
        ref = _svj_of(seq)
        assert _svj_of(pool / "a") == ref and _svj_of(pool / "b") == ref
        shutil.rmtree(pool)


ALL_TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def run() -> bool:
    ok = True
    for fn in ALL_TESTS:
        try:
            fn()
            print(f"  ✓  {fn.__name__}")
        except Exception as e:                    # noqa: BLE001
            ok = False
            print(f"  ✗  {fn.__name__}: {type(e).__name__}: {e}")
    return ok


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
