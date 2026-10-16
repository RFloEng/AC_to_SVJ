"""Tests for the cars-folder audit (synthetic folders, no AC content)."""
from __future__ import annotations

import sys
import tempfile
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from car_audit import audit_folder, write_reports
import test_kn5_export as T


def test_audit_classifies_data_and_kn5():
    with tempfile.TemporaryDirectory() as t:
        root = Path(t) / "cars"
        (root / "open_car" / "data").mkdir(parents=True)
        (root / "open_car" / "data" / "car.ini").write_text("[HEADER]\n", encoding="utf-8")
        (root / "packed_car").mkdir()
        (root / "packed_car" / "data.acd").write_bytes(b"x")
        (root / "not_a_car").mkdir()
        (root / "not_a_car" / "readme.txt").write_text("hi", encoding="utf-8")
        T._write(root / "enc_car" / "enc.kn5", T._car_nodes(), encrypted=True)
        (root / "enc_car" / "data").mkdir()
        (root / "enc_car" / "data" / "car.ini").write_text("[HEADER]\n", encoding="utf-8")
        T._write(root / "fixed_car" / "m.kn5", T._car_nodes(), encrypted=True)
        T._write(root / "fixed_car" / "unencrypted" / "m_decrypted.kn5", T._car_nodes())
        (root / "fixed_car" / "data").mkdir()
        (root / "fixed_car" / "data" / "car.ini").write_text("[HEADER]\n", encoding="utf-8")
        for n, log in (("log_failed", "FILE: a.kn5\n[error] Could not resolve a working per-car salt\n"),
                       ("log_fixed", "FILE: a.kn5\n[error] Could not resolve a working per-car salt\n"),
                       ("log_plain", "FILE: a.kn5\n[warn] No CSP v1 envelope was found\n")):
            (root / n / "unencrypted").mkdir(parents=True)
            (root / n / "data").mkdir()
            (root / n / "unencrypted" / "decrypt_log.txt").write_text(
                "header\n" + log, encoding="utf-8")
        (root / "log_fixed" / "unencrypted" / "a_decrypted.kn5").write_bytes(b"k")
        with zipfile.ZipFile(root / "dl.zip", "w") as z:
            z.writestr("zcar/data.acd", b"x")
        rows = {r["name"]: r for r in audit_folder(root, check_kn5=True)}
        assert "not_a_car" not in rows
        assert rows["open_car"]["data"] == "unpacked" and rows["open_car"]["kn5"] == "no_kn5"
        assert rows["packed_car"]["data"] == "needs_unpack"
        assert rows["enc_car"]["kn5"] == "encrypted_no_copy"
        assert rows["fixed_car"]["kn5"] == "encrypted_copy_found"
        assert rows["dl.zip::zcar"]["data"] == "needs_unpack" and rows["dl.zip::zcar"]["source"] == "archive"
        assert rows["log_failed"]["decrypt_log"] == "failed"
        assert rows["log_fixed"]["decrypt_log"] == "recovered"
        assert rows["log_plain"]["decrypt_log"] == "plain"
        assert rows["open_car"]["decrypt_log"] == "no_log"
        # without the KN5 check nothing reads the models
        assert {r["kn5"] for r in audit_folder(root)} == {"not_checked"}


def test_reports_list_the_cars_to_repair_later():
    rows = [
        {"name": "a", "source": "folder", "data": "unpacked", "kn5": "clean", "note": ""},
        {"name": "b", "source": "folder", "data": "needs_unpack", "kn5": "not_checked", "note": ""},
        {"name": "c", "source": "folder", "data": "unpacked", "kn5": "encrypted_no_copy", "note": ""},
        {"name": "x.rar", "source": "archive", "data": "unreadable", "kn5": "not_checked", "note": "x.rar: no 7-Zip"},
    ]
    with tempfile.TemporaryDirectory() as t:
        p = write_reports(rows, Path(t), "S")
        assert p["need_unpack"].read_text(encoding="utf-8").split() == ["b"]
        assert p["kn5_encrypted"].read_text(encoding="utf-8").split() == ["c"]
        assert p["unreadable"].read_text(encoding="utf-8").split() == ["x.rar"]
        assert p["csv"].read_text(encoding="utf-8").count("\n") >= 5
        assert p["counts"] == {"cars": 3, "ready": 1, "need_unpack": 1, "kn5_encrypted": 1,
                                "decrypt_failed": 0, "unreadable": 1}
        # nothing to list: no empty files
        q = write_reports(rows[:1], Path(t) / "o", "S")
        assert set(q) == {"csv", "counts"}


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
