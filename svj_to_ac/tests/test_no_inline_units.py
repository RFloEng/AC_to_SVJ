"""Guard rail: no unit-conversion arithmetic anywhere outside ``units.py``.

Greps the Python source tree (excluding ``units.py`` itself, tests, and
generated artefacts) for the literal numeric constants that pop up when
someone hand-rolls a conversion: 57.29 / 57.2958 for rad-to-deg,
6894 for psi, 273.15 for Kelvin, 0.0254 for inches, 0.453592 for lbs,
60 / (2 * pi) for RPM. Fails the build if any of these appear in code
that isn't ``units.py``.

The rule, from ``05_PITFALLS.md`` 'Units mixed inside writers': "All
unit conversion lives in units.py."
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
UNITS_PATH = REPO_ROOT / "units.py"

# Files / dirs that may legitimately reference numeric constants in
# documentation, version pins, or test fixtures.
EXCLUDE = {
    UNITS_PATH,
    REPO_ROOT / "tests" / "test_units.py",
    REPO_ROOT / "tests" / "test_no_inline_units.py",
    REPO_ROOT / "smoke_test.py",  # lists banned constants as data, not as code
}
EXCLUDE_DIRS = {"examples", "test_car", "assets", ".pytest_cache", "__pycache__"}

# Banned substrings that strongly suggest an inline unit conversion.
# Kept narrow on purpose: we want to flag obvious magic numbers, not
# every appearance of '60' or '1000'.
BANNED = (
    "57.29",     # rad -> deg (truncated)
    "57.2958",   # rad -> deg (full)
    "6894.7",    # psi -> Pa
    "6894.76",   # psi -> Pa (alternative truncation)
    "273.15",    # Kelvin offset
    "0.0254",    # inch -> metre
    "0.453592",  # lb -> kg
    "0.745699",  # hp -> kW
    "0.000145",  # Pa -> psi
    "0.000145038",
    "9.80665",   # g -> m/s^2 (sometimes used in unit code)
)

# Compiled regex for "60 / (2 * pi)" style rad/s -> rpm constructs.
RPM_PATTERN = re.compile(r"60\s*[/*]\s*\(?\s*2\s*\*\s*(math\.)?pi", re.IGNORECASE)


def _python_files() -> list[Path]:
    out: list[Path] = []
    for p in REPO_ROOT.rglob("*.py"):
        if any(part in EXCLUDE_DIRS for part in p.parts):
            continue
        if p in EXCLUDE:
            continue
        out.append(p)
    return out


def test_no_banned_constants_outside_units() -> None:
    offenders: list[tuple[Path, int, str]] = []
    for path in _python_files():
        text = path.read_text(encoding="utf-8")
        for lineno, line in enumerate(text.splitlines(), 1):
            # Strip line comments so we don't flag explanatory prose.
            code = line.split("#", 1)[0]
            for needle in BANNED:
                if needle in code:
                    offenders.append((path, lineno, line))
                    break
            else:
                if RPM_PATTERN.search(code):
                    offenders.append((path, lineno, line))
    assert not offenders, (
        "inline unit conversion found outside units.py:\n"
        + "\n".join(f"  {p}:{n}: {l}" for p, n, l in offenders)
    )


def test_units_py_actually_contains_the_constants() -> None:
    """Sanity check: the constants MUST live somewhere -- in units.py."""
    text = UNITS_PATH.read_text(encoding="utf-8")
    # At least the Kelvin offset, psi factor, rad/deg factor must appear.
    assert "273.15" in text
    assert "6894.757293168" in text
    assert "180.0 / math.pi" in text or "math.pi" in text
