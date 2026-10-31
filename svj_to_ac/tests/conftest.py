"""pytest fixtures. Adds the AC_to_SVJ modules (the repository root) to `sys.path`."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
# AC_to_SVJ lives one folder up (the same repository): `ac_parsers`, `tire_lab`, ... are
# importable as top-level modules for the ground-truth parsers.
_AC_TO_SVJ = _REPO_ROOT.parent
if _AC_TO_SVJ.is_dir() and str(_AC_TO_SVJ) not in sys.path:
    sys.path.append(str(_AC_TO_SVJ))
