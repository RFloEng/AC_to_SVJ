"""Where the user's Assetto Corsa cars are, and the few things the apps remember.

`find_cars_dirs()` looks for ``content/cars`` of every Steam library on the machine;
`load_settings` / `save_settings` keep the folder the user chose (one JSON file in the
user's profile) so every tool of the suite starts from it.
"""
from __future__ import annotations

import json
import os
import re
import string
from pathlib import Path

_REL = Path("steamapps") / "common" / "assettocorsa" / "content" / "cars"


def settings_path() -> Path:
    base = os.environ.get("APPDATA") or os.path.join(os.path.expanduser("~"), ".config")
    return Path(base) / "AC_to_SVJ" / "settings.json"


def load_settings() -> dict:
    try:
        return json.loads(settings_path().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_settings(**values) -> None:
    data = load_settings()
    data.update(values)
    try:
        p = settings_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, indent=1), encoding="utf-8")
    except OSError:
        pass


def _steam_roots() -> list:
    roots: list = []
    for drive in string.ascii_uppercase:
        for tail in ("Program Files (x86)/Steam", "Program Files/Steam", "Steam", "SteamLibrary",
                     "Games/Steam", "Games/SteamLibrary"):
            p = Path(f"{drive}:/") / tail
            if p.is_dir():
                roots.append(p)
    # libraries Steam knows about
    for root in list(roots):
        vdf = root / "steamapps" / "libraryfolders.vdf"
        try:
            text = vdf.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for m in re.finditer(r'"path"\s+"([^"]+)"', text):
            p = Path(m.group(1).replace("\\\\", "\\"))
            if p.is_dir() and p not in roots:
                roots.append(p)
    return roots


def find_cars_dirs() -> list:
    """The ``content/cars`` folders of the AC installations found (Windows / Steam)."""
    out: list = []
    for root in _steam_roots():
        p = root / _REL
        if p.is_dir() and p not in out:
            out.append(p)
    return out
