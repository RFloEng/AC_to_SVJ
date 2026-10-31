"""ac_writers — one writer module per AC data file.

Every writer module exposes the same public API::

    def write(doc: SvjDoc, out_dir: Path) -> Path: ...
    def _render(doc: SvjDoc) -> str: ...

Conventions (all enforced by ``_format_value`` and the test helpers):

* Section headers and keys are **upper case**.
* Comments use ``;``. ``#`` and ``//`` are tolerated by the parser but
  we never emit them.
* No spaces around ``=`` (Kunos convention; some mod tools choke on
  ``KEY = value``).
* LF line endings, UTF-8 without BOM. INI parsers accept Latin-1 too,
  but the project sticks to UTF-8 across the board.
* Trailing newline at end-of-file.
* Floats render with ``.`` decimal separator, never ``,``. Integers
  render without a fractional part.
* Vec3 values use ``"x, y, z"`` (comma-space) to match Kunos's hand-
  edited files; the parser also accepts ``|`` and tabs.

These rules are codified in :func:`format_value`, :func:`format_section`
and :func:`format_vec3` — every concrete writer composes its output via
those helpers and never hand-rolls formatting.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


# --------------------------------------------------------------------------- #
# Formatting primitives -- the *only* place INI text is shaped                #
# --------------------------------------------------------------------------- #


def format_value(v: Any) -> str:
    """Render a single scalar value for an AC ``KEY=VALUE`` line.

    * ``bool`` -> ``"1"`` / ``"0"`` (Kunos convention; the parser also
      accepts ``TRUE``/``FALSE`` but we standardise on the numeric form).
    * ``int``  -> bare decimal.
    * ``float`` -> ``int(v)`` if exact, else Python ``repr`` (shortest
      round-trip).
    * ``str`` -> verbatim, with surrounding whitespace stripped; if the
      string contains a comma we let it through (used by ``TAGS``).
    * ``None`` -> empty string; callers should normally skip emission.
    """
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if v == int(v) and abs(v) < 1e15:
            return str(int(v))
        return repr(v)
    return str(v).strip()


def format_vec3(v: Sequence[float] | str | None) -> str:
    """Render a 3-tuple as ``"x, y, z"``. Strings are passed through after
    light normalisation. ``None`` returns the empty string."""
    if v is None:
        return ""
    if isinstance(v, str):
        # Trust caller-formatted strings but tidy separators.
        return v.strip()
    if len(v) != 3:
        raise ValueError(f"vec3 must have 3 components, got {len(v)}")
    return f"{format_value(float(v[0]))}, {format_value(float(v[1]))}, {format_value(float(v[2]))}"


def format_section(name: str, items: Mapping[str, Any] | Iterable[tuple[str, Any]]) -> str:
    """Render ``[NAME]\\nKEY=value\\n...`` (no trailing blank line)."""
    lines = [f"[{name.upper()}]"]
    pairs = items.items() if isinstance(items, Mapping) else items
    for key, val in pairs:
        if val is None:
            continue
        lines.append(f"{key.upper()}={format_value(val)}")
    return "\n".join(lines)


def join_sections(sections: Iterable[str]) -> str:
    """Concatenate rendered sections with a single blank line between them,
    finishing with a trailing newline."""
    return "\n\n".join(s.rstrip() for s in sections) + "\n"


def write_text(path: Path, text: str) -> Path:
    """Write ``text`` to ``path`` with UTF-8, LF endings, no BOM."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")
    return path


# --------------------------------------------------------------------------- #
# AC-native pass-through helpers                                              #
# --------------------------------------------------------------------------- #


def get_ac_native_ini(doc, name: str) -> Mapping[str, Mapping[str, Any]] | None:
    """Look up an AC-native section dict by INI name.

    The SVJ ``x_assettocorsa`` extension MAY carry a pre-parsed copy of
    the original AC INI sections under
    ``x_assettocorsa._ac_native["<name>"]`` (mirroring the tyre
    ``_ac_native`` convention from ``02_PROJECT_SPEC.md §9``). When
    present this is a perfect round-trip source. When absent the writer
    must fall back to canonical SVJ fields.

    Parameters
    ----------
    doc
        :class:`svj_loader.SvjDoc`.
    name
        INI filename without the ``.ini`` suffix (e.g. ``"car"``,
        ``"brakes"``, ``"aero"``).
    """
    if doc.assetto is None:
        return None
    native = doc.assetto.raw.get("_ac_native") if doc.assetto.raw else None
    if not isinstance(native, Mapping):
        return None
    block = native.get(name)
    if not isinstance(block, Mapping):
        return None
    return block


def render_sections_from_native(native: Mapping[str, Mapping[str, Any]]) -> str:
    """Render an entire AC-native section dict back to INI text."""
    rendered = [format_section(sec, kv) for sec, kv in native.items()]
    return join_sections(rendered)


__all__ = [
    "format_section",
    "format_value",
    "format_vec3",
    "get_ac_native_ini",
    "join_sections",
    "render_sections_from_native",
    "write_text",
]
