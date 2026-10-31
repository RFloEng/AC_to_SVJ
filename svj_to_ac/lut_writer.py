"""AC two-column LUT (look-up table) writer.

AC's :file:`*.lut` format is one ``x | y`` pair per line, optional ``;``
comments, trailing newline. The parser in
``AC_to_SVJ/ac_parsers.py`` (``parse_lut``) accepts ``|``, tab, comma, and
whitespace delimiters interchangeably; we standardise on the Kunos
``"x | y"`` convention (space-pipe-space) for readability.

Conventions enforced here:

* LF line endings only (``open(..., newline="\\n")``). AC accepts CRLF
  too, but mixing inside one project causes git noise.
* Trailing newline on every file. Some downstream parsers drop the last
  line otherwise.
* No locale-dependent formatting: ``.`` is the decimal separator and
  whole numbers are emitted with at least one digit of fraction when the
  caller passes a float (``1.0`` rather than ``1``); this preserves the
  caller's intent and matches AC's typical hand-edited files.
* Optional single-line ``;`` comment at the top.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

Pair = tuple[float, float] | Sequence[float]


def format_lut(
    pairs: Iterable[Pair],
    header_comment: str | None = None,
) -> str:
    """Render an iterable of ``(x, y)`` pairs to AC LUT text.

    Parameters
    ----------
    pairs
        Iterable of length-2 numeric sequences. Each pair is rendered as
        ``"x | y"`` on its own line.
    header_comment
        Optional ``;``-prefixed comment placed at the top of the file
        (the leading ``;`` is added automatically; do not include it).

    Returns
    -------
    str
        Full LUT text, always ending in ``\\n``.
    """
    lines: list[str] = []
    if header_comment is not None:
        for piece in header_comment.splitlines() or [header_comment]:
            lines.append(f"; {piece}".rstrip())
    for pair in pairs:
        x, y = _unpack_pair(pair)
        lines.append(f"{_format_number(x)}|{_format_number(y)}")
    # AC's hand-written LUTs use spaces around the pipe — match that for
    # readability while keeping the delimiter unambiguous.
    lines = [_pretty(line) for line in lines]
    return "\n".join(lines) + "\n"


def write_lut(
    path: str | Path,
    pairs: Iterable[Pair],
    header_comment: str | None = None,
) -> Path:
    """Write the LUT at ``path``. Returns the resolved :class:`pathlib.Path`."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        format_lut(pairs, header_comment=header_comment),
        encoding="utf-8",
        newline="\n",
    )
    return p


def _unpack_pair(pair: Pair) -> tuple[float, float]:
    if len(pair) != 2:  # type: ignore[arg-type]
        raise ValueError(f"LUT pair must be length 2, got {len(pair)}")  # type: ignore[arg-type]
    return float(pair[0]), float(pair[1])  # type: ignore[index]


def _format_number(v: float) -> str:
    """Render a float for an AC LUT.

    Heuristics: integers (or values that round-trip via ``int``) are
    emitted without a decimal point; everything else uses ``repr``-style
    formatting (Python's shortest float round-trip) so we never lose
    precision needlessly.
    """
    if v == int(v) and abs(v) < 1e15:
        return str(int(v))
    return repr(float(v))


def _pretty(line: str) -> str:
    """Insert spaces around an unspaced ``|`` for readability."""
    if line.startswith(";"):
        return line
    if "|" in line:
        x, y = line.split("|", 1)
        return f"{x} | {y}"
    return line


__all__ = ["Pair", "format_lut", "write_lut"]
