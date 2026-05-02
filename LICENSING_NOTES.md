# Licensing Notes

This document helps you choose a license for the AC → SVJ Converter before
public release. It is **informational, not legal advice**. For anything
mission-critical, consult a lawyer in your jurisdiction.

---

## 1. What you own

Every file in this repository was authored originally for this project.
There is no pasted third-party source code. Your copyright covers:

- `converter.py`, `ac_parsers.py`, `acd_reader.py` (car-data loader),
  `tire_lab.py`, `smoke_test.py`
- `test_car/` (synthetic reference data you wrote)
- `README.md`, `CHANGELOG.md`, `THIRD_PARTY_NOTICES.md`, this file
- `examples/mx5_nd_club.svj.json` (generated from your synthetic test car)

You can license this code under any terms you like.

## 2. What comes from elsewhere (and how it affects your choice)

### 2.1 Pacejka MF 5.2 formulas

Published in an academic textbook. Formulas are facts, not copyrightable.
Attribute in `THIRD_PARTY_NOTICES.md` (already done).

### 2.2 SAE J670 coordinate convention

Public engineering standard. Free to follow.

## 3. Python dependencies

None of the packages in `requirements.txt` is copyleft. You can release
under any license (permissive, copyleft, or proprietary) without issue.

| Package | License | Copyleft? | Can you bundle? | Can downstream relicense? |
|---------|---------|-----------|-----------------|---------------------------|
| gradio  | Apache 2.0 | No | Yes (with NOTICE) | Yes |
| numpy / scipy | BSD-3 | No | Yes | Yes |
| matplotlib | BSD-style | No | Yes | Yes |
| Pillow | HPND | No | Yes | Yes |
| socksio | ISC | No | Yes | Yes |

## 4. Your license options — pick one

The table below compares the options most relevant to a developer tool like
this. Every row assumes you remain the copyright holder.

| License | Commercial use | Modifications | Downstream relicense | Patent grant | File size |
|---|---|---|---|---|---|
| **MIT** | ✅ | ✅ | ✅ (any terms) | ❌ implicit | ~1 KB |
| **Apache 2.0** | ✅ | ✅ | ✅ | ✅ explicit | ~11 KB |
| **BSD-3-Clause** | ✅ | ✅ | ✅ | ❌ implicit | ~1.5 KB |
| **MPL 2.0** | ✅ | Must share *modifications to MPL files* | Yes, for non-MPL files | ✅ explicit | ~17 KB |
| **LGPL v3** | ✅ | Must share lib modifications | Non-LGPL files yes | ✅ | ~25 KB |
| **GPL v3** | ✅ | Entire derivative must be GPL | ❌ | ✅ | ~35 KB |
| **AGPL v3** | ✅ | Derivatives + network services must be AGPL | ❌ | ✅ | ~35 KB |
| **CC0 / Unlicense** | ✅ | ✅ | ✅ | ❌ | ~1 KB |

### When to pick what

- **MIT** — Pick this if you want the widest possible adoption (including by
  commercial studios and proprietary research teams) with the simplest
  legal footprint. It's the most common choice for developer tooling and
  matches Content Manager's permissive approach. *This is the pragmatic
  default.*

- **Apache 2.0** — Pick this if you expect corporate contributors or are
  worried about patent claims. The explicit patent grant protects you if
  someone tries to assert a patent after contributing. Slightly more
  paperwork than MIT but universally respected.

- **BSD-3-Clause** — Virtually identical to MIT in effect. Pick this only
  if your ecosystem uses it (e.g. numpy/scipy). No real advantage for a
  new project.

- **MPL 2.0** — Pick this if you want modifications to this specific tool
  to remain open, but don't want to force projects that *use* it (as a
  library or via CLI) to be open. File-level copyleft is the best
  "balanced" choice.

- **LGPL v3** — Library-focused copyleft. Overkill for a CLI/GUI tool —
  skip it.

- **GPL v3 / AGPL v3** — Strong copyleft. Pick these only if your goal is
  to force downstream projects open-source. Be aware: commercial studios
  will avoid GPL code entirely for their production pipelines.

- **CC0 / Unlicense** — Public-domain dedication. Only pick if you want to
  formally abandon copyright. Some jurisdictions (Germany, France) don't
  fully recognize public-domain dedication; CC0 has better international
  enforceability than the Unlicense.

### My recommendation

For an open-source AC modding / vehicle-dynamics tool intended for the
widest possible community reach — including commercial sim-racing studios,
academic researchers, and hobbyist modders — **MIT** is the strongest
default. It's short, unambiguous, compatible with everything, and imposes
the lowest possible friction on adoption.

If you anticipate contributions from larger organizations who need patent
clarity, upgrade to **Apache 2.0**. The only practical difference for you
is slightly more boilerplate in file headers; adoption is unaffected.

If you specifically want to prevent commercial forks from going closed,
**MPL 2.0** is the right balance point — it keeps the tool open without
blocking commercial consumers.

## 5. How to apply the license you choose

1. Replace `LICENSE` in this folder with the full text of your chosen
   license (copy verbatim from <https://choosealicense.com> or
   <https://spdx.org/licenses/>).
2. Add a one-line SPDX tag to each source file header, e.g.:
   ```python
   # SPDX-License-Identifier: MIT
   # Copyright (c) 2026 <YOUR_NAME_OR_HANDLE>
   ```
3. Add a `## License` section at the bottom of `README.md` pointing to
   `LICENSE`.
4. If you choose Apache 2.0, also keep a `NOTICE` file with attribution
   for any futu