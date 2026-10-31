# SVJ_to_AC — Roadmap

Status as of v0.1.0 (May 2026): every check in `smoke_test.py` passes
(53/53), the full pytest suite passes (163/163), and the converter
produces a Content Manager-ready `content/cars/<model>/` folder plus
an optional KsEditor-compilable FBX skeleton.

## Near-term (v0.1.x bugfix releases)

- **Live KsEditor verification.** The ASCII FBX 2014 output passes
  structural checks here but has not been opened in KsEditor on a
  Windows machine. First user report wins.
- **Mark `pyproject.toml` test basetemp Synology-safe.** Add a
  conditional `addopts = --basetemp=...` so users on cloud-synced
  drives don't need to remember the flag.
- **Real-car batch corpus.** Run the converter over a folder of
  AC_to_SVJ outputs and shake out per-car edge cases (turbo cars,
  mid-engine layouts, semi-trailing arm rears).
- **Pacejka fit quality regression.** Stage 6's slow path runs but
  doesn't measure R^2. A regression that fits MX-5 tyres without
  `_source_params` and asserts R^2 >= 0.98 on Fx/Fy at standard load
  points would tighten the warranty.

## v0.2 -- coverage and fidelity

- **CSP / SOL extensions** (`car.ini`'s `[GRAPHICS_OFFSET]`,
  per-wheel rolling resistance, dynamic turbo, extended FFB blocks).
  Currently passed through via `_ac_native` only.
- **Multi-compound tyres.** The writer emits `[FRONT_N]/[REAR_N]`
  when more than one front or rear set is present, but the slow path
  hasn't been exercised with > 4 compounds.
- **Skin templating.** Generate `skins/default/` with a blank
  `livery.png` placeholder so Content Manager doesn't flag missing
  skins.
- **Sound bank stub generation.** Currently a static `sfx/GUIDs.txt`.
  Could emit a per-car FMOD GUID set if the user supplies a base bank.
- **Per-axle brake torque inference.** Estimate from `chassis.mass_total`
  and `brakes.bias` when SVJ doesn't carry per-axle data, instead of
  hard-coded defaults.

## v0.3 -- model pipeline

- **KN5 writer research.** Kunos's KN5 binary format is partially
  reverse-engineered (`AC_to_SVJ/kn5_reader.py` decodes it). A writer
  would let SVJ_to_AC bypass KsEditor entirely for purely-programmatic
  cars (rallycross procedural generation, etc.).
- **GLB -> FBX importer.** The forward project exports `.glb` per LOD.
  Reading a `.glb` back and converting to ASCII FBX would let users
  iterate on mesh edits without KsEditor in the loop.
- **LOD generator.** Decimate the body mesh to LOD_B / LOD_C / LOD_D
  silhouettes automatically.

## v0.4+ -- ecosystem

- **acd packer.** Kunos's `.acd` is one-way obfuscated, but Content
  Manager has a documented packer; calling it from the UI would let
  users ship encrypted `data.acd` files without leaving the converter.
- **Round-trip CI.** Set up GitHub Actions to convert a fixed corpus
  forward (AC_to_SVJ) and back (SVJ_to_AC) on every commit and diff
  the result.
- **CSP physics-mod export.** Targeted output that uses Custom
  Shaders Patch's extended INI fields (`extension/*.ini`).

## Out of scope (probably forever)

- **Real-time AC plugin.** SVJ_to_AC is an offline batch tool. A
  live in-sim editor is a different project.
- **Tyre lab simulator.** AC_to_SVJ ships one (`tire_lab.py`). We
  consume it as a library for the Pacejka inverse; no reason to
  duplicate.
- **GUI for the SVJ JSON itself.** Easier to edit JSON in any text
  editor than to build an SVJ schema-aware form.

## How to contribute

1. Pick an item from "near-term" or "v0.2", open an issue first to
   confirm scope.
2. Add or extend the relevant `tests/test_*.py` so the new behaviour
   has a failing test before code lands.
3. Run `python smoke_test.py` and `python -m pytest` locally; both
   must pass before opening a PR.
4. Match the project's house style: small modules, single source of
   truth for coords/units, all paths via `pathlib.Path`.
