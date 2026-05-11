<p align="center">
  <img src="docs/AC-to-svj logo.png" alt="AC to SVJ" width="180">
</p>

# AC → SVJ Converter  ·  beta 0.9

> **Repository:** https://github.com/RFloEng/AC_to_SVJ

Converts **Assetto Corsa** car data into the
[SVJ v0.97 standard vehicle JSON](https://github.com/RFloEng/SVJ-standard-vehicle-json)
— SAE J670 axes, SI units — and runs a virtual Pacejka MF 5.2 bench on every
tyre section.

## Install

```bash
pip install -r requirements.txt
python converter.py
```

Opens the Gradio UI at `http://127.0.0.1:7860`.

## Four tabs

**1 · Upload files** — drop any combination of `.ini` / `.lut` / `.json`
files, get an SVJ and tyre-comparison plots back.

**2 · Single car directory** — enter a full path to a car folder; `data/*.ini`,
`data/*.lut`, `data/ctrl_*.ini`, and `ui/ui_car.json` are read automatically.

**3 · Batch collection** — point at a `cars/` folder, click **Scan** to list
every convertible car with a tick-box, uncheck what you don't need, click
**Convert selected**. Output is a ZIP named
`ac_svj_batch_<date>_conv<ver>_svj<ver>.zip` containing, per car:
`{model}.svj.json`, three Pacejka comparison PNGs, `conversion_log.txt`,
and a root-level `batch_summary.csv` with fit quality per axle/compound.
Cars with an encrypted `data.acd` are listed in `skipped.txt` — unpack them
via Content Manager → Tools → Unpack Data and re-run.

**4 · Tire Lab** — standalone Pacejka MF 5.2 bench: fit from a `tyres.ini`
file, or dial AC parameters by hand.

## Coordinate system

AC uses `(X-right, Y-up, Z-forward)`. SVJ / SAE J670 uses
`(X-forward, Y-right, Z-down)`. Transform: `svj = (ac.z, ac.x, -ac.y)`.

All suspension pickup keys (`WBCAR_*`, `WBTYRE_*`, `LINK_*_CAR`,
`PANHARD_CAR`, `WATTS_CAR`, `STRUT_CAR/TYRE`, `TRAIL_CAR`, `SEMI_TRAIL_CAR`)
share the same lateral convention: the AC X component is an
*inboard-positive* offset from the **wheel centre** — not from the car
centreline. Absolute lateral position = `half_track − X`.
Vertical shift: `chassis_z_offset = −rolling_radius`.

## Suspension type coverage

| AC `TYPE` | SVJ `system_type` | Notes |
|---|---|---|
| `0` / STRUT | `macpherson` | `STRUT_CAR/TYRE` → strut link |
| `1` / DWB | `double_wishbone` | upper/lower wishbone + tie rod |
| `2` / MULTILINK | `multi_link` | upper/lower lateral + toe link |
| `3` / AXLE | `solid_axle` | trailing links + Panhard or Watts |
| `4` / TRAILING_ARM | `trailing_arm` | arm pivot + outer |
| `5` / SEMI_TRAILING | `semi_trailing_arm` | semi-trailing pivot + outer |

## Files

| File | Purpose |
|---|---|
| `converter.py` | SVJ assembly + Gradio UI |
| `tire_lab.py` | AC tyre forward model + MF 5.2 fitter + plots |
| `ac_parsers.py` | All `.ini` / `.lut` parsers |
| `acd_reader.py` | Car-data loader (`data/` or `data.acd` detection) |
| `smoke_test.py` | 62-check regression suite against `test_car/` |
| `test_car/` | Synthetic MX-5 ND2 with every field populated |
| `examples/` | Reference SVJ from real Kunos data (`ks_mazda_mx5_nd`) |
| `DEVELOPMENT.md` | Coordinate conventions, data-source map, contributor notes |
| `ROADMAP.md` | Planned improvements |
| `TIRE_LAB_ROADMAP.md` | Detailed Pacejka extension spec |

## Fit quality (test car)

| Axle | Lateral R² | Longitudinal R² |
|---|---|---|
| Front | ≥ 0.999 | ≥ 0.988 |
| Rear | ≥ 0.999 | ≥ 0.990 |

## Third-party notices

No third-party source code is bundled. Runtime Python dependencies
(`gradio`, `numpy`, `scipy`, `matplotlib`, `Pillow`) are installed via `pip`
and are all permissively licensed. See `THIRD_PARTY_NOTICES.md` for full
credits. The Pacejka MF 5.2 formulas are mathematical facts and not
copyrightable; the implementation is original.

## License

[MIT](LICENSE) — https://github.com/RFloEng/AC_to_SVJ

Intended for use with AC car data you have legally obtained.
