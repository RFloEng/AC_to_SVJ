# SVJ_to_AC — Known pitfalls and fixes

Re-read before stages 4, 5, 6, and 8. Each entry: symptom → cause → fix.

## Coordinate sign mistakes

**Symptom**: Suspension geometry mirrors left-right, or wheels end up below the road, or the car points backwards on track.

**Cause**: The Y/Z flip between SVJ (Z-down) and AC (Y-up) is easy to get wrong, and pickup X is *inboard-from-wheel* not *centreline-relative*.

**Fix**: All transforms go through `coords.svj_to_ac_point` and `coords.pickup_lateral_to_ac`. Add a round-trip test on every PR: random 3D cloud → SVJ → AC → SVJ → assert max error < 1e-9. For pickups, an extra test: known FL upper wishbone outer point at (svj_x=1.4, svj_y=0.75, svj_z=0.35), half-track 0.78 → AC must equal (0.03, -0.35, 1.4).

## Units mixed inside writers

**Symptom**: Camber values 60× too large; engine redline at 100 RPM; tyre pressure 30 000 psi.

**Cause**: Someone forgot a conversion inline.

**Fix**: All unit conversion lives in `units.py`. Add a `tests/test_no_inline_units.py` that greps the repo for raw `* 57.29`, `/ 0.0254`, `* 0.453592`, etc. outside `units.py` and fails the build.

## INI formatting quirks

**Symptom**: `ac_parsers` parses your output but values come back wrong, or AC fails to load the car.

**Cause**: AC's INI parser is fussy. Sections must be uppercase. Keys must be uppercase. Comments use `;` not `#`. No spaces around `=`. Floats use `.` not `,`. Trailing whitespace breaks some parsers. CRLF on Windows, LF on Linux — AC is fine with either, but be consistent.

**Fix**: `lut_writer.py` and every INI writer use a single `_format_value(v)` helper. Trailing newline on every file. Pick LF universally (`open(path, "w", newline="\n")`). Run AC_to_SVJ's parser as the test oracle — if it accepts your output, AC will.

## Suspension type 3 ambiguity

**Symptom**: Solid axle car simulates with floating roll behaviour.

**Cause**: SVJ allows both `panhard_rod` and `watts_link` to be populated. AC's `suspensions.ini` allows only one.

**Fix**: In `suspensions_ini.py`, when both are present: emit Panhard, write a warning to `conversion_log.txt`, and store the unused Watts geometry in a commented-out block at the bottom of the section so it survives round-trips.

## Pacejka inverse

**Symptom**: Tyres are squirrelly, grip levels off vs the original AC car, lap times 2 s/lap slower.

**Cause**: MF 5.2 fit → AC parametric form is underdetermined. The fitter found a local minimum that matches at fitted load points but diverges elsewhere.

**Fix**: Two-layer strategy. Layer 1: AC_to_SVJ embeds `_ac_native` raw parameters in every compound during forward conversion (separate patch — file a tracking issue). Layer 2: when `_ac_native` is absent, the fallback fitter logs `WARNING: tyres reconstructed via Pacejka inverse — expect grip drift ±5%` to `conversion_log.txt` and to the UI. Never silently approximate.

## ctrl LUT references

**Symptom**: AC loads the car but the turbo doesn't spool, ABS does nothing, etc.

**Cause**: `ctrl_*.ini` files reference a LUT by filename (`LUT=ctrl_turbo0.lut`). If the LUT isn't emitted, AC silently ignores the controller.

**Fix**: `ctrl_files.py` always emits the INI and the LUT as a pair, in the same function call. Test that for every `ctrl_*.ini` in the output, a `ctrl_*.lut` exists.

## KsEditor FBX requirements

**Symptom**: KsEditor opens the FBX, you click "Save kn5", and the car appears with no wheels, or KsEditor crashes silently.

**Cause**: KsEditor expects specific dummy names, Z-up world, cm units, identity rotation on dummies, and one mesh node per wheel — not per axle.

**Fix**:
- Dummy names exactly: `WHEEL_LF`, `WHEEL_RF`, `WHEEL_LR`, `WHEEL_RR` (Wheel L/R F/R format, uppercase, underscore).
- `SUSP_LF/RF/LR/RR` siblings of the wheel dummies, at the same translation.
- Header `FBX 2014` — older FBX versions crash KsEditor on Windows 11.
- Axis system `UpAxis: 1, UpAxisSign: 1, FrontAxis: 2, FrontAxisSign: 1, CoordAxis: 0, CoordAxisSign: 1` (Z-up, Y-forward, X-right).
- `UnitScaleFactor: 1.0` paired with `OriginalUnitScaleFactor: 1.0` and document scale set so 1 unit = 1 cm.
- One material per mesh, default Kunos grey `ksPerPixel` shader.

## data.acd encryption

**Symptom**: User asks why your tool doesn't produce a `data.acd`.

**Cause**: ACD is one-way obfuscated by Kunos. There's no published encryptor.

**Fix**: Don't try. Emit unpacked `data/` only. README must tell users: "Open in Content Manager → Tools → Pack Data to produce `data.acd` if you want to ship encrypted."

## sfx/GUIDs.txt missing

**Symptom**: Car loads but AC crashes when you press the throttle.

**Cause**: AC requires a sound bank reference even for silent cars.

**Fix**: Emit `sfx/GUIDs.txt` with a stub referencing Kunos's default car bank:
```
{a234abcd-1234-5678-90ab-cdef12345678} event:/cars/<model>/engine_ext
{b345bcde-2345-6789-01bc-def123456789} event:/cars/<model>/engine_int
```
Generate fresh GUIDs per car. The car will be silent unless the user later swaps in a real sound bank, but it won't crash.

## Reproducible zips

**Symptom**: Two consecutive conversions of the same SVJ produce different zip hashes.

**Cause**: `zipfile.ZipFile` writes mtimes from the filesystem.

**Fix**: In `pack_car.zip_folder`, walk entries in sorted order and set `ZipInfo.date_time = (1980, 1, 1, 0, 0, 0)`. Test by zipping twice and comparing SHA256.

## Gradio file uploads on Windows

**Symptom**: `python svj_to_ac.py` works locally but uploaded files on Windows have backslash paths that break downstream.

**Cause**: Gradio passes platform-native temp paths.

**Fix**: All path handling goes through `pathlib.Path` from the moment a string enters the function. Never `os.path.join`. Never `str.replace("\\", "/")`.

## Encoding

**Symptom**: AC fails to load the car with a complaint about a character in `ui_car.json`.

**Cause**: AC expects UTF-8 *without* BOM for `ui_car.json` and ASCII or Latin-1 for INI files.

**Fix**: `ui_car_json.write(...)` uses `open(path, "w", encoding="utf-8", newline="\n")`. INI writers use the same. Strip the BOM if Python sneaks one in. Test with a fixture containing accented characters in the brand name.
