# Changelog

All notable changes are documented here.

## [Unreleased]

### Schema fixes

Found by validating about 100 installed cars against the standard's validator; all of
them now validate.

- `assets.meshes[]` no longer carries a `lod` key (the schema forbids extra keys); the LOD
  stays in the entry's `description`. Mesh ids are cut to `[a-z0-9_]` (a name such as
  `uno_turbo_mk2_1.0.1` was rejected).
- A one-point aero curve (a constant in AC) is written as the same value at `x` and
  `x + 1`; the schema wants two points. An empty curve writes no map.
- A tyre with a sidewall taller than its width (odd mod data) gets no `size_code` and no
  `aspect_ratio`, and a rim that is not 10 to 30 inches no `rim_diameter_code`; the
  section heights stay in `dimensions`.
- Movable-part ids use the standard's canonical names (`hatch` becomes `tailgate`,
  `bonnet` becomes `hood`).

### Unified suspension (SVJ 0.100, spec 25)

- The corner-based `suspension` section is deprecated in the standard. `build_svj` now
  writes the suspension as bodies (`upright_fl`, `upper_wishbone_fl`, `rack`, `wheel_fl`,
  ...), `joint` constraints with stated types, `spring` / `damper` elements and
  `wheel_stations[]` (tyre set, brake, alignment, visual binding), plus
  `anti_roll_bars[]`. `build_svj(unified_suspension=False)` keeps the old section.
  The file declares 0.100; validation drops the deprecation warning.
- Done with the standard's own migration, bundled as `svj_migrate/` (Apache-2.0, see
  `THIRD_PARTY_NOTICES.md`) and run as the last step, so the rest of the converter is
  unchanged. Everything it creates is `x_derived`; what AC does not give (carrier and arm
  masses, steering-wheel ratio) is reported, not invented. Wheel-referenced rates are kept
  as quoted where there are no mounts.
- 40 random installed cars: no error comes from the suspension. Remaining validation
  errors are the known ones (`assets.meshes[].lod`, one-point aero maps, tyre size code).
  Axle-body cars with no link data warn that `axle_rear` is not connected.

### Doors, wipers and other moving parts

- New `ksanim.py` reads `animations/*.ksanim` (per node and frame: quaternion, position,
  scale, in the node's local frame; frame 0 is the rest pose, 30 fps assumed).
- **GLB:** the animations are exported as glTF animations (`car_door_l`, `car_wiper`,
  ...) on the matching nodes, so doors open and wipers sweep in any glTF viewer. An
  animated node switches from a matrix to TRS (same rest pose). Driver-rig bones and
  nodes that never move are left out. `kn5_all_lods_to_glbs(include_animations=False)` /
  `build_svj(include_animations=False)` turn it off; a car without animations exports
  exactly as before.
- **SVJ:** `x_assettocorsa.moving_parts` lists each animated node with its joint:
  `revolute` (hinge: `pivot`, `axis`, `axis_local`, `angle_rad`), `prismatic`
  (`travel_m`) or `general`, in the parent node's frame and composed to the KN5 model
  frame (`pivot_model`, `axis_model`; before GRAPHICS_OFFSET). For example the
  doors of one car turn 70 degrees about the node's local X axis through its origin.
- **SVJ movable parts (spec 24, now declared as SVJ 0.100):** each hinge or slide becomes a
  `chassis.mass_bodies[]` entry (`x_derived`, mass 0 because AC gives no mass; `visual`
  bound to the GLB node `SVJ::body::<id>`), a marker on the chassis and on the body at
  the pivot (marker +Z = the axis, in the SVJ frame, placed exactly as the mesh is), a
  `revolute` / `translational` joint `hinge_<part>` / `slide_<part>` ranging from the rest
  pose to the end of the animation, and one pose per animation (`closed` plus
  `car_door_l`, ...). Ids follow the naming convention (`DOOR_LF` is `door_fl`). The SVJ
  validator and the binding check pass; its only new warning is the missing mass.
  Suspension, hub, wheel and steering animations are not movable parts and stay in
  `x_assettocorsa.moving_parts`. The KN5 model has X to the left (`WHEEL_LF` is +X), the
  SVJ frame has Y to the right.

### Speed

Output is unchanged (GLBs byte-identical, SVJ identical apart from the timestamp).

- **Tire plots are only rendered when wanted.** They were drawn (about 2 s per car)
  even with "Include tire comparison PNGs" unticked; `build_svj(include_plots=False)` /
  `run_bench(plots=False)` skip them: 2 s to 0.2 s per car for the SVJ part.
- **Uncompressed DDS textures decode about 14x faster** (numpy instead of Pillow's
  per-pixel Python loop; verified byte-identical on 217 textures and synthetic masks).
- **DDS to PNG results are cached** by content, so the LODs and skins of a car that
  share a texture convert it once.

### Numbers with a stray second decimal point

- `WHEELBASE=2.58.0`, `CG_LOCATION=0.59.0`, `RADIUS=0.281.5` ... (typos in some mods;
  AC itself reads the leading number) were rejected, so the converter fell back to a
  2.6 m wheelbase / 50 % weight split and the mesh was displaced from the physics
  frame (gr2_simca_rallye2: wheels 0.22 to 0.25 m off). They now read as 2.58, 0.59,
  0.281. About 20 cars in a large install carry such values (suspensions.ini,
  tyres.ini); regenerate them.

### Crash normal maps no longer baked into the intact car

- Many mods (about 45 % of the damage-shader materials in a large install) put the
  dent normal map (`DAMAGE_NM.dds`, `normaldamage.dds`, `DAMAMGE_NM.dds`...) in the
  `txNormal` slot of the `*_damage*` body-paint shader; the game fades it in with the
  damage mask, but the exported GLB showed the dents on the intact car. Such a normal
  map is now skipped (and left out of the GLB) unless "Include crash/damage textures"
  is ticked. `nodamage_NM`-style names are the intact normal map and are kept.
  Cars converted before this need their GLBs regenerated.

### Audit reads decrypt logs

- The cars-folder audit now reads `unencrypted/decrypt_log.txt` (written by a KN5
  decrypt tool): new `decrypt_log` column (`failed` / `recovered` / `plain` /
  `no_log`) and a `cars_kn5_decrypt_failed_*.txt` list of cars whose protected KN5
  was not recovered. Text files only, so it is fast (no KN5 reading).

### SVJ v0.99.2 alignment

- **Target version** `0.97` → `0.99.2`.
- **Visual bindings** extended to the v0.99.2 categories: wheels
  (`SVJ::wheel::wheel_fl`) and brake discs (`SVJ::brake::disc_fl`) are bound
  alongside the chassis and uprights, and uprights now use the preferred
  `SVJ::suspension::upright_fl` form (the legacy `SVJ::body::upright_fl` is
  still valid in the standard). New `map_ac_nodes_to_svj_parts()`; the old
  `map_ac_nodes_to_svj()` is unchanged.
- **GLB nodes are renamed to the bound names** (`kn5_to_glb(node_names=...)`,
  original AC name kept in `extras.ac_name`). Previously the SVJ file named
  `SVJ::body::chassis` etc. but the exported GLB still used raw AC names
  (`BODY`, `WHEEL_LF`), so the bindings pointed at nodes that did not exist.
- **`vehicle_info.drive_type`** is now `FWD` / `RWD` (derived from the layout
  code) instead of the invalid `FR`; the layout code stays in
  `powertrain.layout`.
- `smoke_test.py`: the stale `version == "0.95"` check now compares against the
  converter's `SVJ_VERSION`, and the Pacejka checks read the tyre-set layout the
  converter actually emits.
- Known gap: `suspension.<station>.wheel.rim_width` is required by the schema
  but AC does not store a rim width, so the converter still omits it (4 schema
  errors on any car).

### Audit a big cars folder: which cars need unpacking or repair

"Audit…" (batch tab) classifies every car folder, and every car inside the `.zip` /
`.rar` / `.7z` files next to them: `data` is `unpacked`, `needs_unpack` (only an encrypted
`data.acd`) or `no_data`; with "also check KN5 encryption" (reads the model files, slow)
`kn5` is `clean`, `encrypted_copy_found`, `encrypted_no_copy` or `no_kn5`. It writes
`cars_audit_<stamp>.csv` with every car and plain lists, one name per line, of the cars
that need attention (`cars_need_unpack_*.txt`, `cars_kn5_encrypted_*.txt`,
`cars_unreadable_archives_*.txt`) to the output folder (the cars folder's parent when
none is set). Detection only; nothing is decrypted. New `car_audit.py`, `test_audit.py`.

### Second log: failed cars only

A batch with failures now also writes `batch_failed_<stamp>.log` (a summary line per
failed car, then the full traceback of each) and `batch_failed_<stamp>.txt` (one car
name per line) beside the master log. Nothing is written when no car failed.

### Option: skip cars already in the output folder

"Skip cars already in the output folder" (off by default) leaves out a car whose output
folder already holds its `.svj.json` (and a GLB when GLBs are requested), so an
interrupted batch can be resumed. The scan also shows how many cars the output folder
already has. It is off by default because a re-run is how fixes reach existing files.

### CSP extended-physics switches kept

Every `[_EXTENSION*]` section of the data files (`TORQUE_MODE_EX`, `FIX_PROGRESSIVE_RATE`,
`USE_DWB2`, tyre rays and rolling-resistance options, `USE_ADVANCED_SYSTEM` for the brakes,
`CLUTCH_LUT`, flex) and the turbo `EXT_*` keys are now stored as written under
`x_assettocorsa.csp_physics`, so none of the 423 cars that use them loses an option.

### CSP COSMIC suspension geometry

`TYPE=COSMIC` axles (11 installed cars, 23 axles) were left without links. They are a
small multibody model (bodies, distance / ball / hinge joints, sliders, coilovers) and
are now reduced to the SVJ's chassis-to-upright links by the new `cosmic.py`:

- **Setup adjusters** (a slider plus a distance joint on the same two bodies and
  points, used for camber / caster / toe) and **bushings** (a slider with a spring or
  damper) merge the two bodies into one rigid body.
- **Rods** (a distance joint or the steer joint from the chassis to the hub) become rod
  links; two rods ending at the same hub point become one A-arm. A body held by two ball
  joints and joined to the hub by a third is an A-arm; a body held by one ball joint
  and sliding on the hub is a strut. The result is `double_wishbone`, `macpherson` or
  `multi_link`.
- **Springs and dampers:** COSMIC rates are per component, not wheel rates. The
  coilover's mounts give the damper axis, its motion ratio comes from the geometry (strut
  inclination, or the lever of the lower arm about its pivot axis), and the wheel rate is
  `rate x ratio^2`, plus the one-metre vertical "virtual" springs these cars use for
  pure wheel rate. The damper tables of `dampers.ini` are damper-level and now carry the
  real ratio and mounts.
- Anything not recognised (for example the BRM Type 25 de Dion with a Watts linkage) is
  listed in `topology._note` and left out, not guessed. The motion ratio assumes the
  wheel travel equals the lower ball joint's vertical travel, so it is approximate.
- Also fixed: a negative engine `MINIMUM` (a mod typo, `-9000`) is no longer taken as the
  idle speed.

### CSP motion-ratio tables, and anti-roll bars that were never read

- **`MOTION_RATIO`** (CSP DWB2, 4 cars): the deflection | ratio table (a file or an
  inline `(|x=y|...)` table) is kept as `x_motion_ratio_curve` on the spring and the
  damper, and its value at the zero point becomes the scalar `motion_ratio`. The spring
  `rate` stays the wheel rate. CSP applies the ratio to damper tables, so damper tables
  on such a car are now `reference: "damper"` with that ratio (synthesised wheel-level
  dampers keep ratio 1 and only carry the curve).
- **Fix: anti-roll bars.** The converter looked for the bar in the axle sections, but
  AC keeps it in the global `[ARB]` section (`FRONT`, `REAR`; CSP `EXTEND=1` adds
  `FRONT_MOTION_RATIO` / `REAR_MOTION_RATIO`). Every car therefore got the placeholder
  8000 / 6000. The real values are now read; `arb.motion_ratio` is set when given. The
  number is written as AC gives it (documented as N/m); it is not converted to the
  SVJ's Nm/rad, and the corner says so in `arb._note`. Cars without an `[ARB]` section
  keep the flagged placeholder.
- A bump-stop table that does not start at 0 | 0 and grow with the deflection is
  ignored (one car writes a three-point table of 1 / 0 / 1).

### CSP extended physics: real damper and bump-stop tables

Cars that use Custom Shaders Patch extended physics ship lookup tables instead of
AC's scalar rates. They were ignored, and the damper curves were synthesised.

- **`dampers.ini`** (24 installed cars): force against speed for the bump and rebound
  of each axle, with slow-speed tables continued by the fast-speed ones. They replace
  the synthesised `bump_curve` / `rebound_curve`, and the damper is no longer flagged
  `_est`. The setting used is the `DEFAULT` of the matching `setup.ini` damper entry
  (lowest otherwise); all available settings are listed in `damper.x_settings`.
  Sub-folder references (`\dampers_asymmetric\...`) are resolved, and never outside
  `data/`.
- **Which level the curves are at:** wishbone cars without a motion-ratio table
  are wheel level (`damper.reference = "wheel"`, motion ratio 1). COSMIC coilover cars
  (`[FRONT_C0]` sections) are component level (`reference = "damper"`); their geometry
  is not converted yet, so no motion ratio is derived for them.
- **Bump stops** (`bumpstops.ini`, or the older `BUMP_STOP_LUT`): the rubber's force
  against deflection becomes `bump_stop.rate_curve`, with `PACKER_RANGE` as the gap
  (the CSP meaning for corner springs). `stack` multiplies the force. When no usable
  gap exists the curve is kept in `static_setup.x_bump_stop_curve`.
- Not yet: motion-ratio tables (`MOTION_RATIO`), COSMIC geometry, the other
  `[_EXTENSION]` flags.

### Cars straight from .zip / .rar / .7z files (no installation)

`.rar` and `.7z` are read through the 7-Zip command line (install 7-Zip; `7z.exe` is
found on PATH or in its default folder; nothing is bundled). A password-protected
archive or a missing 7-Zip is reported in the log instead of failing silently.

Point "AC cars folder" at a folder of downloaded mods: the archives directly inside
it are scanned and each car found shows as `<zip>::<car folder>` next to the installed
ones. A zip may hold several cars and the car folder may sit at any depth
(`content/cars/<car>/...`) or at the zip root (then it is named after the zip). Only
what the converter reads is unpacked into a scratch folder, per car, and deleted right
after: `data/`, `ui/`, the `.kn5` models (including an `unencrypted/` copy and LODs)
and skin images; sound banks, animations and `extension/` are skipped. Entries that
would land outside the scratch folder are ignored and size limits apply. New `car_archives.py`, `test_archives.py`.

### Option: highest-quality LOD only

`highest_lod_only` (GUI: "Highest LOD only"; `build_svj` / `kn5_all_lods_to_glbs`)
exports only LOD A and leaves out LOD B-D, in the GLBs and in `assets.meshes`.

### Targets SVJ v0.99.4 (strut ends, mount names) and two schema fixes

- **Strut:** the strut link now ends at `hardpoints.strut_lower` (the lower clamp
  on the upright, `STRUT_TYRE`); `strut_top` is the body-side end (`STRUT_CAR`).
  Before, both ends were the same hardpoint family and the link had no usable length.
- **Spring / damper mounts:** only `inboard_mount` / `outboard_mount` are written;
  `inboard_point` / `outboard_point` are now documented misspellings and are gone.
- **`wheel.rim_width`** (required by the schema, absent from AC): estimated as
  0.8 x the tyre section width and flagged `_est_rim_width`.
- **`static_setup.ride_height`**: AC's `BASEY` is a wheel-centre offset, not a ride
  height, and is negative, which the schema rejects. Omitted unless positive.
- Cars with simple geometry (for example a double-wishbone / solid-axle one) now
  pass the SVJ validator with no errors.

### Suspension topology that a kinematic solver can use

Found by reading all 1,398 converted files with an independent suspension solver.

- **Solid axles** (`TYPE=AXLE`, 91 rear axles): were two wishbone-style arms plus a
  toe link under a `solid_axle` label, which over-constrains the axle. Now one
  `upper_link` and one `lower_link` rod per side (chassis end = midpoint of the two
  AC pivots, flagged `_est`), `topology.axle_body` shared by both sides, no toe link.
- **MacPherson** (`TYPE=STRUT`): the `strut` link is now type `strut` ending at
  `hardpoints.strut_top`, and a stray upper arm from boilerplate `WBCAR_TOP_*` keys
  is dropped. The spring and damper use the strut's real mounts
  (`STRUT_CAR` -> `STRUT_TYRE`); the motion ratio is the strut's vertical share
  (0.81-1.00 on the installed cars) and the damper curves are converted to the
  strut (`v * MR`, `F / MR`).
- **`STRUT_*` on a wishbone axle** is no longer a `damper_link` rod (a damper does
  not locate the wheel); the points are kept in `damper.x_ac_strut`.
- **Corners without geometry** (`COSMIC`, `ML` and `AXLE` without pickups; 35
  corners) no longer get a made-up `virtual` link: `links` is empty with a
  `_note`. The schema asks for at least one link, so these corners fail that one
  rule until the standard allows an empty list.
- A missing steering tie rod or rear toe link is reported in `topology._note`
  (for example an unreadable `WBTYRE_STEER` value).
- Spring and damper mounts are written as `inboard_mount` / `outboard_mount`
  (spec 9.3 / 9.4); the old `inboard_point` / `outboard_point` stay for now.

### Targets SVJ v0.99.3 (differential control)

The standard adopted the proposal, so the extension keys became real fields:
`center_diff.lock_control` (was `x_lock_control`) and `transfer_case.coupling`
(was `x_coupling`, now only `ramp_torque` / `max_torque`; no `type`). AC's `FILTER`
is a smoothing coefficient, not seconds, so it is kept as `x_filter` per stage
instead of `filter_time_constant`. Stages without a usable curve are dropped. All
1,398 installed cars still give a schema-valid powertrain block (v0.99.3 schema).

### Engine layout: MR / RR instead of FR for every rear-driven car

AC has no engine position, so every RWD car came out as `FR`. The layout is now
inferred from the static front weight (`CG_LOCATION`): below 46 % on the front axle
-> `MR`; known rear-engined families (Porsche 911 line, RUF, Singer, Fiat 126,
Alpine A110) -> `RR`. Heuristic: the log says when it was applied and why.

### Drivetrain: AWD structure and the settings that were dropped

Audited against the 1,398 installed cars. Before, an AWD car came out with one
`diff_front` carrying the generic `[DIFFERENTIAL]` values and nothing else.

- **AWD is now described.** `[AWD]` (459 cars) gives a `front` and a `rear`
  differential (own lock under power / coast and preload), a `transfer_case`
  (`full_time`) with `torque_split = [front, 1 - front]` and a `center_diff`.
  `FRONT_SHARE` is read as a percentage (47) or a fraction (0.4). A leftover
  `[AWD]` section on a RWD/FWD car is ignored.
- **`TYPE=AWD2`** (35 cars, previously labelled FR/RWD): layout `AWD`,
  `transfer_case.type = on_demand`, no fixed split; the torque-limited centre
  coupling (`CENTRE_RAMP_TORQUE` / `CENTRE_MAX_TORQUE`) goes to
  `transfer_case.x_coupling`.
- **Clutch `MAX_TORQUE`** -> `powertrain.clutch.max_torque`; **reverse ratio**
  (`GEAR_R`, stored positive) -> `gearbox.reverse_ratios`.
- **Autoclutch shift profiles** are found by the name `[AUTOCLUTCH]` gives them
  (they were looked up under a fixed name and lost on 1,370 cars):
  `driver_aids.shift_profiler.{upshift,downshift}`.
- **AWD centre-diff controllers** (`ctr_awd_center_lock.ini`, 102 cars) are kept
  with their LUT curves: `x_assettocorsa.drivetrain.controllers`, and as
  `center_diff.x_lock_control` in the shape proposed for the standard. Also kept:
  `auto_shifter`, `downshift_protection`, extra `[DIFFERENTIAL]` keys.
- Diff type names now use the schema's `lsd_torsen` / `lsd_viscous`.
- All 1,398 cars produce a schema-valid powertrain block (checked against the
  SVJ v0.99.2 schema).
- **What the SVJ cannot express yet** (dynamic differential lock, centre
  coupling): see `proposals/svj_differential_control_proposal.md` in the standard
  repository. The converter already writes those two as `x_lock_control` and
  `x_coupling`, so adopting the proposal is a key rename.
- New `test_drivetrain.py`, run by `smoke_test.py`.

### Exact mesh placement from AC's physics link, and CG fixes

- **Mesh placement uses AC's own link between model and physics**: `car.ini`
  `[BASIC] GRAPHICS_OFFSET` maps the KN5 model into the physics frame (origin at
  the CG). The body is translated by `GRAPHICS_OFFSET` + the physics-origin to
  ground/front-axle shift, and each wheel (with its `SUSP`/`HUB`/`UPRIGHT`/`DISC`
  nodes) is placed individually at its physics wheel centre, as the game does at
  runtime. A wheel whose node is more than 0.15 m from its physics position (a
  model that disagrees with its physics) is left alone and reported.
- **Wheel mesh centre = physics wheel centre.** The tyre mesh keeps its own
  radius, so a tyre slightly larger than the physics radius sinks a little into
  the ground (the loaded-tyre look) instead of being forced to touch at y = 0.
  Without a usable `GRAPHICS_OFFSET` the wheel centres are matched the same way;
  the older tyre-contact alignment below is now only a last resort (no wheel
  centres at all).
- **`GRAPHICS_PITCH_ROTATION` researched, recorded, not applied by default.**
  Fitted on ~700 installed cars: the unit is degrees and a positive value lowers
  the nose (`y' = y - pitch * z`, about the CG) - applying it with the opposite sign
  is clearly worse. But it compensates the **rake** of AC's physics frame (the body
  is tilted against the ground: `lf_punto_abarth` has front/rear ground heights 8 cm
  apart, ~1.8 deg, and pitch 1.5), whereas the SVJ frame is level. Measured as the
  front/rear wheel-height mismatch between the model and the SVJ wheel centres
  (all cars, RMS): not applied 1.35 cm, pitch applied 2.13 cm; for |pitch| >= 1:
  2.38 cm vs 4.94 cm. So the model is left un-rotated; the value is kept in
  `x_assettocorsa.graphics.pitch_rotation_deg`. `build_svj(apply_graphics_pitch=True)`
  rotates the body about the CG (wheels are still placed individually) for anyone
  who wants AC's tilted attitude. The log also shows how far each model's own wheel
  positions were from the physics.
- **Fix: centre-of-gravity position was mirrored.** `center_of_gravity[0]` put the
  CG `front_weight x wheelbase` behind the front axle; it is
  `rear_weight x wheelbase` (a front-heavy car's CG is nearer the front axle),
  as the KN5 + `GRAPHICS_OFFSET` geometry of installed cars confirms.
- **Fix: CG height was the constant 0.42 m for every car.** It is now derived
  from the car's data: `tyre radius - BASEY` per axle, interpolated at the CG
  (an explicit `CG_HEIGHT` still wins; the old default remains only when neither
  exists, and is logged as a default).
- **Fix: numbers with a trailing annotation were rejected** (`WHEELBASE=2.530
  (2.52)` fell back to the 2.6 m default, moving the rear axle and the CG). The
  numeric parsers now accept a leading number followed by whitespace, a
  parenthesis or the end.
- The conversion log reports the placement mode, wheels placed/skipped and the
  mesh's lowest point and height.

### Mesh height: wheels floating or sunk

- **Fix: wheels floating above (or sunk below) the ground when the GLB is
  loaded with the SVJ.** AC models carry an arbitrary vertical offset, while the
  SVJ physics has its ground at z = 0 with each wheel centre one tyre radius
  above it. The exporter only aligned the front axle front-to-back. It now also
  moves the mesh vertically so the lowest tyre vertex (under the `WHEEL_xx`
  nodes; blur discs and other dropped variants ignored) sits on y = 0.
  Measured on installed cars: `av_fiat_punto_s1600` floated 9.0 cm,
  `S2000_Fiat_Punto` sat 12.1 cm low, `lf_punto_abarth` 16.6 cm low.
  Cars without `WHEEL_xx` nodes are left as they are.
- Opt out with `ground_align=False` / `--no-ground-align`. The export report
  shows how far the mesh was moved.

### Output files

- **Fix: conversion failed at the very end for cars whose display name holds
  characters Windows forbids in file names** (e.g. `3.2 (E36) "Regional Rally"`
  -> `OSError: Invalid argument`). Output stems are now filesystem-safe
  (`<>:"/\|?*` and control characters removed, trailing dots/spaces trimmed,
  reserved names such as `CON` rejected, falling back to the folder name); names
  that were already safe are unchanged.
- **Meshes next to the JSON** — new `meshes_subdir` option on `build_svj`
  (default `"meshes"`; `""` puts the GLBs in the same folder as the JSON and
  writes bare `assets.meshes[].uri` values such as `car.glb`) and a GUI checkbox
  "Meshes next to the JSON (no meshes/ folder)".

### Crash / damage textures

- The crashed-state textures (`txDamage`, `txDamageMask` of AC's `*_damage` /
  `*_damage_dirt` shaders, and damage-named maps nothing visible uses) are now
  **left out of the GLB by default**: a static model never shows them, and they
  were being embedded as unreferenced images. A texture that is also used as the
  diffuse (`txDamage` often points at the same file) is kept. Opt back in with
  `include_damage_textures=True` / `--include-damage-textures`. The export
  report says how many were left out. (Damage *meshes* were already dropped,
  see `*_DAMAGE` above.)

### Encrypted KN5s and model detection

- **Encrypted KN5s are detected** (CSP marker; detection only) and never
  exported: `kn5_to_glb` and `list_skins` refuse them with a clear message.
  Before, such a file parsed fine but produced a GLB with placeholder textures
  and meshes.
- **Automatic model selection for any car** — `resolve_car_kn5()` searches the
  top level and subfolders (not `extension/`, `skins/`, `texture/`, …). With an
  encrypted KN5 present, an unencrypted copy of the **same** model (≥ 50 % node
  overlap) is used instead; otherwise the car is refused with an explanation.
  Without one, the previous rule applies (file named like the folder, else the
  largest). LODs and skins follow the chosen file (`skins/` is found from the
  car root even when the KN5 sits in a subfolder).
- `kn5_override` on `build_svj` / `kn5_all_lods_to_glbs`; `kn5_reader.py`
  accepts a car folder. The converter log reports which file was used and which
  were skipped.
- New `docs/ENCRYPTED_CARS.md` explaining the expected folder layout.

### KN5 → GLB exporter (`kn5_reader.py`)

- **Variants** — runtime-variant meshes are now *dropped* (subtree and all)
  instead of being made transparent. Rules are exact: a node is a blur/damage
  variant when a name token is exactly `blur` or `damage` (so `RIM_BLUR_LF`,
  `BODY_DAMAGE` go; `UNDAMAGED_PANEL`, `BENTLEY_BADGE` stay). The low-res
  half of an in-file LOD pair (`COCKPIT_LR`, `STEER_LR`) is dropped **only when
  its `_HR` twin exists**, so `_LR` = Left Rear (`WHEEL_LR`, `SUSP_LR`, …) is
  never touched. New `keep_variants` option on `kn5_to_glb` /
  `kn5_all_lods_to_glbs` and `--keep-variants` on the `kn5_reader.py` CLI.
  Ideas adapted from semiloker/assetto-corsa-gltf (MIT) — see
  `THIRD_PARTY_NOTICES.md`.
- **Livery paint** — the GLB now opens in the colour the game would show. A
  flat `txDetail` map (with `useDetail` set) is the paint on many Kunos cars:
  AC multiplies it into a shared grey diffuse as `diffuse * detail * 2`, so it
  is baked into `baseColorFactor` (doubled in gamma space, clamped, then
  linearised). Base materials resolve against AC's default skin (the first;
  `default_skin="first"|<name>|"none"`), and every skin is still a
  `KHR_materials_variants` entry, now including skins that differ only by
  detail colour. A flat detail map is no longer also exported as AO; patterned
  detail maps keep the `TEXCOORD_1` AO path.
  `list_skins(kn5)` / `python kn5_reader.py car.kn5 --list-skins` print each
  skin and the colour it paints (bodywork first, rims/glass ranked after).
  `kn5_reader.py` CLI is now argparse-based (`--skin`, `--no-skins`,
  `--scan-nodes`, `--keep-variants`, `--list-skins`).
- **Materials** — roughness now comes from the Blinn exponent and intensity
  (`sqrt(2 / (ksSpecularEXP * ksSpecular + 2))`, clamped) instead of
  `1 - ksSpecular`. `fresnelMaxLevel` maps to `KHR_materials_specular`
  (`specularFactor`); materials with `sunSpecular` (car paint) get
  `KHR_materials_clearcoat` from `sunSpecular` / `sunSpecularEXP`. Not done:
  per-pixel roughness from the `txMaps` texture.
- **Robustness** — texture-table entries that differ only in case are folded
  (largest blob wins) and material slots re-pointed, so exports no longer name
  an image that exists only under another capitalisation. Car folder, `skins/`,
  skin texture and LOD file lookups are case-insensitive (Linux/macOS).
  Meshes that use vertex index 65535 (more than 65,535 vertices) are written
  with `UNSIGNED_INT` indices — 65535 is the reserved restart value for
  `UNSIGNED_SHORT` and made such files invalid.
- **Export report** — `kn5_to_glb(report=dict, verbose=bool)` collects node,
  mesh, triangle, material and image counts, dropped-variant count and the
  world-space bounding box/size (final glTF axes, including the axis-fix and
  front-axle nodes). `format_export_report()` renders it; the
  `kn5_reader.py` CLI prints it after every export.
- New `test_kn5_export.py` (synthetic KN5 encoder; no AC content), run first by
  `smoke_test.py`.

## [0.9.1] — 2026-05-07

### SVJ target: 0.97

The converter now targets SVJ standard **v0.97**. The schema enum no longer
accepts `"0.95"`; valid values are `"0.96"` and `"0.97"`. All existing physics
output is fully backward-compatible — no fields removed or renamed.

### New: KN5 → GLB conversion (`kn5_reader.py`)

A new module, `kn5_reader.py`, parses the Assetto Corsa binary `.kn5` 3-D
model format and exports it as a **GLB** (binary glTF) file in the
**SAE J670** coordinate frame (X-forward, Y-right, Z-down) — the same
frame used for all physics data.

Key details:

- Pure-Python parser; no external C tools required.  Uses `pygltflib>=1.15`
  (added to `requirements.txt`).
- Reads node tree, geometry (positions, normals, UVs, indices), textures,
  and materials.  Supports KN5 versions 5 and 6.
- Coordinate transform matches `ac_to_svj()` exactly:
  `SAE = (AC.Z, AC.X, −AC.Y)`.  Triangle winding is reversed to preserve
  front-face orientation when switching from left-handed (AC) to
  right-handed (SAE J670).
- Node-name heuristic maps AC node names (`BODY`, `WHEEL_LF`, `UPRIGHT_LF`, …)
  to SVJ body IDs (`chassis`, `upright_fl`, …).
- DDS textures are converted to PNG on the fly via Pillow before embedding.
- KN5 materials are mapped to glTF PBR:
  `ksAmbient/ksDiffuse` → `baseColorFactor`, `ksSpecular` → `roughnessFactor`.
- `find_car_kn5()` locates the primary LOD (largest `.kn5` whose stem matches
  the car folder name; falls back to any `.kn5` in the car root).

Public API:

```python
from kn5_reader import parse_kn5, scan_kn5_nodes, map_ac_nodes_to_svj, \
                       kn5_to_glb, find_car_kn5
```

### New: `assets.meshes` + `visual` bindings (SVJ 0.97)

When a KN5 file is detected alongside the car data, `build_svj()` now emits:

- **`assets.meshes`** — manifest block referencing the GLB URI
  (`meshes/<car>.glb`).
- **`chassis.visual`** — `{mesh_ref, node: "SVJ::body::chassis"}`.
- **`suspension.<corner>.visual`** — per-corner upright bindings using
  `SVJ::body::upright_fl/fr/rl/rr` naming.

All fields are optional in the schema; if no KN5 is present, the SVJ is
still fully valid.

### Desktop GUI (tkinter)

- Standalone two-tab tkinter application (`gui.py`); no browser required.
- **Batch tab**: scan a `cars/` folder, tick-list selection, **▶ Convert
  selected** and **■ Stop** button, colour-coded log pane with **Save log**
  export.
- Output written to a folder (not a ZIP): one subfolder per car with
  `{model}.svj.json`, `conversion_log.txt`, Pacejka PNG plots, and
  `meshes/{model}.glb` when KN5→GLB is enabled.
- New checkbox *"Convert KN5 → GLB"* in the Batch tab.
- **Tire Lab tab**: three input modes (car folder, bare `tyres.ini`, manual
  parameters); displays five MF 5.2 / MF 6.2 plots plus copy-ready JSON
  blocks.

### Other

- `_metadata.version` bumped to `"0.97"`.
- `pygltflib>=1.15` added to `requirements.txt`.
- README: SVJ version reference updated to 0.97.

---

## [0.9.0-beta] — 2026-05-02

Initial public beta release.

### Suspension geometry
- All six AC suspension `TYPE` codes covered: MacPherson, double-wishbone,
  multi-link, solid-axle, trailing-arm, semi-trailing-arm.
- Correct lateral coordinate convention for chassis-side pickups (`WBCAR_*`,
  `LINK_*_CAR`, `PANHARD_CAR`, `WATTS_CAR`, `TRAIL_CAR`, `SEMI_TRAIL_CAR`):
  the AC X component is an inboard-positive offset from the wheel centre —
  the same convention as `WBTYRE_*`. Verified against MX-5 ND (DWB V-shape,
  strut inclination 14.9° on Audi TT Cup, F2004 keel geometry).
- Correct vertical (Z) reference: `chassis_z_offset = -rolling_radius`.

### Powertrain & vehicle
- Real `power.lut` / `coast_curve.lut` parsing.
- Turbos, BOV, engine damage, driver aids (autoblip, autoclutch, ABS, TC).
- Wheelbase from `suspensions.ini [BASIC]`; CG from `CG_LOCATION` fraction.

### Tyres
- Pacejka MF 5.2 fitter per axle/compound with R² / RMSE metrics.
- Multi-compound `[FRONT_N]/[REAR_N]`, thermal blocks, pressure model,
  relaxation length, rolling resistance, camber thrust, speed sensitivity.

### Aero
- `aerodynamics.components[]` per SVJ 0.95; wing AoA-CL/CD LUTs; DRS.

### UI (tkinter)
- Standalone two-tab tkinter desktop app (`gui.py`).
- **Batch tab**: scan folder → tick-list of cars → convert selected.
  Output folder named `ac_svj_batch_<date>_conv<ver>_svj<ver>/`.
- **Tire Lab tab**: standalone MF 5.2 / MF 6.2 bench.

### Quality
- `smoke_test.py` with 62 structural checks including DWB V-shape geometry.
- `examples/mx5_nd_club.svj.json` reference output from real Kunos car data.
