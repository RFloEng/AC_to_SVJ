# Cars with an encrypted KN5

Some Assetto Corsa cars ship their 3-D model (`*.kn5`) protected by Content
Manager / CSP. In such a file the node tree can be read, but the textures and
several meshes are **placeholders** (1×1 images, tiny cubes), so exporting it
would silently give you a car with no paint and boxes for wheels.

AC_to_SVJ therefore **detects** these files (by the CSP encryption marker) and
**refuses to use them**. It never decrypts anything. If you have an
**unencrypted KN5 of the same car**, put it inside the car folder and the
converter will pick it automatically.

## How the car folder should look

```
my_car/                              ← the car folder you point the converter at
├─ data/   (or data.acd)             ← physics data, unchanged
├─ skins/                            ← skins stay here; they are found from the car root
│   ├─ red/
│   └─ blue/
├─ model.kn5                         ← the original. May be encrypted: it is detected
│                                       and skipped, never used while it is encrypted
├─ unencrypted/                      ← any subfolder name works
│   └─ model_unencrypted.kn5         ← unencrypted KN5 of THE SAME car (any file name)
│                                       (optional extra: model_unencrypted_LOD_B.kn5, …)
├─ extension/                        ← accessory models: ignored
└─ texture/, ui/, sfx/, animations/  ← ignored
```

Rules:

- **Any file name, any subfolder** (up to two levels deep). Folders called
  `extension`, `skins`, `texture`, `textures`, `ui`, `sfx`, `animations`, `data`
  and `vertex_masks` are never searched for the model.
- The file **must not carry the encryption marker**.
- It must be **the same car**: its node names (BODY, WHEEL_LF, SUSP_LF, …) have
  to overlap with the encrypted model's by at least 50 %. This stops an
  unrelated or accessory KN5 from being used by mistake. If several qualify, the
  best overlap wins, then a matching file name (ignoring `_decrypted`,
  `_unencrypted`, `_clean`), then the shallower and larger file.
- **Textures are read from inside the KN5** (an unencrypted KN5 embeds them). A
  separate `textures/` folder next to it is not needed and is not read.
- If the car has **no** encrypted KN5, behaviour is as before: the KN5 named
  like the folder, otherwise the largest one at the top level (subfolders are
  now searched too when the top level has none).
- LODs (`<name>_LOD_B.kn5`, `_C`, `_D`) are looked up next to the chosen KN5;
  an encrypted LOD is skipped.

## What you will see

- A conversion log line such as
  `unencrypted\model_unencrypted.kn5 (unencrypted copy; 100% node overlap with the encrypted model.kn5)`
  followed by `skipped (encrypted): …/model.kn5`.
- If only encrypted files exist, the car is **refused** (no GLB, no visual
  bindings; the physics conversion still runs) with a message explaining the
  layout above.

## Choosing the file yourself

- Command line: `python kn5_reader.py <car folder>` detects it;
  `python kn5_reader.py path/to/file.kn5` uses exactly that file (and refuses it
  if it is encrypted).
- Python: `build_svj(..., kn5_override=Path(...))`,
  `kn5_all_lods_to_glbs(..., kn5_override=Path(...))`,
  `resolve_car_kn5(car_folder)` for the decision and the explanation.

## Not covered

- `data.acd` encryption is a separate issue: the physics data must be unpacked
  as described in the converter's "needs unpack" note.
- This tool does not decrypt KN5 or `data.acd` files.
