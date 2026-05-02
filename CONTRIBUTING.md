# Contributing

**Repository:** https://github.com/RFloEng/AC_to_SVJ

Thanks for taking the time to contribute. This project is small, so the
process is light.

## Getting set up

```bash
git clone <repo-url>
cd ac-to-svj
python -m venv .venv
source .venv/bin/activate       # or .venv\Scripts\activate on Windows
pip install -r requirements.txt
python smoke_test.py            # should exit 0
python converter.py             # opens the Gradio UI
```

`smoke_test.py` runs against the synthetic `test_car/` and does not need any
real AC data. If you drop a `real_car/` folder next to the repo root with
an unpacked `data/` directory, the smoke test picks up extra real-car
checks automatically.

## What to send

Bug reports — please include: the car folder (or a minimal reduction), the
converter version (shown in the log header), the first line of the log
(`✓ Source: …`), and the failing assertion from `smoke_test.py` if you have
one.

Pull requests — keep one change per PR. If your change touches the SVJ
schema output, please update `examples/mx5_nd_club.svj.json` in the same
commit and run `smoke_test.py` before pushing.

## Code style

PEP 8, 4-space indent, no tabs. Type hints on new public functions. Keep
diagnostics routed through the converter's `log` callback rather than
`print()`.

## AC car data

Never commit third-party car data — stock Kunos cars, paid mods, or ripped
content. The `.gitignore` excludes `real_car/` and `*.acd` to help with
this, but please double-check your staged files before you push.

## Attribution

If you reference an external algorithm or standard in a new file, add it to
`THIRD_PARTY_NOTICES.md` with a link or citation.
