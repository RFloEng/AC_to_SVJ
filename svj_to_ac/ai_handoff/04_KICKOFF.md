# Kickoff message

Paste this as the first user turn after the system prompt is set.

---

I'm building **SVJ_to_AC**, the inverse of the AC_to_SVJ converter at https://github.com/RFloEng/AC_to_SVJ. You have the system prompt; the full spec is in `02_PROJECT_SPEC.md` and the build plan in `03_BUILD_PLAN.md`. Read both before producing anything.

I have a local clone of AC_to_SVJ at a sibling directory — assume you can read `AC_to_SVJ/ac_parsers.py`, `AC_to_SVJ/tire_lab.py`, `AC_to_SVJ/test_car/`, and `AC_to_SVJ/examples/` as ground truth. Never invent INI fields; grep `ac_parsers.py` when in doubt.

Start with **Stage 1 — Skeleton** from `03_BUILD_PLAN.md`. Produce the file tree, then stop and wait for me to confirm before moving to Stage 2.

When you produce files: show the full content of each one in a separate fenced code block, prefixed by the relative path as a comment. After all files are shown, give me the single shell command to run for the acceptance test, and the expected output.

A few decisions I want you to default on without asking, but flag in your reply:
- Use `pyproject.toml` with `setuptools` backend (not `setup.py`).
- Use `ruff` for linting in a pre-commit config — add `.pre-commit-config.yaml`.
- `requirements.txt` is the install path; `pyproject.toml` is metadata only.
- Author name: **RFloEng**. Repo URL: `https://github.com/RFloEng/SVJ_to_AC`.

Go.
