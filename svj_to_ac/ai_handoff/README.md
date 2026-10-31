# SVJ_to_AC — AI handoff pack

Everything an AI coding assistant (Claude, Cursor, Aider, GPT-4/5, Codex) needs to build the inverse of [AC_to_SVJ](https://github.com/RFloEng/AC_to_SVJ) without further context.

## Files in this folder

| File | Purpose |
| --- | --- |
| `01_SYSTEM_PROMPT.md` | Role, constraints, style. Paste into the system-prompt slot, or as the first message if there is no system slot. |
| `02_PROJECT_SPEC.md` | Full specification: inputs, outputs, coordinate inverse, unit table, per-file mapping, acceptance criteria. Self-contained — does not require web access. |
| `03_BUILD_PLAN.md` | Ten sequenced stages, each with a goal, files to produce, acceptance test, and a copy-paste prompt. Run them in order. |
| `04_KICKOFF.md` | The literal first user message to send after the system prompt. Bootstraps stage 1. |
| `05_PITFALLS.md` | Known traps and their fixes. Attach as reference; the AI should re-read before stages 4, 5, and 9. |

## Recommended workflow

1. Start a fresh chat with the target AI.
2. Paste `01_SYSTEM_PROMPT.md` as system prompt (or first message).
3. Attach or paste `02_PROJECT_SPEC.md` and `05_PITFALLS.md` as project context.
4. Paste `04_KICKOFF.md` as the first user turn.
5. After each stage completes, paste the next stage's prompt from `03_BUILD_PLAN.md`.
6. Between stages, run the acceptance test listed in the plan. Don't advance until it passes.

## Tooling fit

- **Claude / ChatGPT web UI**: paste sequentially; attach the AC_to_SVJ repo as a zip if the model supports files.
- **Cursor / Windsurf**: drop the entire pack into the project, point the AI at `02_PROJECT_SPEC.md`, work stage-by-stage.
- **Aider**: `aider --read 02_PROJECT_SPEC.md --read 05_PITFALLS.md` then feed stages.
- **Codex CLI / Claude Code**: same as Aider; commit between stages.

## What this pack does not include

- The AC_to_SVJ source code itself. Clone it locally and put it next to the new project so the AI can read `ac_parsers.py` as the ground-truth schema source.
- Real SVJ examples. Use `AC_to_SVJ/examples/ks_mazda_mx5_nd*.svj.json` as the canonical fixture.
- An FBX template. Stage 7 (optional) generates one programmatically.
