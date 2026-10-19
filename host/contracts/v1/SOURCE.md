Vendored copy of the canonical Robotics Device API v1 schemas (single source of truth: the assistant repo).

- repo: /c/Users/vibhe/Documents/Codex/2026-09-27/can-you-integrate-to-solidworks-to/outputs/AI_Assistant_Robot/otto_base_robo_code-main
- branch feature/robotics-platform, commit 60d601e, path robotics/shared/schemas/
- prose: robotics/docs/integration_contract.md section 6 (gimbal profile)

Do not edit here. To update: copy the files again and run `python -m pytest -q`.
tests/test_service.py::test_vendored_schemas_match_canonical fails if this copy drifts from the canonical repo (skipped when that repo is absent).
