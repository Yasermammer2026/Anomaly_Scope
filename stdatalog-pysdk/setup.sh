#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 -m venv .venv
.venv/bin/python -m pip install wheels/*.whl
printf '\nSetup complete. Start with: bash start-gui.sh\n'
