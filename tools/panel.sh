#!/usr/bin/env bash
# Panel de ajuste en vivo: gráficas, sliders de parámetros y grabación en logs/.
# Uso: tools/panel.sh   y abre http://localhost:8050
# Después pulsa play en la web del ejercicio con src/follow_line.py pegado.
set -euo pipefail
cd "$(dirname "$0")/.."
exec uv run --quiet --with aiohttp python -u tools/panel/panel.py "$@"
