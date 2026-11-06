#!/usr/bin/env bash
# Barrido de un parámetro con el resto fijo (ver tools/optimizador/barrido.py).
# Antes: tools/panel.sh en marcha, play en la web y el optimizador PARADO.
#   tools/barrido.sh                         w_max = 6…13, 20 vueltas cada uno
#   tools/barrido.sh --resumen <csv>         tabla de resultados de un barrido
set -euo pipefail
cd "$(dirname "$0")/.."
exec uv run --quiet --with optuna --with aiohttp python -u tools/optimizador/barrido.py "$@"
