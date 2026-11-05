#!/usr/bin/env bash
# Optimizador automático de parámetros (Optuna). Ver tools/optimizador/optimizar.py.
#
# Antes: tools/panel.sh en marcha y play en la web del ejercicio.
#   tools/optimizar.sh                     optimiza con tools/optimizador/config.json
#   tools/optimizar.sh otra_config.json    con otra configuración
#   tools/optimizar.sh --informe [estudio] mejores ensayos, importancia y gráficas
# Se para con Ctrl+C y se reanuda con el mismo comando.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "${1:-}" == "--informe" ]]; then
  shift
  exec uv run --quiet --with optuna --with matplotlib --with scikit-learn \
    python -u tools/optimizador/informe.py "$@"
fi
exec uv run --quiet --with optuna --with aiohttp python -u tools/optimizador/optimizar.py "$@"
