#!/usr/bin/env bash
# Ejecuta las pruebas del controlador dentro del contenedor "robotics" (mismas
# versiones de OpenCV y numpy que el ejercicio) y las del optimizador en el host
# (necesitan optuna). Uso: tools/test.sh [args de pytest]
set -euo pipefail
cd "$(dirname "$0")/.."
docker exec robotics rm -rf /tmp/sjk
docker exec robotics mkdir -p /tmp/sjk
docker cp src robotics:/tmp/sjk/src >/dev/null
docker cp tests robotics:/tmp/sjk/tests >/dev/null
# El pytest 6 del contenedor no recoge nada si un fichero se salta entero: se excluye
docker exec -w /tmp/sjk robotics python3 -m pytest -q -p no:cacheprovider tests \
  --ignore=tests/test_ventana.py --ignore=tests/test_vueltas.py "$@"
uv run --quiet --with optuna --with aiohttp --with pytest \
  python -m pytest -q -p no:cacheprovider tests/test_ventana.py tests/test_vueltas.py "$@"
