#!/usr/bin/env bash
# Ejecuta las pruebas dentro del contenedor "robotics" (mismas versiones de
# OpenCV y numpy que el ejercicio). Uso: tools/test.sh [args de pytest]
set -euo pipefail
cd "$(dirname "$0")/.."
docker exec robotics rm -rf /tmp/sjk
docker exec robotics mkdir -p /tmp/sjk
docker cp src robotics:/tmp/sjk/src >/dev/null
docker cp tests robotics:/tmp/sjk/tests >/dev/null
docker exec -w /tmp/sjk robotics python3 -m pytest -q -p no:cacheprovider tests "$@"
