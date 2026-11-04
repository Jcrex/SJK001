#!/usr/bin/env bash
# Cronometra una vuelta en el contenedor "robotics" y guarda el log en logs/.
# Uso: [SJK_V=4 SJK_KP=0.006 ...] tools/vuelta.sh [etiqueta] [timeout_s]
set -euo pipefail
cd "$(dirname "$0")/.."
etiqueta="${1:-prueba}"
timeout="${2:-300}"
nombre="$(date +%Y%m%d-%H%M%S)_${etiqueta}.csv"
mkdir -p logs

docker exec robotics rm -rf /tmp/sjk_sim
docker exec robotics mkdir -p /tmp/sjk_sim
docker cp src/follow_line.py robotics:/tmp/sjk_sim/follow_line.py >/dev/null
docker cp tools/sim/. robotics:/tmp/sjk_sim/ >/dev/null

env_args=()
for var in $(compgen -e | grep '^SJK_' || true); do env_args+=(-e "$var=${!var}"); done

docker exec "${env_args[@]}" robotics bash -c \
  "source /opt/ros/humble/setup.bash && python3 /tmp/sjk_sim/run_vuelta.py /tmp/sjk_sim/follow_line.py /tmp/sjk_sim/$nombre $timeout" \
  2>&1 | grep -v RTPS_TRANSPORT_SHM
docker cp "robotics:/tmp/sjk_sim/$nombre" "logs/$nombre" >/dev/null 2>&1 || true
docker cp "robotics:/tmp/sjk_sim/${nombre%.csv}_odom.csv" "logs/${nombre%.csv}_odom.csv" >/dev/null 2>&1 || true
docker exec robotics cat "/tmp/sjk_sim/$nombre.err" 2>/dev/null | grep -v RTPS_TRANSPORT_SHM | tail -5 || true
echo "log: logs/$nombre"
