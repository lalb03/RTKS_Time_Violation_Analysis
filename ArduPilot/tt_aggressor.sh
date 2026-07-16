#!/usr/bin/env bash

set -euo pipefail

TRIGGER="/dev/shm/timetrap_e4.trigger"

rm -f "${TRIGGER}"

echo "[TimeTrap E4] Waiting for ArduPilot trigger..."

# while the file doesn't exist, the aggressor stays not active
while [[ ! -e "${TRIGGER}" ]]; do
    sleep 0.005
done

rm -f "${TRIGGER}"

echo "[TimeTrap E4] Trigger received"
echo "[TimeTrap E4] Starting L3 aggressor: FIFO/20, 8 MiB, 6 seconds"

taskset -c 0 chrt -f 20 stress-ng \
    --cache 1 \
    --cache-level 3 \
    --timeout 6s \
    --metrics-brief

echo "[TimeTrap E4] Aggressor stopped"
