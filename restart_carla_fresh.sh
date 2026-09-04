#!/usr/bin/env bash
# =============================================================================
#  restart_carla_fresh.sh  —  Kill CARLA & restart fresh in low graphics (Linux/Bash)
# =============================================================================

echo "============================================================================="
echo "  [1/2] Killing all active CARLA processes..."
echo "============================================================================="

# Kill CarlaUE4 processes
pkill -9 -f "CarlaUE4" 2>/dev/null || true

sleep 2
echo "  [OK] CARLA stopped. Ports and GPU memory released."
echo ""

echo "============================================================================="
echo "  [2/2] Launching CARLA in Low-Graphics / Low-Effort Mode..."
echo "============================================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/start_carla.py" "$@"
