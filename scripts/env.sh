#!/usr/bin/env bash
# -----------------------------------------------------------------------
# AI-Grand-Prix environment setup
#
# Source this before running PX4 SITL with custom worlds/models:
#   source scripts/env.sh
#
# Adds the mav_simulator model/world trees to GZ_SIM_RESOURCE_PATH so
# that worlds with <include><uri>model://X3</uri></include> (and gates,
# warehouse, etc.) can resolve their assets.
# -----------------------------------------------------------------------

# Locate repo root from this script's path (works whether sourced or run)
if [ -n "${BASH_SOURCE[0]}" ]; then
	_AGP_SCRIPT="${BASH_SOURCE[0]}"
else
	_AGP_SCRIPT="$0"
fi
AGP_ROOT="$(cd "$(dirname "${_AGP_SCRIPT}")/.." && pwd)"
export AGP_ROOT

MAV_GZ="${AGP_ROOT}/external/mav_simulator/mav_gazebo"
MAV_DESC="${AGP_ROOT}/external/mav_simulator/mav_description"

# Order matters: list custom paths FIRST so they shadow PX4 defaults if
# names collide.
export GZ_SIM_RESOURCE_PATH="${MAV_GZ}:${MAV_GZ}/models:${MAV_GZ}/worlds:${MAV_DESC}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

# Convenience: project-local PX4 venv (matches the Makefile fix in our
# PX4 fork). Activate it implicitly if it exists.
PX4_VENV="${AGP_ROOT}/external/PX4-Autopilot/.venv-px4"
if [ -d "${PX4_VENV}" ] && [ -z "${VIRTUAL_ENV}" ]; then
	# shellcheck disable=SC1091
	source "${PX4_VENV}/bin/activate"
fi

echo "[agp] GZ_SIM_RESOURCE_PATH set; mav_simulator assets visible to Gazebo."
echo "[agp] Run e.g.:  cd external/PX4-Autopilot && make px4_sitl gz_x500_x3_illini_warehouse"
