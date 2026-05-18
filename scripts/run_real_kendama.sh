#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENSAI_DIR="${OPENSAI_DIR:-/home/src1/OpenSai}"
DRIVER_DIR="${DRIVER_DIR:-${OPENSAI_DIR}/drivers/FlexivRizonRedisDriver/redis_driver}"
LOG_DIR="${LOG_DIR:-${REPO_DIR}/logs/real_robot}"

DRIVER_BIN="${DRIVER_DIR}/build/flexiv_rizon4_redis_driver_robot_only"
DRIVER_CONFIG="config_titania.xml"
OPENSAI_BIN="${OPENSAI_DIR}/bin/OpenSai_main"
OPENSAI_CONFIG="config_folder/xml_config_files/single_rizon_real.xml"
OPENSAI_CONFIG_NAME="$(basename "${OPENSAI_CONFIG}")"
OPENSAI_CONFIG_KEY="::sai-interfaces-webui::config_file_name"
PYTHON_SCRIPT="${REPO_DIR}/kendama_throw_and_catch.py"
ZERO_JOINTS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
CONTROL_TORQUES_KEY="opensai::commands::Titania::control_torques"
PYTHON_ARGS=("$@")

DRIVER_PID=""
OPENSAI_PID=""

cleanup() {
    local status=$?

    if [[ -n "${OPENSAI_PID}" ]] && kill -0 "${OPENSAI_PID}" 2>/dev/null; then
        echo "Stopping OpenSai_main (${OPENSAI_PID})"
        kill "${OPENSAI_PID}" 2>/dev/null || true
    fi

    if [[ -n "${DRIVER_PID}" ]] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
        echo "Stopping Flexiv Redis driver (${DRIVER_PID})"
        sudo kill "${DRIVER_PID}" 2>/dev/null || kill "${DRIVER_PID}" 2>/dev/null || true
    fi

    exit "${status}"
}
trap cleanup EXIT INT TERM

require_executable() {
    local path="$1"
    if [[ ! -x "${path}" ]]; then
        echo "Missing executable: ${path}" >&2
        exit 1
    fi
}

require_file() {
    local path="$1"
    if [[ ! -f "${path}" ]]; then
        echo "Missing file: ${path}" >&2
        exit 1
    fi
}

wait_for_log_pattern() {
    local log_file="$1"
    local pattern="$2"
    local process_pid="$3"
    local timeout_s="$4"
    local label="$5"
    local elapsed_s=0

    while (( elapsed_s < timeout_s )); do
        if grep -q "${pattern}" "${log_file}" 2>/dev/null; then
            return 0
        fi
        if ! kill -0 "${process_pid}" 2>/dev/null; then
            echo "${label} exited before becoming ready. See ${log_file}" >&2
            exit 1
        fi
        sleep 1
        elapsed_s=$((elapsed_s + 1))
    done

    echo "Timed out waiting for ${label}. See ${log_file}" >&2
    exit 1
}

seed_joint_hold_goal() {
    local current_q
    current_q="$(redis-cli GET "opensai::sensors::Titania::joint_positions" 2>/dev/null || true)"
    if [[ -z "${current_q}" || "${current_q}" == "(nil)" ]]; then
        echo "Could not read Titania joint positions from Redis." >&2
        exit 1
    fi

    redis-cli SET "opensai::controllers::Titania::joint_controller::joint_task::goal_position" "${current_q}" >/dev/null
    redis-cli SET "opensai::controllers::Titania::joint_controller::joint_task::goal_velocity" "${ZERO_JOINTS}" >/dev/null
    redis-cli SET "opensai::controllers::Titania::joint_controller::joint_task::goal_acceleration" "${ZERO_JOINTS}" >/dev/null
    redis-cli SET "${CONTROL_TORQUES_KEY}" "${ZERO_JOINTS}" >/dev/null
}

clear_torque_command() {
    redis-cli SET "${CONTROL_TORQUES_KEY}" "${ZERO_JOINTS}" >/dev/null
}

wait_for_opensai_startup_with_joint_hold() {
    local timeout_tenths="$1"
    local elapsed_tenths=0
    local config_value=""

    while (( elapsed_tenths < timeout_tenths )); do
        seed_joint_hold_goal
        redis-cli SET "opensai::controllers::Titania::active_controller_name" "joint_controller" >/dev/null

        config_value="$(redis-cli GET "${OPENSAI_CONFIG_KEY}" 2>/dev/null || true)"
        if [[ "${config_value}" == "${OPENSAI_CONFIG_NAME}" || "${config_value}" == "\"${OPENSAI_CONFIG_NAME}\"" ]]; then
            return 0
        fi
        if ! kill -0 "${OPENSAI_PID}" 2>/dev/null; then
            echo "OpenSai_main exited before becoming ready. See ${OPENSAI_LOG}" >&2
            exit 1
        fi

        sleep 0.1
        elapsed_tenths=$((elapsed_tenths + 1))
    done

    echo "Timed out waiting for OpenSai_main to publish ${OPENSAI_CONFIG_KEY}. Last value: ${config_value:-<empty>}. See ${OPENSAI_LOG}" >&2
    exit 1
}

require_executable "${DRIVER_BIN}"
require_executable "${OPENSAI_BIN}"
require_file "${DRIVER_DIR}/config_folder/${DRIVER_CONFIG}"
require_file "${OPENSAI_DIR}/${OPENSAI_CONFIG}"
require_file "${PYTHON_SCRIPT}"

mkdir -p "${LOG_DIR}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
DRIVER_LOG="${LOG_DIR}/driver_${TIMESTAMP}.log"
OPENSAI_LOG="${LOG_DIR}/opensai_${TIMESTAMP}.log"
: >"${DRIVER_LOG}"
: >"${OPENSAI_LOG}"
ln -sfn "$(basename "${DRIVER_LOG}")" "${LOG_DIR}/latest_driver.log"
ln -sfn "$(basename "${OPENSAI_LOG}")" "${LOG_DIR}/latest_opensai.log"

echo "Refreshing sudo credentials for the Flexiv driver."
sudo -v

echo "Clearing stale Redis torque command before starting the driver."
clear_torque_command

echo "Starting Flexiv Redis driver. Log: ${DRIVER_LOG}"
cd "${DRIVER_DIR}"
sudo env LD_LIBRARY_PATH=/home/src1/rdk_install/lib: \
    taskset --cpu-list 7 \
    chrt -rr 79 \
    ./build/flexiv_rizon4_redis_driver_robot_only \
    "${DRIVER_CONFIG}" >"${DRIVER_LOG}" 2>&1 &
DRIVER_PID=$!
cd "${REPO_DIR}"

echo "Waiting for Flexiv Redis driver to enter RT_JOINT_TORQUE."
wait_for_log_pattern "${DRIVER_LOG}" "RT_JOINT_TORQUE" "${DRIVER_PID}" 30 "Flexiv Redis driver"

echo "Seeding measured joint hold goal before OpenSai starts."
seed_joint_hold_goal
redis-cli DEL "${OPENSAI_CONFIG_KEY}" >/dev/null

echo "Starting OpenSai_main. Log: ${OPENSAI_LOG}"
cd "${OPENSAI_DIR}"
"${OPENSAI_BIN}" "${OPENSAI_CONFIG}" >"${OPENSAI_LOG}" 2>&1 &
OPENSAI_PID=$!
cd "${REPO_DIR}"

echo "Waiting for OpenSai to finish startup."
wait_for_opensai_startup_with_joint_hold 100

echo "Forcing OpenSai into joint_controller with a measured hold goal."
seed_joint_hold_goal
redis-cli SET "opensai::controllers::Titania::active_controller_name" "joint_controller" >/dev/null
sleep 0.2

echo "Current active controller:"
redis-cli GET "opensai::controllers::Titania::active_controller_name" || true

if ((${#PYTHON_ARGS[@]})); then
    echo "Running kendama_throw_and_catch.py --real ${PYTHON_ARGS[*]}"
else
    echo "Running kendama_throw_and_catch.py --real"
fi
cd "${REPO_DIR}"
python3 -u "${PYTHON_SCRIPT}" --real "${PYTHON_ARGS[@]}"
