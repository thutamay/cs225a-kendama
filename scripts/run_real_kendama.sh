#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENSAI_DIR="${OPENSAI_DIR:-/home/src1/OpenSai}"
DRIVER_DIR="${DRIVER_DIR:-${OPENSAI_DIR}/drivers/FlexivRizonRedisDriver/redis_driver}"
LOG_DIR="${LOG_DIR:-${REPO_DIR}/logs/real_robot}"

DRIVER_BIN="${DRIVER_DIR}/build/flexiv_rizon4_redis_driver_robot_only"
DRIVER_CONFIG="config_folder/config_titania.xml"
OPENSAI_BIN="${OPENSAI_DIR}/bin/OpenSai_main"
OPENSAI_CONFIG="config_folder/xml_config_files/single_rizon_real.xml"
PYTHON_SCRIPT="${REPO_DIR}/kendama_throw_and_catch.py"

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

require_executable "${DRIVER_BIN}"
require_executable "${OPENSAI_BIN}"
require_file "${DRIVER_DIR}/${DRIVER_CONFIG}"
require_file "${OPENSAI_DIR}/${OPENSAI_CONFIG}"
require_file "${PYTHON_SCRIPT}"

mkdir -p "${LOG_DIR}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
DRIVER_LOG="${LOG_DIR}/driver_${TIMESTAMP}.log"
OPENSAI_LOG="${LOG_DIR}/opensai_${TIMESTAMP}.log"

echo "Refreshing sudo credentials for the Flexiv driver."
sudo -v

echo "Starting Flexiv Redis driver. Log: ${DRIVER_LOG}"
cd "${DRIVER_DIR}"
sudo env LD_LIBRARY_PATH=/home/src1/rdk_install/lib: \
    taskset --cpu-list 7 \
    chrt -rr 79 \
    ./build/flexiv_rizon4_redis_driver_robot_only \
    "${DRIVER_CONFIG}" >"${DRIVER_LOG}" 2>&1 &
DRIVER_PID=$!
cd "${REPO_DIR}"

sleep 2

if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
    echo "Flexiv Redis driver exited early. See ${DRIVER_LOG}" >&2
    exit 1
fi

echo "Starting OpenSai_main. Log: ${OPENSAI_LOG}"
cd "${OPENSAI_DIR}"
"${OPENSAI_BIN}" "${OPENSAI_CONFIG}" >"${OPENSAI_LOG}" 2>&1 &
OPENSAI_PID=$!
cd "${REPO_DIR}"

sleep 2

if ! kill -0 "${OPENSAI_PID}" 2>/dev/null; then
    echo "OpenSai_main exited early. See ${OPENSAI_LOG}" >&2
    exit 1
fi

echo "Current active controller:"
redis-cli GET "opensai::controllers::Titania::active_controller_name" || true

echo "Running kendama_throw_and_catch.py --real"
cd "${REPO_DIR}"
python3 "${PYTHON_SCRIPT}" --real
