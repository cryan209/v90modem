#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/sip_v90_modem"

SIP_SERVER=""
USERNAME=""
PASSWORD=""
PTY_LINK="/tmp/v90modem"
LOCAL_PORT="5060"
MODE="v90"
VERBOSE=1
BUILD_FIRST=1
LOG_FILE=""
RUN_PREFLIGHT=1

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Starts sip_v90_modem in softmodem-debug mode with practical defaults.

Options:
  --sip-server <host>     SIP registrar/proxy host
  --username <user>       SIP username
  --password <pass>       SIP password
  --pty-link <path>       PTY symlink path (default: ${PTY_LINK})
  --local-port <port>     Local SIP UDP port (default: ${LOCAL_PORT})
  --mode <v34|v90|v92>    Highest modem family to offer (default: ${MODE})
  --log-file <path>       Optional log output file
  --no-build              Skip 'make sip_v90_modem'
  --skip-preflight        Skip local UDP bind preflight check
  --quiet                 Do not pass --verbose to sip_v90_modem
  -h, --help              Show this help

Examples:
  $(basename "$0")
  $(basename "$0") --sip-server pbx.example.net --username 6001 --password 6001
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sip-server)
      SIP_SERVER="${2:-}"; shift 2 ;;
    --username)
      USERNAME="${2:-}"; shift 2 ;;
    --password)
      PASSWORD="${2:-}"; shift 2 ;;
    --pty-link)
      PTY_LINK="${2:-}"; shift 2 ;;
    --local-port)
      LOCAL_PORT="${2:-}"; shift 2 ;;
    --mode)
      MODE="${2:-}"; shift 2 ;;
    --log-file)
      LOG_FILE="${2:-}"; shift 2 ;;
    --no-build)
      BUILD_FIRST=0; shift ;;
    --skip-preflight)
      RUN_PREFLIGHT=0; shift ;;
    --quiet)
      VERBOSE=0; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2 ;;
  esac
done

case "${MODE}" in
  v34|v90|v92) ;;
  *) echo "Invalid --mode: ${MODE} (expected v34, v90, or v92)" >&2; exit 2 ;;
esac

check_udp_bindability() {
  local port="$1"
  local pid=""

  if ! command -v nc >/dev/null 2>&1; then
    echo "Preflight: 'nc' not found; skipping UDP bind check."
    return 0
  fi

  # First try macOS/BSD style: nc -u -l 127.0.0.1 <port>
  (nc -u -l 127.0.0.1 "${port}" >/dev/null 2>&1) &
  pid=$!
  sleep 0.2
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
    return 0
  fi
  wait "${pid}" >/dev/null 2>&1 || true

  # Fallback syntax used by some nc variants: nc -u -l <port>
  (nc -u -l "${port}" >/dev/null 2>&1) &
  pid=$!
  sleep 0.2
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
    return 0
  fi
  wait "${pid}" >/dev/null 2>&1 || true

  return 1
}

if [[ "${BUILD_FIRST}" -eq 1 ]]; then
  make -C "${ROOT_DIR}" sip_v90_modem
fi

if [[ ! -x "${BIN}" ]]; then
  echo "Binary not found or not executable: ${BIN}" >&2
  exit 1
fi

if [[ "${RUN_PREFLIGHT}" -eq 1 ]]; then
  if ! check_udp_bindability "${LOCAL_PORT}"; then
    echo "Preflight failed: cannot bind local UDP port ${LOCAL_PORT}." >&2
    echo "This usually means the port is blocked/in use, or sandbox/network policy prevents binding." >&2
    echo "Try another port with --local-port, or bypass with --skip-preflight." >&2
    exit 1
  fi
fi

CMD=("${BIN}" "--pty-link" "${PTY_LINK}" "--local-port" "${LOCAL_PORT}" "--mode" "${MODE}")

if [[ -n "${SIP_SERVER}" ]]; then
  CMD+=("--sip-server" "${SIP_SERVER}")
fi
if [[ -n "${USERNAME}" ]]; then
  CMD+=("--username" "${USERNAME}")
fi
if [[ -n "${PASSWORD}" ]]; then
  CMD+=("--password" "${PASSWORD}")
fi
if [[ "${VERBOSE}" -eq 1 ]]; then
  CMD+=("--verbose")
fi

echo "Starting softmodem debug runtime..."
echo "  PTY link:    ${PTY_LINK}"
echo "  Modem mode:  ${MODE}"
echo "  Local SIP:   UDP ${LOCAL_PORT}"
if [[ -n "${SIP_SERVER}" ]]; then
  echo "  SIP server:  ${SIP_SERVER}"
fi
if [[ -n "${USERNAME}" ]]; then
  echo "  SIP user:    ${USERNAME}"
fi
echo
echo "Open your serial terminal against: ${PTY_LINK}"
echo "  Example: minicom -D ${PTY_LINK}"
echo

if [[ -n "${LOG_FILE}" ]]; then
  mkdir -p "$(dirname "${LOG_FILE}")"
  echo "Logging to: ${LOG_FILE}"
  exec "${CMD[@]}" 2>&1 | tee "${LOG_FILE}"
else
  exec "${CMD[@]}"
fi
