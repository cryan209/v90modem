#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/sip_v90_modem"

SIP_SERVER=""
USERNAME=""
PASSWORD=""
PTY_LINK="/tmp/v90modem"
LOCAL_PORT="5060"
VERBOSE=1
BUILD_FIRST=1
LOG_FILE=""

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
  --log-file <path>       Optional log output file
  --no-build              Skip 'make sip_v90_modem'
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
    --log-file)
      LOG_FILE="${2:-}"; shift 2 ;;
    --no-build)
      BUILD_FIRST=0; shift ;;
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

if [[ "${BUILD_FIRST}" -eq 1 ]]; then
  make -C "${ROOT_DIR}" sip_v90_modem
fi

if [[ ! -x "${BIN}" ]]; then
  echo "Binary not found or not executable: ${BIN}" >&2
  exit 1
fi

CMD=("${BIN}" "--pty-link" "${PTY_LINK}" "--local-port" "${LOCAL_PORT}")

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
