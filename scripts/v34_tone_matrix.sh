#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DECODER="${ROOT_DIR}/vpcm_decode"
if [[ $# -ge 1 ]]; then
    TONE_DIR="$1"
elif [[ -d "${ROOT_DIR}/gough-lui-v34-modem-sounds" ]]; then
    TONE_DIR="${ROOT_DIR}/gough-lui-v34-modem-sounds"
else
    TONE_DIR="${ROOT_DIR}/gough-lui-v90-v92-modem-sounds"
fi
LAW="${LAW:-ulaw}"

if [[ ! -x "${DECODER}" ]]; then
    echo "missing decoder: ${DECODER}" >&2
    echo "build it first with: make vpcm_decode" >&2
    exit 1
fi

if [[ ! -d "${TONE_DIR}" ]]; then
    echo "missing tone directory: ${TONE_DIR}" >&2
    exit 1
fi

printf "%-42s %-3s %-8s %-8s %-8s %-9s %-9s %-9s\n" \
    "file" "ch" "exp_rates" "info0a" "info1a" "phase3" "info0_ms" "info1_ms" "phase3_ms"

for wav in "${TONE_DIR}"/*.wav; do
    [[ -e "${wav}" ]] || continue

    expected_rates="-"
    if [[ "$(basename "${wav}")" =~ -([0-9]{4,5})-([0-9]{4,5})\.[Ww][Aa][Vv]$ ]]; then
        expected_rates="${BASH_REMATCH[1]}/${BASH_REMATCH[2]}"
    fi

    channels=(L R)
    first_output="$("${DECODER}" --v34 --channel L --law "${LAW}" "${wav}" 2>/dev/null)"
    if printf '%s\n' "${first_output}" | grep -q '^Channel: Mono,'; then
        channels=(mono)
    fi

    for ch in "${channels[@]}"; do
        if [[ "${ch}" == "L" ]]; then
            output="${first_output}"
        else
            output="$("${DECODER}" --v34 --channel "${ch}" --law "${LAW}" "${wav}" 2>/dev/null)"
        fi

        info0="no"
        info1="no"
        phase3="no"
        info0_ms="-"
        info1_ms="-"
        phase3_ms="-"

        if printf '%s\n' "${output}" | grep -q "INFO0a:          decoded"; then
            info0="yes"
            info0_ms="$(printf '%s\n' "${output}" | sed -n 's/.*INFO0a:          decoded at \([0-9.]*\) ms/\1/p' | head -n 1)"
            [[ -n "${info0_ms}" ]] || info0_ms="-"
        fi

        if printf '%s\n' "${output}" | grep -q "INFO1a:          decoded"; then
            info1="yes"
            info1_ms="$(printf '%s\n' "${output}" | sed -n 's/.*INFO1a:          decoded at \([0-9.]*\) ms/\1/p' | head -n 1)"
            [[ -n "${info1_ms}" ]] || info1_ms="-"
        fi

        if printf '%s\n' "${output}" | grep -q "Phase 3:         seen"; then
            phase3="yes"
            phase3_ms="$(printf '%s\n' "${output}" | sed -n 's/.*Phase 3:         seen at \([0-9.]*\) ms/\1/p' | head -n 1)"
            [[ -n "${phase3_ms}" ]] || phase3_ms="-"
        fi

        printf "%-42s %-3s %-8s %-8s %-8s %-8s %-9s %-9s %-9s\n" \
            "$(basename "${wav}")" "${ch}" "${expected_rates}" "${info0}" "${info1}" "${phase3}" \
            "${info0_ms}" "${info1_ms}" "${phase3_ms}"
    done
done
