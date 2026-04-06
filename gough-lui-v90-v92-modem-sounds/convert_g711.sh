#!/usr/bin/env bash
# Convert all stereo WAVs to raw G.711 A-law and μ-law, split L/R channels.
# SpanDSP output: A/ and U/
# FFmpeg output:  FF_A/ and FF_U/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOL="${SCRIPT_DIR}/../tools/wav_to_g711"

if [[ ! -x "$TOOL" ]]; then
    echo "ERROR: $TOOL not found or not executable. Run 'gcc' in tools/ first." >&2
    exit 1
fi

mkdir -p "${SCRIPT_DIR}/A" "${SCRIPT_DIR}/U" "${SCRIPT_DIR}/FF_A" "${SCRIPT_DIR}/FF_U"

for wav in "${SCRIPT_DIR}"/*.wav; do
    base="$(basename "${wav%.wav}")"
    echo "Processing: $base"

    # --- SpanDSP A-law ---
    "$TOOL" "$wav" "${SCRIPT_DIR}/A/${base}_L.g711" "${SCRIPT_DIR}/A/${base}_R.g711" a

    # --- SpanDSP μ-law ---
    "$TOOL" "$wav" "${SCRIPT_DIR}/U/${base}_L.g711" "${SCRIPT_DIR}/U/${base}_R.g711" u

    # --- FFmpeg A-law ---
    # Left channel (pan filter: FL = front-left)
    ffmpeg -y -loglevel error \
        -i "$wav" \
        -filter_complex "[0:a]pan=mono|c0=FL[out]" -map "[out]" \
        -ar 8000 -acodec pcm_alaw -f alaw \
        "${SCRIPT_DIR}/FF_A/${base}_L.g711"
    # Right channel
    ffmpeg -y -loglevel error \
        -i "$wav" \
        -filter_complex "[0:a]pan=mono|c0=FR[out]" -map "[out]" \
        -ar 8000 -acodec pcm_alaw -f alaw \
        "${SCRIPT_DIR}/FF_A/${base}_R.g711"

    # --- FFmpeg μ-law ---
    # Left channel
    ffmpeg -y -loglevel error \
        -i "$wav" \
        -filter_complex "[0:a]pan=mono|c0=FL[out]" -map "[out]" \
        -ar 8000 -acodec pcm_mulaw -f mulaw \
        "${SCRIPT_DIR}/FF_U/${base}_L.g711"
    # Right channel
    ffmpeg -y -loglevel error \
        -i "$wav" \
        -filter_complex "[0:a]pan=mono|c0=FR[out]" -map "[out]" \
        -ar 8000 -acodec pcm_mulaw -f mulaw \
        "${SCRIPT_DIR}/FF_U/${base}_R.g711"
done

echo "Done."
echo "  SpanDSP A-law : A/"
echo "  SpanDSP μ-law : U/"
echo "  FFmpeg  A-law : FF_A/"
echo "  FFmpeg  μ-law : FF_U/"
