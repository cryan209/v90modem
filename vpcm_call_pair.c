#include "vpcm_call_pair.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static void vpcm_call_pair_make_tap_name(char *dst, size_t dst_cap, const char *src)
{
    size_t i;
    size_t out = 0;

    if (!dst || dst_cap == 0)
        return;
    if (!src)
        src = "stream";
    for (i = 0; src[i] != '\0' && out + 1 < dst_cap; ++i) {
        unsigned char ch = (unsigned char) src[i];

        if (isalnum(ch))
            dst[out++] = (char) tolower(ch);
        else if (out > 0 && dst[out - 1] != '-')
            dst[out++] = '-';
    }
    while (out > 0 && dst[out - 1] == '-')
        out--;
    if (out == 0 && dst_cap > 1)
        dst[out++] = 'x';
    dst[out] = '\0';
}

static size_t vpcm_call_pair_transfer_codewords(vpcm_call_t *src_call,
                                                vpcm_call_t *dst_call,
                                                const uint8_t *tx_codewords,
                                                int codeword_len,
                                                bool robbed_bit,
                                                uint8_t *rx_codewords,
                                                size_t scratch_cap)
{
    size_t written;
    size_t moved;
    size_t read_len;
    size_t i;

    if (!src_call || !dst_call || !tx_codewords || !rx_codewords || codeword_len <= 0)
        return 0;
    (void) scratch_cap;

    written = vpcm_g711_stream_write(&src_call->tx_stream, tx_codewords, (size_t) codeword_len);
    if (written != (size_t) codeword_len)
        return 0;

    moved = vpcm_g711_stream_read(&src_call->tx_stream, rx_codewords, (size_t) codeword_len);
    if (moved != (size_t) codeword_len)
        return 0;
    if (robbed_bit) {
        for (i = 5; i < moved; i += 6)
            rx_codewords[i] &= 0xFE;
    }

    moved = vpcm_g711_stream_write(&dst_call->rx_stream, rx_codewords, moved);
    if (moved != (size_t) codeword_len)
        return 0;

    read_len = vpcm_g711_stream_read(&dst_call->rx_stream, rx_codewords, (size_t) codeword_len);
    if (read_len == (size_t) codeword_len) {
        vpcm_call_advance_tick(src_call);
        vpcm_call_advance_tick(dst_call);
    }
    return read_len;
}

static uint64_t vpcm_call_pair_count_bit_errors(const uint8_t *a, const uint8_t *b, int len)
{
    uint64_t errors = 0;
    int i;

    if (!a || !b || len <= 0)
        return 0;
    for (i = 0; i < len; ++i) {
        unsigned int diff = (unsigned int) (a[i] ^ b[i]);

        while (diff != 0) {
            errors += (uint64_t) (diff & 1U);
            diff >>= 1;
        }
    }
    return errors;
}

bool vpcm_call_pair_init(vpcm_call_pair_t *pair,
                         vpcm_call_t *caller,
                         vpcm_call_t *answerer,
                         const char *label)
{
    if (!pair || !caller || !answerer)
        return false;
    pair->caller = caller;
    pair->answerer = answerer;
    pair->label = label;
    return true;
}

bool vpcm_call_pair_attach_taps(vpcm_call_pair_t *pair, const char *dir)
{
    char base[64];
    char caller_tx_path[512];
    char caller_rx_path[512];
    char answerer_tx_path[512];
    char answerer_rx_path[512];
    char caller_wav_path[512];
    char answerer_wav_path[512];

    if (!pair || !pair->caller || !pair->answerer)
        return false;
    if (!dir || !*dir)
        return true;

    vpcm_call_pair_make_tap_name(base, sizeof(base), pair->label);
    if (snprintf(caller_tx_path, sizeof(caller_tx_path), "%s/%s-caller-tx.g711", dir, base)
            >= (int) sizeof(caller_tx_path)
        || snprintf(caller_rx_path, sizeof(caller_rx_path), "%s/%s-caller-rx.g711", dir, base)
            >= (int) sizeof(caller_rx_path)
        || snprintf(answerer_tx_path, sizeof(answerer_tx_path), "%s/%s-answerer-tx.g711", dir, base)
            >= (int) sizeof(answerer_tx_path)
        || snprintf(answerer_rx_path, sizeof(answerer_rx_path), "%s/%s-answerer-rx.g711", dir, base)
            >= (int) sizeof(answerer_rx_path)
        || snprintf(caller_wav_path, sizeof(caller_wav_path), "%s/%s-caller.wav", dir, base)
            >= (int) sizeof(caller_wav_path)
        || snprintf(answerer_wav_path, sizeof(answerer_wav_path), "%s/%s-answerer.wav", dir, base)
            >= (int) sizeof(answerer_wav_path)) {
        return false;
    }

    if (!vpcm_call_attach_stream_taps(pair->caller, caller_tx_path, caller_rx_path))
        return false;
    if (!vpcm_call_attach_wav_tap(pair->caller, caller_wav_path)) {
        vpcm_call_detach_stream_taps(pair->caller);
        return false;
    }
    if (!vpcm_call_attach_stream_taps(pair->answerer, answerer_tx_path, answerer_rx_path)) {
        vpcm_call_detach_stream_taps(pair->caller);
        return false;
    }
    if (!vpcm_call_attach_wav_tap(pair->answerer, answerer_wav_path)) {
        vpcm_call_detach_stream_taps(pair->caller);
        vpcm_call_detach_stream_taps(pair->answerer);
        return false;
    }
    return true;
}

bool vpcm_call_pair_record_g711_exchange(vpcm_call_pair_t *pair,
                                         const uint8_t *caller_tx_codewords,
                                         const uint8_t *answerer_tx_codewords,
                                         size_t codeword_len)
{
    uint8_t caller_scratch[codeword_len];
    uint8_t answerer_scratch[codeword_len];

    if (!pair || !pair->caller || !pair->answerer
        || !caller_tx_codewords || !answerer_tx_codewords || codeword_len == 0) {
        return false;
    }

    if (vpcm_g711_stream_write(&pair->caller->tx_stream, caller_tx_codewords, codeword_len) != codeword_len
        || vpcm_g711_stream_read(&pair->caller->tx_stream, caller_scratch, codeword_len) != codeword_len
        || vpcm_g711_stream_write(&pair->answerer->rx_stream, caller_tx_codewords, codeword_len) != codeword_len
        || vpcm_g711_stream_read(&pair->answerer->rx_stream, answerer_scratch, codeword_len) != codeword_len) {
        return false;
    }

    if (vpcm_g711_stream_write(&pair->answerer->tx_stream, answerer_tx_codewords, codeword_len) != codeword_len
        || vpcm_g711_stream_read(&pair->answerer->tx_stream, answerer_scratch, codeword_len) != codeword_len
        || vpcm_g711_stream_write(&pair->caller->rx_stream, answerer_tx_codewords, codeword_len) != codeword_len
        || vpcm_g711_stream_read(&pair->caller->rx_stream, caller_scratch, codeword_len) != codeword_len) {
        return false;
    }

    vpcm_call_write_stereo_wav(pair->caller, caller_tx_codewords, answerer_tx_codewords, codeword_len);
    vpcm_call_write_stereo_wav(pair->answerer, answerer_tx_codewords, caller_tx_codewords, codeword_len);
    vpcm_call_advance_tick(pair->caller);
    vpcm_call_advance_tick(pair->answerer);
    return true;
}

void vpcm_call_pair_detach_taps(vpcm_call_pair_t *pair)
{
    if (!pair)
        return;
    if (pair->caller)
        vpcm_call_detach_stream_taps(pair->caller);
    if (pair->answerer)
        vpcm_call_detach_stream_taps(pair->answerer);
}

bool vpcm_call_pair_drive_to_run(vpcm_call_pair_t *pair,
                                 vpcm_call_run_mode_t run_mode)
{
    if (!pair || !pair->caller || !pair->answerer)
        return false;

    vpcm_call_set_state(pair->caller, VPCM_CALL_WAIT_DIALTONE);
    vpcm_call_set_state(pair->answerer, VPCM_CALL_WAIT_RINGING);

    if (!vpcm_call_step_to_next_state(pair->caller) || !vpcm_call_step(pair->answerer))
        return false;
    if (!vpcm_call_step_to_next_state(pair->caller) || !vpcm_call_step(pair->answerer))
        return false;
    if (!vpcm_call_step(pair->caller) || !vpcm_call_step_to_next_state(pair->answerer))
        return false;
    if (!vpcm_call_step_to_run_mode(pair->caller, run_mode)
        || !vpcm_call_step_to_run_mode(pair->answerer, run_mode)) {
        return false;
    }
    return true;
}

bool vpcm_call_pair_drive_to_done(vpcm_call_pair_t *pair)
{
    if (!pair || !pair->caller || !pair->answerer)
        return false;

    vpcm_call_set_state(pair->caller, VPCM_CALL_HANGUP);
    vpcm_call_set_state(pair->answerer, VPCM_CALL_HANGUP);
    if (!vpcm_call_step_to_next_state(pair->caller)
        || !vpcm_call_step_to_next_state(pair->answerer)) {
        return false;
    }
    vpcm_call_pair_detach_taps(pair);
    return true;
}

void vpcm_call_pair_set_v91_state(vpcm_call_pair_t *pair,
                                  vpcm_v91_modem_state_t state)
{
    if (!pair || !pair->caller || !pair->answerer)
        return;
    vpcm_call_set_v91_state(pair->caller, state);
    vpcm_call_set_v91_state(pair->answerer, state);
}

bool vpcm_call_pair_run_v91_data(vpcm_call_pair_t *pair,
                                 vpcm_v91_session_t *session,
                                 v91_state_t *caller_tx,
                                 v91_state_t *caller_rx,
                                 v91_state_t *answerer_tx,
                                 v91_state_t *answerer_rx,
                                 const vpcm_cp_frame_t *cp_ack,
                                 uint8_t *caller_data_in,
                                 uint8_t *caller_data_out,
                                 uint8_t *answerer_data_in,
                                 uint8_t *answerer_data_out,
                                 uint8_t *caller_pcm_tx,
                                 uint8_t *caller_pcm_rx,
                                 uint8_t *answerer_pcm_tx,
                                 uint8_t *answerer_pcm_rx,
                                 int total_codewords,
                                 int codewords_per_report,
                                 bool robbed_bit,
                                 vpcm_call_pair_v91_chunk_logger_fn chunk_logger,
                                 void *chunk_logger_user_data,
                                 vpcm_call_pair_v91_data_report_t *report)
{
    vpcm_call_pair_v91_data_report_t local_report;
    int codeword_offset;

    if (!pair || !pair->caller || !pair->answerer || !session
        || !caller_tx || !caller_rx || !answerer_tx || !answerer_rx || !cp_ack
        || !caller_data_in || !caller_data_out || !answerer_data_in || !answerer_data_out
        || !caller_pcm_tx || !caller_pcm_rx || !answerer_pcm_tx || !answerer_pcm_rx) {
        return false;
    }

    memset(&local_report, 0, sizeof(local_report));
    for (codeword_offset = 0; codeword_offset < total_codewords; codeword_offset += codewords_per_report) {
        vpcm_call_pair_v91_chunk_t chunk;
        int chunk_codewords;
        int chunk_bytes;
        int byte_offset;
        int frame_index;
        int caller_produced;
        int answerer_produced;
        int caller_consumed;
        int answerer_consumed;

        memset(&chunk, 0, sizeof(chunk));
        if (!vpcm_v91_session_describe_chunk(cp_ack,
                                             total_codewords,
                                             codewords_per_report,
                                             codeword_offset,
                                             &chunk_codewords,
                                             &chunk_bytes,
                                             &byte_offset,
                                             &frame_index)) {
            return false;
        }

        if (!vpcm_v91_session_encode_duplex_chunk(session,
                                                  caller_tx,
                                                  answerer_tx,
                                                  caller_data_in,
                                                  answerer_data_in,
                                                  caller_pcm_tx,
                                                  answerer_pcm_tx,
                                                  codeword_offset,
                                                  chunk_codewords,
                                                  byte_offset,
                                                  chunk_bytes,
                                                  &caller_produced,
                                                  &answerer_produced)) {
            return false;
        }

        if (vpcm_call_pair_transfer_codewords(pair->caller,
                                              pair->answerer,
                                              caller_pcm_tx + codeword_offset,
                                              chunk_codewords,
                                              robbed_bit,
                                              answerer_pcm_rx + codeword_offset,
                                              pair->caller->tx_stream.capacity) != (size_t) chunk_codewords
            || vpcm_call_pair_transfer_codewords(pair->answerer,
                                                 pair->caller,
                                                 answerer_pcm_tx + codeword_offset,
                                                 chunk_codewords,
                                                 robbed_bit,
                                                 caller_pcm_rx + codeword_offset,
                                                 pair->answerer->tx_stream.capacity) != (size_t) chunk_codewords) {
            return false;
        }

        vpcm_call_write_stereo_wav(pair->caller,
                                   caller_pcm_tx + codeword_offset,
                                   caller_pcm_rx + codeword_offset,
                                   (size_t) chunk_codewords);
        vpcm_call_write_stereo_wav(pair->answerer,
                                   answerer_pcm_tx + codeword_offset,
                                   answerer_pcm_rx + codeword_offset,
                                   (size_t) chunk_codewords);

        if (!vpcm_v91_session_decode_duplex_chunk(session,
                                                  caller_rx,
                                                  answerer_rx,
                                                  caller_data_out,
                                                  answerer_data_out,
                                                  caller_pcm_rx,
                                                  answerer_pcm_rx,
                                                  codeword_offset,
                                                  chunk_codewords,
                                                  byte_offset,
                                                  chunk_bytes,
                                                  &caller_consumed,
                                                  &answerer_consumed)) {
            return false;
        }

        chunk.frame_index = frame_index;
        chunk.codeword_offset = codeword_offset;
        chunk.chunk_codewords = chunk_codewords;
        chunk.chunk_bytes = chunk_bytes;
        chunk.byte_offset = byte_offset;
        chunk.caller_data_in = caller_data_in + byte_offset;
        chunk.answerer_data_in = answerer_data_in + byte_offset;
        chunk.caller_data_out = caller_data_out + byte_offset;
        chunk.answerer_data_out = answerer_data_out + byte_offset;
        chunk.caller_pcm_tx = caller_pcm_tx + codeword_offset;
        chunk.caller_pcm_rx = caller_pcm_rx + codeword_offset;
        chunk.answerer_pcm_tx = answerer_pcm_tx + codeword_offset;
        chunk.answerer_pcm_rx = answerer_pcm_rx + codeword_offset;
        chunk.caller_to_answerer_ok = (memcmp(chunk.caller_data_in,
                                              chunk.answerer_data_out,
                                              (size_t) chunk_bytes) == 0);
        chunk.answerer_to_caller_ok = (memcmp(chunk.answerer_data_in,
                                              chunk.caller_data_out,
                                              (size_t) chunk_bytes) == 0);

        local_report.c2a_chunks_checked++;
        local_report.a2c_chunks_checked++;
        local_report.c2a_bits_checked += ((uint64_t) chunk_bytes * 8ULL);
        local_report.a2c_bits_checked += ((uint64_t) chunk_bytes * 8ULL);
        local_report.c2a_bit_errors += vpcm_call_pair_count_bit_errors(chunk.caller_data_in,
                                                                       chunk.answerer_data_out,
                                                                       chunk_bytes);
        local_report.a2c_bit_errors += vpcm_call_pair_count_bit_errors(chunk.answerer_data_in,
                                                                       chunk.caller_data_out,
                                                                       chunk_bytes);
        if (!chunk.caller_to_answerer_ok)
            local_report.c2a_mismatch_chunks++;
        if (!chunk.answerer_to_caller_ok)
            local_report.a2c_mismatch_chunks++;

        chunk.c2a_bits_checked = local_report.c2a_bits_checked;
        chunk.a2c_bits_checked = local_report.a2c_bits_checked;
        chunk.c2a_bit_errors = local_report.c2a_bit_errors;
        chunk.a2c_bit_errors = local_report.a2c_bit_errors;
        chunk.c2a_mismatch_chunks = local_report.c2a_mismatch_chunks;
        chunk.a2c_mismatch_chunks = local_report.a2c_mismatch_chunks;
        chunk.c2a_chunks_checked = local_report.c2a_chunks_checked;
        chunk.a2c_chunks_checked = local_report.a2c_chunks_checked;

        if (chunk_logger)
            chunk_logger(&chunk, chunk_logger_user_data);
    }

    if (report)
        *report = local_report;
    return true;
}
