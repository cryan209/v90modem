/*
 * v90.c — V.90 digital modem module
 *
 * ITU-T V.90 digital (server) side implementation.
 *
 * Phase 2: Wraps SpanDSP V.34 for INFO0d/INFO1d exchange.
 * Phase 3: Generates PCM codewords (Jd, J'd, Sd, S̄d, TRN1d) directly.
 * Phase 4: V.90-specific CP receive and MP/MP' transmit.
 * Data:    Modulus encoder → PCM codewords at 8 kHz.
 */

#include "v90.h"
#include "vpcm_cp.h"
#include "v92_phase4_decode.h"

#include <spandsp.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

/* V.90 downstream encoder constants (ITU-T V.90 §5) */
#define V90_MI          128     /* Default constellation points per frame interval */
#define V90_FRAME_LEN   6       /* Symbols per data frame */

/* Jd frame is 72 bits (Table 13): 17 sync + 51 data + 4 fill */
#define V90_JD_BITS         72

/* Phase 4 timing constants (ITU-T V.90 §9.4.1) */
#define V90_RI_SYMBOLS   192  /* Ri duration: at least 192T (§9.4.1.1) */
/* §9.6.1.1: on a rate renegotiation Rd is 384T and R̄d is 24T, and Rd must
 * begin on the boundary of a data frame.  Startup's Ri is only "at least
 * 192T", so the two are deliberately different constants. */
#define V90_RD_RENEG_SYMBOLS   384
#define V90_RDBAR_RENEG_SYMBOLS 24
/* §9.6.1: the digital modem shall initiate a retrain if it does not receive
 * an E sequence within 5000 ms plus two round-trip delays after transmitting
 * the Rd→R̄d transition.  At 8000 symbols/s that is 40000 symbols, plus an
 * allowance for two round trips on this bearer. */
#define V90_RENEG_E_TIMEOUT_SYMBOLS  (8000*7)
/* How long to hold Rd waiting for the analogue modem to answer at all.  Its
 * reply is S for 128T then S̄ for 16T then an optional SCR of up to 2000 ms
 * before CP, so a conformant answer is well inside two seconds; three is
 * generous and still far short of the peer's own retrain timer. */
#define V90_RENEG_ANSWER_TIMEOUT_SYMBOLS  (8000*3)
#define V90_RI_POST_CP_SYMBOLS 24
/* V.90 requires at least 2040T and MP within 2000 ms (§9.4.1.2/.3).
 * SmartLink recognizes barred Ri but enables its TRN2d study about 240 ms
 * later; 2040T leaves only ~120 useful symbols before MP.  Conversely, 8000T
 * reaches MP at the peer's short Phase-4 timeout.  The measured interop point
 * is 4000T (500 ms): SmartLink decoded MP/MP'/Ed and entered DATA.  Keep the
 * environment override for controlled receiver-conditioning A/B work. */
#define V90_TRN2D_DEFAULT_SYMBOLS 4000

static int v90_trn2d_symbols(void)
{
    static int cached;

    if (cached == 0) {
        const char *value = getenv("ME_V90_TRN2D_SYMBOLS");
        char *end;
        long parsed;

        cached = V90_TRN2D_DEFAULT_SYMBOLS;
        if (value && *value) {
            parsed = strtol(value, &end, 10);
            if (end != value && *end == '\0'
                && parsed >= 2040 && parsed <= 16000)
                cached = (int)parsed;
        }
        cached -= cached % V90_FRAME_LEN;
        if (cached < 2040)
            cached = 2040;
    }
    return cached;
}
#define V90_B1D_FRAMES    48
#define V90_B1D_SYMBOLS   (V90_B1D_FRAMES * V90_FRAME_LEN)
#define V90_MP_MAX_BITS  256
/* Native V.92 mapped SUVd/CPd queue: a profile CPd (base + modulus +
 * one 64-point constellation set) is ~1.4k bits after frame fill. */
#define V90_V92_TX_QUEUE_BITS 2048

/* V.92 Phase 4 constants (ITU-T V.92 §8.8.5 Table 31) */
/* SUVd: 17 sync + 1 start + 1 id + 13 rsv + 1 silent + 1 ack + 1 start
 *       + 16 CRC + 1 fill = 52 bits → round to next multiple of 6 = 54 */
/* Ed: 2 downstream data frames × 6 symbols/frame = 12 codewords (§8.8.2/V.92) */
#define V90_ED_SYMBOLS   12

/* ITU-T V.90 §9.3.1.2: 64 repetitions.  SmartLink detects the six-sample
 * periodicity after 150 samples, leaving enough of Sd for confirmation before
 * the required S-bar transition.  Extending Sd makes SmartLink time out while
 * waiting for S-bar after it has already accepted Sd. */
#define V90_SD_REPS     64
#define V90_SD_BAR_REPS 8

/* TRN1d: the spec requires ≥2040T (§9.3.1.4), and 2040 is already
 * exactly 340 six-symbol data frames.  Start Jd immediately afterward so
 * the receiver's reference-Ucode acquisition is not fed extra TRN1d signs. */
/* §9.3.1.4 makes TRN1d a *minimum* of 2040T, and only requires Jd to start
 * within 4000 ms of TRN1d's start -- so sending the bare minimum leaves ~3.7 s
 * of headroom unused.  §9.3.2.5 has the analogue modem condition its equaliser
 * on the *first 2040T*, so transmitting exactly 2040T gives a peer that arms
 * even slightly late a short training sequence and no way to recover.
 *
 * Live against the USR Courier (2026-07-25) that is the observed failure: it
 * detects our Sd->S-bar-d and goes silent per §9.3.2.4 (correctly), then never
 * decodes Jd, never sends S, and retrains exactly at its §9.3.2.7 deadline
 * (5.0 s after the end of Ja).  Default 2500T = 312 ms adds margin while
 * v90_jd_s_wait_symbols() below subtracts it from the same §9.3.1.5 budget, so
 * the total from the start of TRN1d is unchanged. */
#define V90_TRN1D_MIN_LEN 2040

static int v90_trn1d_len(void)
{
    static int cached;

    if (cached == 0) {
        const char *value = getenv("ME_V90_TRN1D_SYMBOLS");

        /* §8.4.5: "TRN1d shall be an integer multiple of 6 symbols long."
         * This is not cosmetic -- §8.4.4 puts the first symbol of Sd in data
         * frame interval 0 and requires the digital modem to keep data frame
         * alignment from that point on.  A TRN1d that is not a whole number of
         * 6-symbol frames shifts Jd, J'd and DIL out of frame alignment for the
         * rest of Phase 3, which is exactly the kind of thing that leaves the
         * peer unable to decode Jd at all.  2040 (=6*340) was compliant; 2500
         * is not (2500/6 = 416.67), so default to 2496 = 6*416. */
        cached = 2496;
        if (value && *value) {
            char *end;
            long parsed = strtol(value, &end, 10);

            /* Below 2040T violates §9.3.1.4.  The ceiling is §9.3.1.4's own:
             * Jd must start within 4000 ms of TRN1d starting, so TRN1d cannot
             * exceed 4000 ms = 32000T at 8 kHz.
             *
             * This was 16000 (2000 ms), on the reasoning that "much above ~2 s
             * starts eating the Jd window §9.3.2.7 entitles the peer to use".
             * A working V.90 digital modem contradicts that: the Eicon Diva
             * Server PRI transmits 30000T (3750 ms) of TRN1d -- 94% of the
             * budget -- on two independent calls that a USR Courier answered
             * with CONNECT.  Verified bit-for-bit against the §5.3 GPC
             * sequence; see docs/eicon_downstream_comparison.md.  The old
             * ceiling was what blocked testing that value.
             *
             * v90_jd_s_wait_symbols() subtracts this from the same budget and
             * stays positive here: at 32000T it still leaves 12800T of Jd wait.
             *
             * The *default* is deliberately unchanged.  30000T is unverified
             * against a live peer, and SmartLink is known to be timing
             * sensitive in this phase (see V90_SD_REPS above), so it is offered
             * as an experiment via ME_V90_TRN1D_SYMBOLS, not adopted. */
            if (end != value && *end == '\0'
                && parsed >= V90_TRN1D_MIN_LEN && parsed <= 32000)
                cached = (int) parsed;
        }
        /* Enforce the multiple-of-6 rule whatever the source of the value. */
        cached -= cached % 6;
        if (cached < V90_TRN1D_MIN_LEN)
            cached = V90_TRN1D_MIN_LEN;     /* 2040 = 6*340, already compliant */
    }
    return cached;
}

/* The decoded Ja event is the standards-driven trigger for downstream Sd.
 * Retain a deliberately late 3 s fallback only as a last-resort guard against
 * an undecodable Ja; it is later than SmartLink's normal Ja timing so it must
 * not race the real protocol transition. */
/* Interop fallback: start Sd even without a decoded Ja.  Must comfortably
 * exceed the analogue modem's whole Phase 3 upstream (S/PP/TRN ~2.2 s +
 * ~0.6 s silent gap before Ja) so it cannot pre-empt the energy-gap Ja
 * detector, which fires right when the peer starts listening for Sd. */
#define V90_WAIT_JA_FALLBACK_SAMPLES 48000

/* The explicit SmartLink Ja look-ahead starts the digital sequence before the
 * analogue modem has completed its fixed Phase 3 training study.  Suppress S
 * candidates until enough Jd has been presented for that study to finish.
 * Normal standards-driven Ja decoding has no artificial guard; an explicit
 * value can override either behaviour for other interoperability rigs. */
static int v90_min_jd_symbols(void)
{
    const char *value;
    char *end;
    long parsed;

    value = getenv("ME_V90_MIN_JD_SYMBOLS");
    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            return (int) parsed;
    }
    value = getenv("ME_V90_J_LOOKAHEAD_BITS");
    return (value && *value && strcmp(value, "0") != 0) ? 10000 : 0;
}

/* How long Jd may run without the analogue modem's S, in Jd symbols.
 *
 * §9.3.1.5 gives the digital modem 5100 ms plus a round-trip delay measured
 * from the *start of TRN1d* before it must retrain.  Jd begins exactly
 * v90_trn1d_len() symbols after that reference point, so the Jd-relative budget
 * is 5100 ms - TRN1d + rtd.
 *
 * This matters because the peer is entitled to take nearly all of it: §9.3.2.7
 * lets the analogue modem wait up to 5000 ms from the start of its post-Ja
 * silence before it begins transmitting S.  The previous defaults here (12000
 * symbols = 1.5 s to resync, 19296 = 2.4 s to auto-terminate) expired roughly
 * 3 s before a *compliant* peer's own deadline, so we tore Phase 3 down or
 * pushed on to J'd/DIL while the peer was still legitimately silent and
 * counting.  That is the "NO S RECEIVED" seen on every live d-modem call. */
static int v90_jd_s_wait_symbols(void)
{
    const char *value;
    char *end;
    long parsed;
    /* §9.3.1.5 does not bound the "round-trip delay" term, and we do not
     * measure rtd anywhere yet.  The d-modem rig reports rtd of 484-1808
     * samples; 500 ms is comfortably above that and still spec-legal, since
     * the allowance only ever makes us more patient than the minimum. */
    int rtd_allowance = 4000;

    value = getenv("ME_V90_JD_RTD_SYMBOLS");
    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            rtd_allowance = (int) parsed;
    }
    return 5100*8 - v90_trn1d_len() + rtd_allowance;
}

static int v90_jd_autoterminate_symbols(void)
{
    const char *value;
    char *end;
    long parsed;

    value = getenv("ME_V90_JD_AUTOTERMINATE_SYMBOLS");
    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            return (int) parsed;
    }
    /* Interop fallback: push on to J'd/DIL rather than retraining, for peers
     * whose S we cannot detect.  Only ever after the full §9.3.1.5 wait --
     * cutting it short is what stopped the peer from ever getting to send S. */
    value = getenv("ME_V90_J_LOOKAHEAD_BITS");
    return (value && *value && strcmp(value, "0") != 0)
           ? v90_jd_s_wait_symbols()
           : 0;
}

/* Jd' is exactly 12 symbols in V.90 §8.4.3.  Keep that interoperable
 * default, while allowing a longer all-zero diagnostic window to distinguish
 * a receiver decision/alignment miss from a wrong Jd' bit stream on live
 * hardware. */
static int v90_jd_prime_symbols(void)
{
    const char *value = getenv("ME_V90_JD_PRIME_SYMBOLS");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 12 && parsed <= 720)
            return (int) parsed;
    }
    return 12;
}

/* Per V.90 §9.3.1.6 and §8.4.1 the digital modem should repeat the entire DIL
 * sequence until the analogue modem signals completion with an S/S-bar
 * transition -- it must NOT decide on its own when the peer has "enough" DIL,
 * and real modems can need more than one descriptor cycle to converge their
 * impairment/constellation design.
 *
 * Spec-correct behaviour is thus value 0 = repeat-until-S.  That is NOT yet the
 * default because two live findings (2026-07-24, USR Courier) must be resolved
 * first:
 *   1. The upstream S/S-bar detector false-fires on the analogue modem's silent
 *      DIL turnaround (a carrier-offset rotation of near-silent noise reads as a
 *      sustained S; the AGC normalises equalizer magnitude, so only *raw* input
 *      power discriminates -- now logged next to each far-end-S detection so the
 *      floor can be calibrated).  Repeat-until-S without that gate just accepts
 *      the first false S past the minimum and still ends at ~1 cycle; a naive
 *      power gate instead suppressed the *real* pre-DIL S and stalled Phase 3.
 *   2. The analogue modem retrains ~15 s + round-trip after INFO1a (§9.4.2), so
 *      the whole DIL/Ri/CPt/TRN2d/MP/B1 sequence shares one deadline -- extra
 *      DIL cycles trade against that budget and can themselves trip the timeout.
 * Until (1) is calibrated, default to the one-cycle interop cap.  Non-zero N
 * auto-terminates after N complete cycles; SmartLink ADI/ADI-QC want exactly 1.
 * The minimum-cycle floor for *accepting* an S is separate (v90_handle_rx_event). */
static int v90_dil_autoterminate_cycles(void)
{
    const char *value;
    char *end;
    long parsed;

    value = getenv("ME_V90_DIL_AUTOTERMINATE_CYCLES");
    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            return (int) parsed;
    }
    value = getenv("ME_V90_DIL_PROFILE");
    if (value && (strcmp(value, "smartlink-adi-qc") == 0
                  || strcmp(value, "smartlink-adi") == 0))
        return 1;
    return 1;
}

/* Interop-only allowance for a peer whose upstream S termination request is
 * not yet recovered reliably.  DIL must still end at a segment boundary; this
 * only permits the bounded auto-termination target to fall slightly before a
 * complete synthetic fallback cycle.  The standards-driven default is zero. */
static int v90_dil_autoterminate_early_symbols(void)
{
    const char *value = getenv("ME_V90_DIL_EARLY_SYMBOLS");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= 8000)
            return (int) parsed;
    }
    return 0;
}

/* Interop-only per-DS0-phase amplitude shaping for TRN1d.
 *
 * §8.4.5 makes TRN1d a *constant* U_INFO codeword, so every DS0 phase carries
 * identical statistics and, once the peer's equaliser converges, its per-phase
 * variance measurement is byte-identical across all six phases.  The
 * SmartLink/d-modem V90AutoDigitalImpDetector normalises trn1Sigma by the
 * spread of that per-phase array; a zero spread makes it saturate to +/-2^30..
 * 2^31, which the detector reads as an impairment and answers with
 * "drop to V34 requested".  A real CO line never presents a perfectly uniform
 * DS0 stream because robbed-bit signalling perturbs specific frames of the
 * six-frame superframe -- which is exactly what this detector exists to
 * measure.
 *
 * The mechanism is a per-phase *variance* difference, not a per-phase level
 * offset: a static level offset is variance-invariant and, being a smooth
 * per-phase gain, is nulled by the peer's equaliser before the detector sees
 * it (confirmed live 2026-07-22 -- offset amp=3 left the peer's per-phase
 * array perfectly uniform).  Instead we dither the codeword magnitude by a
 * per-phase amount, which the linear equaliser cannot predict away, so it
 * survives as residual per-phase variance.  The dither is symmetric (zero
 * mean) so the average codeword stays at U_INFO -- the peer already has U_INFO
 * from INFO1a; TRN1d only trains its equaliser.
 *
 * This returns the peak dither radius in Ucodes (0 disables, spec-pure
 * default).  Live-tunable via ME_V90_TRN1D_SHAPE so the amplitude can be swept
 * against the peer's RBS variance threshold without a rebuild.
 *
 * LIVE RESULT 2026-07-22 -- kept as a diagnostic, but it does NOT move this
 * peer.  With dither amp=9 the TX tap carried a clear 8.3% per-phase magnitude
 * spread on the wire, yet the SmartLink/d-modem detector still reported its
 * per-phase "second update" array as perfectly uniform (3775 x6) and trn1Sigma
 * still saturated to +/-2^29..2^31 -> "drop to V34".  Its pre-equaliser
 * "initail var" spread was likewise unchanged from the unshaped baseline
 * (~1.9x, i.e. channel noise, not our signal).  So the peer either normalises
 * per-phase level in its equaliser before measuring, or the trn1Sigma
 * computation is input-independent -- either way the saturation is a peer-side
 * defect that downstream TRN1d shaping cannot reach.  The real next step is to
 * disassemble the blob's V90AutoDigitalImpDetector; see the interop-rig notes. */
static int v90_trn1d_shape_amplitude(void)
{
    const char *value = getenv("ME_V90_TRN1D_SHAPE");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= 20)
            return (int) parsed;
    }
    return 0;
}

/* Symmetric per-phase magnitude dither for TRN1d shaping.  Each DS0 phase gets
 * a distinct dither radius (scaled to the requested peak) so the peer's
 * per-phase variance array is non-uniform and its trn1Sigma stays finite.
 * Phase is the DS0 index mod 6; TRN1d begins on a superframe boundary (Sd and
 * S-bar-d are whole multiples of six symbols), so this aligns with the peer's
 * superframe phase, though any fixed six-periodic radius pattern breaks the
 * degenerate uniformity regardless of alignment. */
static int v90_trn1d_phase_dither(int phase, int amplitude)
{
    /* Distinct per-phase radii (three levels) at peak 3; scaled by amplitude. */
    static const int radius[6] = { 3, 2, 1, 2, 3, 1 };
    static uint32_t lcg = 0x1234567u;
    int r, span, d;

    if (amplitude <= 0)
        return 0;
    r = radius[((phase % 6) + 6) % 6] * amplitude / 3;
    if (r <= 0)
        return 0;
    span = 2 * r + 1;
    lcg = lcg * 1103515245u + 12345u;
    d = (int) ((lcg >> 16) % (uint32_t) span) - r;   /* uniform in [-r, +r] */
    return d;
}

/* Symbols of Jd to transmit without seeing the peer's S before requesting a
 * Phase-2 restart from the live modem engine.  Set to 0 to disable recovery.
 *
 * Enabling this stops us pouring stale Jd/Phase-4 audio over the peer's
 * retrain — verified live to change the SmartLink client's bulk-delay
 * estimate from wild swings (2440 then 13112 samples) to a stable ~0.
 * The engine detects the JD -> WAIT_JA transition and restarts its V.34
 * control-channel state, so the default protects the SmartLink retrain path.
 *
 * The window is the §9.3.1.5 S deadline, not a shorter guess.  It was 12000
 * symbols (1.5 s), which cannot distinguish "the peer retrained" from "the
 * peer is silent because §9.3.2.4 told it to be" -- and the latter is the
 * normal case for the whole Jd wait.  A peer that really did retrain now costs
 * us the full deadline before we resync; the durable fix is to notice energy
 * returning as something other than S, since §9.3.2.4/§9.3.2.9 make silence
 * here expected rather than diagnostic. */
/* How long to hold silence after a Jd no-S resync before re-emitting Sd.
 * Measured against slmodemd: after its own retrain the peer sits in
 * WaitForSd and gives up after about 1.9 to 3.6 s, while our generic WAIT_JA
 * fallback is 6 s -- so we were still silent when it quit, every time, and
 * its log showed "Error Energy = -0.000" throughout.  A short silence still
 * protects a peer that is genuinely re-ranging; this one just has to land
 * inside its window. */
static int v90_wait_ja_resync_samples(void)
{
    const char *value = getenv("ME_V90_WAIT_JA_RESYNC_SAMPLES");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            return (int) parsed;
    }
    return 9600;   /* 1.2 s at 8 kHz */
}

static int v90_jd_resync_symbols(void)
{
    const char *value = getenv("ME_V90_JD_RESYNC_SYMBOLS");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= INT_MAX)
            return (int) parsed;
    }
    return v90_jd_s_wait_symbols();
}

/* Milliseconds of silence to hold after detecting Ja before starting to
 * transmit Sd (§9.3.1.3 permits up to 500ms here). Default 0 preserves the
 * original immediate-Sd behaviour. See ME_V90_SD_DELAY_MS callers for the
 * value measured against the d-modem/slmodemd rig. */
static int v90_sd_delay_samples(const v90_state_t *s);

/* Ucode-to-PCM codeword mapping (ITU-T V.90 Table 1/V.90) */
/* A-law positive codewords indexed by Ucode */
static const uint8_t v90_ucode_to_alaw[128] = {
    /* Ucode   0-  7 */ 0xD5, 0xD4, 0xD7, 0xD6, 0xD1, 0xD0, 0xD3, 0xD2,
    /* Ucode   8- 15 */ 0xDD, 0xDC, 0xDF, 0xDE, 0xD9, 0xD8, 0xDB, 0xDA,
    /* Ucode  16- 23 */ 0xC5, 0xC4, 0xC7, 0xC6, 0xC1, 0xC0, 0xC3, 0xC2,
    /* Ucode  24- 31 */ 0xCD, 0xCC, 0xCF, 0xCE, 0xC9, 0xC8, 0xCB, 0xCA,
    /* Ucode  32- 39 */ 0xF5, 0xF4, 0xF7, 0xF6, 0xF1, 0xF0, 0xF3, 0xF2,
    /* Ucode  40- 47 */ 0xFD, 0xFC, 0xFF, 0xFE, 0xF9, 0xF8, 0xFB, 0xFA,
    /* Ucode  48- 55 */ 0xE5, 0xE4, 0xE7, 0xE6, 0xE1, 0xE0, 0xE3, 0xE2,
    /* Ucode  56- 63 */ 0xED, 0xEC, 0xEF, 0xEE, 0xE9, 0xE8, 0xEB, 0xEA,
    /* Ucode  64- 71 */ 0x95, 0x94, 0x97, 0x96, 0x91, 0x90, 0x93, 0x92,
    /* Ucode  72- 79 */ 0x9D, 0x9C, 0x9F, 0x9E, 0x99, 0x98, 0x9B, 0x9A,
    /* Ucode  80- 87 */ 0x85, 0x84, 0x87, 0x86, 0x81, 0x80, 0x83, 0x82,
    /* Ucode  88- 95 */ 0x8D, 0x8C, 0x8F, 0x8E, 0x89, 0x88, 0x8B, 0x8A,
    /* Ucode  96-103 */ 0xB5, 0xB4, 0xB7, 0xB6, 0xB1, 0xB0, 0xB3, 0xB2,
    /* Ucode 104-111 */ 0xBD, 0xBC, 0xBF, 0xBE, 0xB9, 0xB8, 0xBB, 0xBA,
    /* Ucode 112-119 */ 0xA5, 0xA4, 0xA7, 0xA6, 0xA1, 0xA0, 0xA3, 0xA2,
    /* Ucode 120-127 */ 0xAD, 0xAC, 0xAF, 0xAE, 0xA9, 0xA8, 0xAB, 0xAA,
};

/* V.90 scrambler (V.34 polynomial GPC, 1 + x^-18 + x^-23) */
typedef struct {
    uint32_t sr;
} v90_scrambler_t;

typedef struct {
    int prev_odd;
    uint8_t prev_t[V90_FRAME_LEN];
    int trellis_state;
    double x1;
    double y1;
    double v1;
    int pending_count;
    int pending_ucodes[4][V90_FRAME_LEN];
    uint8_t pending_signs[4][V90_FRAME_LEN];
} v90_shaper_state_t;

static int v90_shaper_delay_frames(const vpcm_cp_frame_t *cp)
{
    if (!cp || cp->shaping_redundancy == 0)
        return 0;
    return (cp->shaping_lookahead + cp->shaping_redundancy - 1)
         / cp->shaping_redundancy;
}

static void v90_scrambler_init(v90_scrambler_t *sc)
{
    sc->sr = 0;  /* V.90 §8.4: scrambler initialized to zero */
}

static int v90_scramble_bit(v90_scrambler_t *sc, int in_bit)
{
    /* V.90 5.3: the digital modem uses V.34 GPC,
     * 1 + x^-18 + x^-23 (not the answer-modem GPA tap at x^-5). */
    int fb = ((sc->sr >> 22) ^ (sc->sr >> 17)) & 1;
    int out_bit = in_bit ^ fb;
    sc->sr = ((sc->sr << 1) | out_bit) & 0x7FFFFF;
    return out_bit;
}

static uint8_t v90_scramble_byte(v90_scrambler_t *sc, uint8_t in)
{
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        int in_bit = (in >> i) & 1;
        int out_bit = v90_scramble_bit(sc, in_bit);
        out |= (uint8_t)(out_bit << i);
    }
    return out;
}

static int v90_descramble_reg_bit(uint32_t *reg, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (int)(*reg >> 22)
                      ^ (int)(*reg >> 17)) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

struct v90_state_s {
    v34_state_t     *v34;
    v90_law_t        law;

    /* Per-attempt override for the pre-Sd delay; < 0 uses the
     * ME_V90_SD_DELAY_MS env default.  See v90_set_sd_delay_ms(). */
    int              sd_delay_override_samples;

    /* Phase 3/4 TX state */
    v90_tx_phase_t   tx_phase;
    int              u_info;        /* U_INFO Ucode from analog modem's INFO1a */
    v90_scrambler_t  scrambler;
    int              diff_enc;      /* Differential encoder state (last sign bit) */
    int              sample_count;  /* Sample counter within current sub-state */
    int              rep_count;     /* Repetition counter (for Jd, Sd, etc.) */
    bool             phase4_hold_logged;
    bool             jd_rate_cap_logged;
    int              phase4_ri_align_remaining;
    bool             jd_terminate_requested;
    bool             jp_terminate_requested;
    bool             training_complete;
    bool             dil_requested;
    bool             jd_terminated_by_s;
    bool             jd_terminated_by_su;
    /* Set when WAIT_JA was entered from the Jd no-S resync rather than at the
     * start of Phase 3, so the fallback back to Sd can use a deadline that
     * fits inside the peer's WaitForSd patience. */
    bool             jd_resync_wait;
    bool             dil_terminate_requested;
    bool             v92_phase3;
    bool             v92_su_seen;
    bool             v92_su_bar_seen;

    /* Jd frame data */
    uint8_t          jd_bits[16];   /* Jd frame packed into bytes (72 bits) */
    int              jd_bit_pos;    /* Current bit position in Jd frame */
    uint8_t          jp_bits[16];   /* V.92 Jp frame packed into bytes (72 bits) */
    int              jp_bit_pos;    /* Current bit position in Jp frame */

    /* DIL descriptor/state */
    v90_dil_desc_t   dil;
    int              dil_segment_index;
    int              dil_pos_in_segment;

    /* Phase 4 CP state */
    bool             cp_ready;                  /* TRN2d→CP/SUVd transition armed */

    /* V.90 §9.6 rate renegotiation, digital-modem side.
     *
     * §9.6 says the procedure "can be initiated at any time during data mode"
     * and that the data signalling rate may change as a result, so it is both
     * the speed shift and -- because it ends in a fresh B1 from the analogue
     * modem, which is what our upstream receiver acquires against -- the
     * re-acquisition after a loss of carrier.  A search over the state the
     * receiver is already in has been measured twice not to reach the state
     * on the far side of one of this peer's one-sample timing slips
     * (docs/v90_upstream_data_path.md). */
    bool             reneg_pending;             /* asked for, waiting for a frame boundary */
    bool             reneg_active;              /* Rd sent, sequence running */
    bool             reneg_timed_out;           /* §9.6.1: no Ed in time, retrain owed */
    int64_t          reneg_rbar_symbol;         /* symbol count at the Rd→R̄d transition */
    int64_t          reneg_symbol_clock;        /* symbols emitted since renegotiation began */
    int              reneg_count;
    vpcm_cp_frame_t  cp_frame;                  /* CP frame to transmit */
    uint8_t          cp_bits[VPCM_CP_MAX_BITS]; /* Encoded CP bits (one per byte) */
    int              cp_nbits;                  /* Total encoded CP bits */
    int              cp_bit_pos;                /* Current bit index in cp_bits */
    bool             cp_ack_received;
    bool             e_received;
    bool             b1_received;
    int              upstream_rate_limit_bps;   /* actual V.34 upstream rate; 0 = no cap */

    /* V.90 Phase 4 Type-0 MP and CPt-selected modulus/shaping mapper. */
    uint8_t          mp_bits[V90_MP_MAX_BITS];
    int              mp_nbits;
    int              mp_bit_pos;
    bool             mp_acknowledge;
    bool             phase4_mapper_ready;
    int              phase4_k;
    int              phase4_d;
    int              phase4_s;
    int              phase4_sr;
    v90_scrambler_t  phase4_scrambler;
    int              phase4_prev_sign;
    v90_shaper_state_t phase4_shaper;
    uint8_t          phase4_frame[V90_FRAME_LEN];
    int              phase4_frame_pos;

    /* V.90 data-mode CP and negotiated modulus/shaping mapper. */
    vpcm_cp_frame_t  data_cp_frame;
    bool             data_cp_received;
    bool             data_mapper_ready;
    int              data_mapper_k;
    int              data_mapper_d;
    int              data_mapper_s;
    int              data_mapper_sr;
    v90_scrambler_t  data_mapper_scrambler;
    int              data_mapper_prev_sign;
    v90_shaper_state_t data_shaper;
    uint8_t          data_mapper_frame[V90_FRAME_LEN];
    int              data_mapper_frame_pos;
    uint64_t         data_input_bits;
    int              data_input_bit_count;

    /* V.92 Phase 4 state */
    bool             v92_mode;                  /* V.92 Phase 4 enabled */
    uint8_t          suv_bits[V92_SUVD_BITS];   /* Encoded SUVd bit stream (one bit per byte) */
    int              suv_bit_pos;               /* Current bit index in suv_bits */

    /* V.92 native upstream Phase 4 gating (§9.6.1.1), driven by real
     * SUVu/CPu/CPu' receive events instead of the compatibility sequence. */
    bool             v92_native_cpu_rx;
    bool             v92_suvu_received;
    bool             v92_cpu_received;
    bool             v92_remote_ack_received;   /* CPu'/SUVu' ack, or E2u */
    bool             v92_cpd_sent;
    bool             v92_ack_sent;              /* sent >= 1 SUVd' (ack=1) */

    /* V.92 native mapped Phase 4 TX: SUVd/CPd bit queue transmitted through
     * the CPt-negotiated TRN2d mapper, and the CPd upstream profile. */
    uint8_t          v92_tx_bits[V90_V92_TX_QUEUE_BITS];
    int              v92_tx_nbits;
    int              v92_tx_pos;
    uint8_t          v92_upstream_drn;          /* Table 30 drn, 0..19 */
    uint8_t          v92_trellis_select;
    uint16_t         v92_gain_q0_16;

    /* Downstream PCM encoder state (data mode) */
    v90_scrambler_t  data_scrambler;
    int              prev_sign;     /* §5.4.5.1 differential sign coding */
    uint32_t         rx_scramble_reg;
    int              rx_prev_sign;

    bool             owns_v34;      /* true if we allocated v34 (v90_init), false if external */
};

static int v90_sd_delay_samples(const v90_state_t *s)
{
    const char *value;
    char *end;
    long parsed;

    if (s && s->sd_delay_override_samples >= 0)
        return s->sd_delay_override_samples;
    value = getenv("ME_V90_SD_DELAY_MS");
    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= 500)
            return (int) (parsed * 8);   /* ms -> samples at 8000 Hz */
    }
    return 0;
}

/* Override the pre-Sd delay for this training attempt.  A negative ms value
 * restores the ME_V90_SD_DELAY_MS env default.  Needed because the delay that
 * suits an initial attempt does not suit a §9.5-retrained one: with the whole
 * Phase 2 chain pre-converged, our Ja detection outruns the peer's
 * WaitForSd arming, and an Sd transmitted before that arming leaves its
 * Phase 3 equalizer training on unanchored TRN1d (error energy pinned at
 * ~3000 instead of converging; observed live 2026-07-22, call 10 attempt 2). */
void v90_set_sd_delay_ms(v90_state_t *s, int ms)
{
    if (!s)
        return;
    if (ms < 0) {
        s->sd_delay_override_samples = -1;
        return;
    }
    /* V.90 §9.3.1.3 permits no more than 500 ms. */
    if (ms > 500)
        ms = 500;
    s->sd_delay_override_samples = ms * 8;
}

static void v90_reset_data_pump_state(v90_state_t *s)
{
    if (!s)
        return;
    /* Preserve the legacy live mapper's all-ones data scrambler seed while
     * keeping the Phase 3/4 scrambler independent. */
    s->data_scrambler.sr = 0x7FFFFF;
    s->prev_sign = 0;
    s->rx_scramble_reg = 0x7FFFFF;
    s->rx_prev_sign = 0;
}

/* ---- PCM codeword helpers ---- */

static inline uint8_t ucode_to_pcm_positive(v90_law_t law, int ucode)
{
    if (law == V90_LAW_ALAW)
        return v90_ucode_to_alaw[ucode & 0x7F];
    return (uint8_t)(0xFF - ucode);  /* µ-law */
}

static inline int16_t v90_pcm_to_linear(v90_law_t law, uint8_t codeword)
{
    if (law == V90_LAW_ALAW)
        return alaw_to_linear(codeword);
    return ulaw_to_linear(codeword);
}

static inline uint8_t v90_pcm_idle(v90_law_t law)
{
    return (law == V90_LAW_ALAW) ? (uint8_t)0xD5 : (uint8_t)0xFF;
}

/* Generate a signed G.711 codeword from a Ucode and sign bit.
 * sign=1 → positive, sign=0 → negative. */
static inline uint8_t v90_pcm_signed_codeword(v90_law_t law, int ucode, int sign)
{
    uint8_t pcm = ucode_to_pcm_positive(law, ucode);
    pcm = (uint8_t) ((pcm & 0x7F) | (sign ? 0x80 : 0x00));  /* bit7 = polarity */
    return pcm;
}

/*
 * The primary Table-14 masks describe the PCM codes at the digital modem's
 * transmitter.  They therefore have to be emitted directly in the local
 * bearer law.  Bit 128 optionally supplies a second, corresponding set of
 * levels at the far codec's D/A output; those levels are for the analogue
 * modem's receiver design and our spectral-shaping metric, not codewords to
 * substitute on the digital bearer.
 *
 * This distinction matters on an interworked path.  A peer can, for example,
 * request transmitter Ucode 81 on a mu-law bearer and report that it becomes
 * A-law Ucode 68 at the codec output.  Translating Ucode 81 as though it were
 * already an A-law output level sends a third, unintended PCM code.
 */
static inline uint8_t v90_cp_transport_codeword(v90_state_t *s,
                                                const vpcm_cp_frame_t *cp,
                                                int ucode,
                                                int sign)
{
    (void)cp;
    return v90_pcm_signed_codeword(s->law, ucode, sign);
}

/* V.90 §8.6.4: R is the six-symbol sign pattern +++--- repeated at the
 * selected magnitude.  The barred R that terminates it is four repetitions
 * of ---+++.  Ri uses U_INFO for every data-frame interval and neither
 * sequence is scrambled or differentially encoded. */
/* §9.4.1.1 gives startup's Ri "at least 192T"; §9.6.1.1 gives a rate
   renegotiation's Rd exactly 384T.  Same signal, different duration, so the
   length is a function of which procedure is running. */
static int v90_ri_length(const v90_state_t *s)
{
    return s->reneg_active ? V90_RD_RENEG_SYMBOLS : V90_RI_SYMBOLS;
}

static inline uint8_t v90_ri_codeword(v90_state_t *s,
                                      int symbol_pos,
                                      bool barred)
{
    int positive = (symbol_pos % V90_FRAME_LEN) < 3;

    if (barred)
        positive = !positive;
    return v90_pcm_signed_codeword(s->law, s->u_info, positive);
}

uint8_t v90_codeword_compose(v90_law_t law, int ucode, int sign)
{
    return v90_pcm_signed_codeword(law, ucode & 0x7F, sign ? 1 : 0);
}

void v90_codeword_decompose(v90_law_t law, uint8_t codeword, int *ucode_out, int *sign_out)
{
    if (sign_out)
        *sign_out = (codeword & 0x80) ? 1 : 0;
    if (ucode_out) {
        if (law == V90_LAW_ALAW)
            *ucode_out = (codeword ^ 0x55) & 0x7F;
        else
            *ucode_out = 0x7F - (codeword & 0x7F);
    }
}

static void v90_bits_put(uint8_t *buf, int *bit_pos, uint32_t value, int bits)
{
    int i;

    for (i = 0; i < bits; i++) {
        int pos = *bit_pos + i;
        if (value & (1U << i))
            buf[pos >> 3] |= (uint8_t) (1U << (pos & 7));
    }
    *bit_pos += bits;
}

static int v90_bits_get(const uint8_t *buf, int bit_pos, int bits)
{
    int i;
    int value;

    value = 0;
    for (i = 0; i < bits; i++) {
        if (buf[(bit_pos + i) >> 3] & (1U << ((bit_pos + i) & 7)))
            value |= 1U << i;
    }
    return value;
}

static uint16_t v90_crc_bit_block(const uint8_t buf[], int first_bit, int last_bit, uint16_t crc)
{
    int pre;
    int post;

    last_bit++;
    pre = first_bit & 0x7;
    first_bit >>= 3;
    if (pre) {
        crc = crc_itu16_bits(buf[first_bit] >> pre, (8 - pre), crc);
        first_bit++;
    }
    post = last_bit & 0x7;
    last_bit >>= 3;
    if ((last_bit - first_bit) != 0)
        crc = crc_itu16_calc(buf + first_bit, last_bit - first_bit, crc);
    if (post)
        crc = crc_itu16_bits(buf[last_bit], post, crc);
    return crc;
}

/*
 * Build a SUVd bit stream (Table 31/V.92) into a flat byte array (one bit per byte).
 * 54 bits total: 17 sync ones + frame fields + 16-bit CRC + fill to multiple of 6.
 * The ack bit (bit 33) is set when the digital modem has received CPu.
 */
static void v90_build_suvd(uint8_t bits[V92_SUVD_BITS], bool ack)
{
    v92_suvd_frame_t frame = {
        .silent_period_requested = false,
        .acknowledge = ack
    };

    if (!v92_suvd_encode(&frame, bits, V92_SUVD_BITS))
        memset(bits, 0, V92_SUVD_BITS);
}

/*
 * Queue an SUVd sequence for native V.92 TRN2d-mapped transmission.  The
 * Table 31 fill rule extends the frame to the next multiple of 6 symbols,
 * which is one d-bit data frame of the CPt-negotiated mapper.
 */
static bool v90_build_v92_suvd_mapped(v90_state_t *s, bool ack)
{
    v92_suvd_frame_t frame = {
        .silent_period_requested = false,
        .acknowledge = ack
    };

    if (!s || s->phase4_d < 1)
        return false;
    if (!v92_suvd_encode_aligned(&frame, s->phase4_d,
                                 s->v92_tx_bits,
                                 (int)sizeof(s->v92_tx_bits),
                                 &s->v92_tx_nbits))
        return false;
    s->v92_tx_pos = 0;
    if (ack)
        s->v92_ack_sent = true;
    return true;
}

/*
 * Queue a native Table 30 CPd for TRN2d-mapped transmission.  The base part
 * carries the selected upstream rate, trellis, and prefilter gain from the
 * V.92 CPd profile; the modulus parameters and a single robbed-bit-safe
 * constellation set (odd Ucodes, smallest magnitude first) describe the
 * upstream data-mode modulation.  Precoder/prefilter coefficients stay
 * absent until real upstream channel measurements exist.  The acknowledge
 * bit reflects whether a valid CPu has been received.
 */
bool v90_build_v92_cpd_frame(const v90_state_t *s, v92_cpd_frame_t *out)
{
    int points = 0;

    if (!s || !out)
        return false;
    memset(out, 0, sizeof(*out));
    out->modulus_present = true;
    out->constellations_present = true;
    out->selected_upstream_drn = s->v92_upstream_drn;
    out->trellis_select = s->v92_trellis_select;
    out->extend_e2u = false;
    out->acknowledge = s->v92_cpu_received;
    out->gain_q0_16 = s->v92_gain_q0_16;
    for (int ucode = 1; ucode < 128 && points < V92_CPD_MAX_POINTS;
         ucode += 2) {
        int16_t linear = v90_pcm_to_linear(
            s->law, ucode_to_pcm_positive(s->law, ucode));

        if (linear <= 0)
            continue;
        out->points[0][points++] = (uint16_t)linear;
    }
    out->set_sizes[0] = (uint8_t)points;
    /* V.92 §6.4.2: k=3 uses equivalence classes modulo 2*Mi across a
     * constellation of N=2*LC signed points.  Therefore Mi must not exceed
     * LC.  The old 2*points value made Mi=N: half of the k=3 residues had no
     * representative in the constellation, so no analogue transmitter could
     * honour the CPd. */
    for (int i = 0; i < 12; i++)
        out->moduli[i] = (uint8_t)points;
    if (points > 0) {
        __uint128_t product = 1;
        int drn = out->selected_upstream_drn;

        for (int i = 0; i < 12; i++)
            product *= (unsigned)points;
        /* §6.4.1 requires product(Mi) >= 2^K, with Table 30/§6.1 giving
         * K=2*(drn+17).  Back off the offer rather than emitting a CPd whose
         * modulus alphabet cannot carry its selected rate. */
        while (drn > 0
               && product < ((__uint128_t)1 << (2*(drn + 17))))
            drn--;
        if (drn < 1)
            return false;
        out->selected_upstream_drn = (uint8_t)drn;
    }
    return points > 0;
}

static bool v90_build_v92_cpd_native(v90_state_t *s)
{
    v92_cpd_frame_t cpd;

    if (!s || s->phase4_d < 1 || !v90_build_v92_cpd_frame(s, &cpd))
        return false;
    if (!v92_cpd_encode(&cpd, s->phase4_d,
                        s->v92_tx_bits,
                        (int)sizeof(s->v92_tx_bits),
                        &s->v92_tx_nbits))
        return false;
    s->v92_tx_pos = 0;
    return true;
}

static bool v90_info_fill_and_sync_ok(const uint8_t *bits, int expected_bits)
{
    return bits
        && expected_bits >= 12
        && v90_bits_get(bits, 0, 12) == V90_INFO_FILL_AND_SYNC_BITS;
}

static int v90_codeword_to_ucode(v90_law_t law, uint8_t codeword)
{
    int ucode;

    if (law == V90_LAW_ULAW)
        return (0xFF - codeword) & 0x7F;

    for (ucode = 0; ucode < 128; ucode++) {
        if (v90_ucode_to_alaw[ucode] == codeword
            || (v90_ucode_to_alaw[ucode] & 0x7F) == (codeword & 0x7F)) {
            return ucode;
        }
    }
    return -1;
}

static uint8_t v90_encode_octet_to_codeword(v90_state_t *s, uint8_t in_octet)
{
    uint8_t sc;
    uint8_t mag;
    int s_bit;
    int sign;
    uint8_t pcm;

    sc = v90_scramble_byte(&s->data_scrambler, in_octet);
    mag = sc & 0x7F;
    s_bit = (sc >> 7) & 1;

    sign = s_bit ^ s->prev_sign;
    s->prev_sign = sign;

    pcm = ucode_to_pcm_positive(s->law, mag);
    if (sign == 0)
        pcm &= 0x7F;
    return pcm;
}

static bool v90_decode_codeword_to_octet(v90_state_t *s, uint8_t codeword, uint8_t *out_octet)
{
    int sign;
    int scrambled_sign;
    int mag;
    uint8_t scrambled_octet;
    uint8_t plain_octet;
    int bit_idx;

    if (!out_octet)
        return false;

    sign = (codeword & 0x80) ? 1 : 0;
    scrambled_sign = sign ^ s->rx_prev_sign;
    s->rx_prev_sign = sign;

    mag = v90_codeword_to_ucode(s->law, codeword);
    if (mag < 0)
        return false;

    scrambled_octet = (uint8_t) ((mag & 0x7F) | ((scrambled_sign & 1) << 7));
    plain_octet = 0;
    for (bit_idx = 0; bit_idx < 8; bit_idx++) {
        int in_bit;
        int out_bit;

        in_bit = (scrambled_octet >> bit_idx) & 1;
        out_bit = v90_descramble_reg_bit(&s->rx_scramble_reg, in_bit);
        plain_octet |= (uint8_t) (out_bit << bit_idx);
    }
    *out_octet = plain_octet;
    return true;
}

static int v90_cp_constellation_ucode(const vpcm_cp_frame_t *cp,
                                      int frame_interval,
                                      int label)
{
    int constellation;

    if (!cp || frame_interval < 0 || frame_interval >= V90_FRAME_LEN || label < 0)
        return -1;
    constellation = cp->dfi[frame_interval];
    if (constellation < 0 || constellation >= cp->constellation_count)
        return -1;
    for (int ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
        if (!vpcm_cp_mask_get(cp->masks[constellation], ucode))
            continue;
        if (label-- == 0)
            return ucode;
    }
    return -1;
}

/* Return the far codec-output Ucode corresponding, by Table-14 label, to a
 * transmitter Ucode.  Both masks are ordered by descending Ucode (§5.4.4).
 * A valid corresponding constellation must preserve the transmitter-set
 * cardinality, so every transmitter label has exactly one output level. */
static int v90_cp_codec_output_ucode(const vpcm_cp_frame_t *cp,
                                     int frame_interval,
                                     int transmitter_ucode)
{
    int constellation;
    int label = 0;

    if (!cp || frame_interval < 0 || frame_interval >= V90_FRAME_LEN
        || transmitter_ucode < 0
        || transmitter_ucode >= VPCM_CP_MASK_BITS)
        return -1;
    if (!cp->codec_constellations_differ)
        return transmitter_ucode;
    constellation = cp->dfi[frame_interval];
    if (constellation < 0 || constellation >= cp->constellation_count)
        return -1;
    for (int ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
        if (!vpcm_cp_mask_get(cp->masks[constellation], ucode))
            continue;
        if (ucode == transmitter_ucode)
            break;
        label++;
    }
    for (int ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
        if (!vpcm_cp_mask_get(cp->codec_masks[constellation], ucode))
            continue;
        if (label-- == 0)
            return ucode;
    }
    return -1;
}

static int v90_upstream_mask_max_drn(uint16_t mask)
{
    for (int bit = 12; bit >= 0; bit--) {
        if (mask & (1U << bit))
            return bit + 2;
    }
    return 0;
}

/* The MP upstream-rate fields must offer only rates the V.34 upstream
 * receiver can actually accept.  Echoing the peer's CPt capability mask
 * verbatim let SmartLink lawfully select 33600 bps against a pump trained
 * at 31200 — its data mode then ran "Rx bit Rate - 0" and died with
 * "vpcm: Link Error" ~10 s after the first completed handshake
 * (2026-07-23, batch-4 call 2).  Mask bit i corresponds to drn i+2 =
 * (i+2)*2400 bit/s. */
static uint16_t v90_capped_upstream_mask(const v90_state_t *s)
{
    uint16_t mask = s->cp_frame.upstream_rate_mask;
    uint16_t limited = 0;

    if (s->upstream_rate_limit_bps <= 0)
        return mask;
    for (int bit = 0; bit <= 12; bit++) {
        if ((bit + 2) * 2400 <= s->upstream_rate_limit_bps)
            limited |= (uint16_t)(1U << bit);
    }
    if ((mask & limited) == 0) {
        fprintf(stderr,
                "[V90] MP: peer upstream mask 0x%04x offers no rate <= %d bps; "
                "echoing it uncapped\n",
                mask, s->upstream_rate_limit_bps);
        return mask;
    }
    return (uint16_t)(mask & limited);
}

void v90_set_upstream_rate_limit(v90_state_t *s, int bps)
{
    if (!s || bps < 0)
        return;
    if (bps != s->upstream_rate_limit_bps)
        fprintf(stderr, "[V90] MP upstream rate cap: %d bps\n", bps);
    s->upstream_rate_limit_bps = bps;
}

static bool v90_cp_mask_is_exact(const uint8_t mask[VPCM_CP_MASK_BYTES],
                                 const int *ucodes,
                                 int count)
{
    if (!mask || !ucodes || count < 0)
        return false;
    if (vpcm_cp_mask_population(mask) != count)
        return false;
    for (int i = 0; i < count; i++) {
        if (!vpcm_cp_mask_get(mask, ucodes[i]))
            return false;
    }
    return true;
}

bool v90_repair_smartlink_dummy_cpt(vpcm_cp_frame_t *cp)
{
    /* Disassembly of SLModem 2.9.11's setTrn2DummyConstel() shows that the
     * failed-design fallback is exactly eight descending Ucodes, 78..71, in
     * all six frame intervals.  Its Table-14 encoder nevertheless leaks one
     * unused transmitter-array byte (Ucode 0) and two unused codec-array
     * bytes (83 and 122) into the masks.  Match every stable field and those
     * exact leak positions before removing them; no other invalid CPt is
     * eligible for this repair. */
    static const int leaked_tx_ucodes[] = {
        0, 71, 72, 73, 74, 75, 76, 77, 78
    };
    static const int leaked_codec_ucodes[] = {
        71, 72, 73, 74, 75, 76, 77, 78, 83, 122
    };

    if (!cp
        || cp->transparent_mode_granted
        || cp->v90_compatibility
        || cp->acknowledge
        || cp->drn != 15
        || !cp->codec_alaw
        || cp->shaping_redundancy != 1
        || cp->shaping_lookahead != 1
        || cp->trn1d_gain_q3_13 != 0x2000
        || cp->shaping_a1_q1_6 != 0x40
        || cp->shaping_a2_q1_6 != 0
        || cp->shaping_b1_q1_6 != 0
        || cp->shaping_b2_q1_6 != 0
        || cp->upstream_rate_mask != 0x1FFF
        || cp->constellation_count != 1
        || !cp->codec_constellations_differ
        || !v90_cp_mask_is_exact(cp->masks[0],
                                 leaked_tx_ucodes,
                                 (int)(sizeof(leaked_tx_ucodes)
                                       / sizeof(leaked_tx_ucodes[0])))
        || !v90_cp_mask_is_exact(cp->codec_masks[0],
                                 leaked_codec_ucodes,
                                 (int)(sizeof(leaked_codec_ucodes)
                                       / sizeof(leaked_codec_ucodes[0]))))
        return false;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        if (cp->dfi[i] != 0)
            return false;
    }
    for (int constellation = 1;
         constellation < VPCM_CP_MAX_CONSTELLATIONS;
         constellation++) {
        if (vpcm_cp_mask_population(cp->masks[constellation]) != 0
            || vpcm_cp_mask_population(cp->codec_masks[constellation]) != 0)
            return false;
    }

    memset(cp->masks[0], 0, sizeof(cp->masks[0]));
    memset(cp->codec_masks[0], 0, sizeof(cp->codec_masks[0]));
    for (int ucode = 71; ucode <= 78; ucode++) {
        vpcm_cp_mask_set(cp->masks[0], ucode, true);
        vpcm_cp_mask_set(cp->codec_masks[0], ucode, true);
    }
    return vpcm_cp_validate(cp, NULL, 0);
}

static bool v90_build_mp_type0(v90_state_t *s, bool acknowledge)
{
    uint16_t crc;
    uint16_t upstream_mask;
    int upstream_drn;
    int pos = 0;

    if (!s || s->phase4_d <= 0)
        return false;
    upstream_mask = v90_capped_upstream_mask(s);
    upstream_drn = v90_upstream_mask_max_drn(upstream_mask);
    if (upstream_drn < 2 || upstream_drn > 14)
        return false;

    memset(s->mp_bits, 0, sizeof(s->mp_bits));
    for (int i = 0; i < 17; i++)
        s->mp_bits[pos++] = 1;
    s->mp_bits[pos++] = 0; /* bit 17: start */
    s->mp_bits[pos++] = 0; /* bit 18: Type 0 */
    pos += 5;              /* bits 19:23 reserved */
    for (int i = 0; i < 4; i++)
        s->mp_bits[pos++] = (uint8_t)((upstream_drn >> i) & 1);
    s->mp_bits[pos++] = 0; /* bit 28 reserved */
    s->mp_bits[pos++] = 0; /* bits 29:30, 16-state trellis */
    s->mp_bits[pos++] = 0;
    s->mp_bits[pos++] = 0; /* bit 31, Q=0 */
    s->mp_bits[pos++] = 0; /* bit 32, minimum shaping */
    s->mp_bits[pos++] = acknowledge ? 1 : 0;
    s->mp_bits[pos++] = 0; /* bit 34: start */
    s->mp_bits[pos++] = 0; /* bit 35 reserved */
    for (int i = 0; i < 13; i++)
        s->mp_bits[pos++] = (uint8_t)((upstream_mask >> i) & 1);
    s->mp_bits[pos++] = 0; /* bit 49 reserved */
    s->mp_bits[pos++] = 0; /* bit 50 reserved */
    s->mp_bits[pos++] = 0; /* bit 51: start */
    pos += 16;             /* bits 52:67 reserved */
    s->mp_bits[pos++] = 0; /* bit 68: start */

    /* V.34 10.1.2.3.2 excludes frame sync and each start bit. */
    crc = 0xFFFF;
    for (int start = 17; start < 68; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(s->mp_bits[bit], 1, crc);
    }
    for (int i = 0; i < 16; i++)
        s->mp_bits[pos++] = (uint8_t)((crc >> i) & 1);
    s->mp_bits[pos++] = 0; /* mandatory fill bit 85 */
    while ((pos % s->phase4_d) != 0 && pos < V90_MP_MAX_BITS)
        s->mp_bits[pos++] = 0;
    if (pos > V90_MP_MAX_BITS || (pos % s->phase4_d) != 0)
        return false;
    s->mp_nbits = pos;
    s->mp_bit_pos = 0;
    s->mp_acknowledge = acknowledge;
    return true;
}

static bool v90_configure_phase4_mapper(v90_state_t *s,
                                        const vpcm_cp_frame_t *cp)
{
    uint64_t product = 1;

    if (!s || !cp || !vpcm_cp_validate(cp, NULL, 0)
        || cp->v90_compatibility
        || cp->acknowledge
        || cp->shaping_redundancy > 3
        || (cp->shaping_redundancy != 0 && cp->shaping_lookahead > 3)
        || cp->drn < 4 || cp->drn > 22)
        return false;

    s->phase4_d = cp->drn + 8;
    s->phase4_sr = cp->shaping_redundancy;
    s->phase4_s = V90_FRAME_LEN - s->phase4_sr;
    s->phase4_k = s->phase4_d - s->phase4_s;
    if (s->phase4_k < 6 || s->phase4_k > 24)
        return false;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m;

        if (constellation < 0 || constellation >= cp->constellation_count)
            return false;
        m = vpcm_cp_mask_population(cp->masks[constellation]);
        if (m <= 0)
            return false;
        if (cp->codec_constellations_differ
            && vpcm_cp_mask_population(cp->codec_masks[constellation]) != m)
            return false;
        product *= (uint64_t)m;
    }
    if (product < (1ULL << s->phase4_k))
        return false;

    s->cp_frame = *cp;
    v90_scrambler_init(&s->phase4_scrambler);
    s->phase4_prev_sign = 0;
    memset(&s->phase4_shaper, 0, sizeof(s->phase4_shaper));
    s->phase4_frame_pos = V90_FRAME_LEN;
    s->phase4_mapper_ready = true;
    s->cp_ack_received = false;
    /* Native V.92 follows TRN2d with SUVd/CPd and has no V.90 Type-0 MP.
       Table 23 CPt therefore carries no V.90 upstream-rate mask. */
    if (!(s->v92_mode && s->v92_native_cpu_rx)
        && !v90_build_mp_type0(s, false)) {
        s->phase4_mapper_ready = false;
        return false;
    }
    return true;
}

static void v90_reset_negotiated_data_mapper(v90_state_t *s)
{
    if (!s)
        return;
    v90_scrambler_init(&s->data_mapper_scrambler);
    s->data_mapper_prev_sign = 0;
    memset(&s->data_shaper, 0, sizeof(s->data_shaper));
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    s->data_input_bits = 0;
    s->data_input_bit_count = 0;
}

static bool v90_configure_data_mapper(v90_state_t *s,
                                      const vpcm_cp_frame_t *cp)
{
    uint64_t product = 1;

    if (!s || !cp || !vpcm_cp_validate(cp, NULL, 0)
        || !cp->v90_compatibility
        || cp->acknowledge
        || cp->shaping_redundancy > 3
        || (cp->shaping_redundancy != 0 && cp->shaping_lookahead > 3)
        || cp->drn < 1 || cp->drn > 22)
        return false;

    s->data_mapper_d = cp->drn + 20;
    s->data_mapper_sr = cp->shaping_redundancy;
    s->data_mapper_s = V90_FRAME_LEN - s->data_mapper_sr;
    s->data_mapper_k = s->data_mapper_d - s->data_mapper_s;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m;

        if (constellation < 0 || constellation >= cp->constellation_count)
            return false;
        m = vpcm_cp_mask_population(cp->masks[constellation]);
        if (m <= 0)
            return false;
        if (cp->codec_constellations_differ
            && vpcm_cp_mask_population(cp->codec_masks[constellation]) != m)
            return false;
        product *= (uint64_t)m;
    }
    if (product < (1ULL << s->data_mapper_k))
        return false;

    s->data_cp_frame = *cp;
    s->data_cp_received = true;
    s->data_mapper_ready = true;
    v90_reset_negotiated_data_mapper(s);
    return true;
}

static bool v90_map_scrambled_frame(v90_state_t *s,
                                    const vpcm_cp_frame_t *cp,
                                    int k,
                                    const uint8_t *scrambled,
                                    int *previous_sign,
                                    uint8_t frame[V90_FRAME_LEN])
{
    uint64_t r = 0;
    int sign;

    if (!s || !cp || !scrambled || !previous_sign || !frame
        || k < 0 || k > 56)
        return false;
    for (int i = 0; i < k; i++)
        r |= (uint64_t)scrambled[6 + i] << i;

    sign = *previous_sign;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m = vpcm_cp_mask_population(cp->masks[constellation]);
        int label = (int)(r % (uint64_t)m);
        int ucode;

        r /= (uint64_t)m;
        ucode = v90_cp_constellation_ucode(cp, i, label);
        if (ucode < 0)
            return false;
        sign = (scrambled[i] & 1) ^ sign;
        frame[i] = v90_cp_transport_codeword(s, cp, ucode, sign);
    }
    if (r != 0)
        return false;
    *previous_sign = sign;
    return true;
}

typedef enum {
    V90_PHASE4_INPUT_ONES,
    V90_PHASE4_INPUT_MP,
    V90_PHASE4_INPUT_ZEROS,
    V90_PHASE4_INPUT_V92    /* native V.92 SUVd/CPd bit queue */
} v90_phase4_input_t;

static bool v90_map_shaped_scrambled_frame(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    int shaping_redundancy,
    int sign_bits,
    int modulus_bits,
    const uint8_t *scrambled,
    v90_shaper_state_t *shaper,
    uint8_t frame[V90_FRAME_LEN]);

static bool v90_phase4_fill_frame(v90_state_t *s, v90_phase4_input_t input)
{
    uint8_t scrambled[32];

    if (!s || !s->phase4_mapper_ready || s->phase4_d > (int)sizeof(scrambled))
        return false;
    for (int i = 0; i < s->phase4_d; i++) {
        int bit;

        if (input == V90_PHASE4_INPUT_ONES) {
            bit = 1;
        } else if (input == V90_PHASE4_INPUT_ZEROS) {
            bit = 0;
        } else if (input == V90_PHASE4_INPUT_V92) {
            if (s->v92_tx_pos >= s->v92_tx_nbits)
                return false;
            bit = s->v92_tx_bits[s->v92_tx_pos++] & 1;
        } else {
            if (s->mp_bit_pos >= s->mp_nbits)
                return false;
            bit = s->mp_bits[s->mp_bit_pos++] & 1;
        }
        scrambled[i] = (uint8_t)v90_scramble_bit(&s->phase4_scrambler, bit);
    }
    if (s->phase4_sr == 0) {
        if (!v90_map_scrambled_frame(s,
                                     &s->cp_frame,
                                     s->phase4_k,
                                     scrambled,
                                     &s->phase4_prev_sign,
                                     s->phase4_frame))
            return false;
    } else if (!v90_map_shaped_scrambled_frame(s,
                                                &s->cp_frame,
                                                s->phase4_sr,
                                                s->phase4_s,
                                                s->phase4_k,
                                                scrambled,
                                                &s->phase4_shaper,
                                                s->phase4_frame)) {
        return false;
    }
    s->phase4_frame_pos = 0;
    return true;
}

static uint8_t v90_phase4_codeword(v90_state_t *s, v90_phase4_input_t input)
{
    if (s->phase4_frame_pos >= V90_FRAME_LEN) {
        int delay_frames = v90_shaper_delay_frames(&s->cp_frame);
        int attempts = 0;

        while (!v90_phase4_fill_frame(s, input)) {
            attempts++;
            if (attempts > delay_frames)
                return v90_pcm_idle(s->law);
        }
    }
    return s->phase4_frame[s->phase4_frame_pos++];
}

typedef struct {
    double x1;
    double y1;
    double v1;
    double metric;
} v90_shaper_filter_state_t;

static bool v90_shaper_rule_inverts(int rule, int position)
{
    switch (rule) {
    case 1: return true;                 /* B: all */
    case 2: return (position & 1) == 0;  /* C: even */
    case 3: return (position & 1) != 0;  /* D: odd */
    default: return false;               /* A: none */
    }
}

/* §5.4.5.5-5.4.5.6 spectral-shaping metric input convention.  The shaper's
 * "select the rule that minimizes" metric feeds on PCM output levels, and two
 * readings of which levels exist:
 *
 *   transmit (default) — the levels of the PCM codewords we actually place
 *            on the digital bearer: the strict reading of §5.4.5.6 ("the
 *            linear value corresponding to PCM codes being transmitted...
 *            given in Table 1").
 *   codec    — the far codec D/A output levels the analogue receiver
 *            actually hears (the Table-14 bit-128 corresponding set; see
 *            v90_cp_transport_codeword()'s comment).  Shapes the spectrum
 *            on the subscriber loop — arguably better engineering, but
 *            nothing the spec licenses a peer to predict.
 *
 * When the two level sets differ (mu-law nonlinearity, differing codec
 * constellations) they rank rule choices differently, and the trellis memory
 * amplifies each disagreement, so the two conventions produce essentially
 * uncorrelated sign sequences (measured: 30.7% of TRN2d signs differ for the
 * call-13 SmartLink constellation, zero Ucode changes).
 *
 * NOTE (2026-07-23, revised): the round-3 TRN2REF pairs DISPROVED the
 * earlier decision-directed reading of SmartLink's trn2dKnownDemod — the
 * captured reference signs do not track the received samples (50% mismatch
 * against our +++--- Ri), so the peer regenerates the transmitter's sign
 * sequence data-aided and this convention IS visible to it.
 * CONFIRMED LIVE (2026-07-23 evening, batch-2 call 5): on the first real
 * post-DIL TRN2d window (peer CPt {91,87,83,79,72,65,53,33}, D=23, K=18),
 * the captured pairs show 0.0% sign mismatch across ~22k symbols under
 * this transmit-levels default, the peer's Error Energy settles at 11-17
 * (the codec-metric era plateaued pinned at 300-380), and the peer
 * progressed through MP into transmitting data-mode CP for the first time
 * ever.  ME_V90_SHAPER_METRIC=codec restores the far-codec metric for
 * experiments.  Cached: this runs inside the per-symbol metric loop. */
static bool v90_shaper_metric_transmit_levels(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *value = getenv("ME_V90_SHAPER_METRIC");

        cached = (value && strcmp(value, "codec") == 0) ? 0 : 1;
        fprintf(stderr,
                "[V90] ACTIVE shaper metric input: %s levels\n",
                cached ? "transmitted bearer-law" : "far-codec output");
    }
    return cached != 0;
}

static v90_shaper_filter_state_t v90_evaluate_shaper_rule(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    const int *ucodes,
    const uint8_t *initial_signs,
    int frame_interval,
    int length,
    int rule,
    v90_shaper_filter_state_t filter)
{
    const double a1 = (double)(int8_t)cp->shaping_a1_q1_6 / 64.0;
    const double a2 = (double)(int8_t)cp->shaping_a2_q1_6 / 64.0;
    const double b1 = (double)(int8_t)cp->shaping_b1_q1_6 / 64.0;
    const double b2 = (double)(int8_t)cp->shaping_b2_q1_6 / 64.0;

    filter.metric = 0.0;
    for (int i = 0; i < length; i++) {
        int sign = initial_signs[i] ^ v90_shaper_rule_inverts(rule, i);
        double x;
        double y;
        double v;

        if (v90_shaper_metric_transmit_levels()) {
            x = (double)v90_pcm_to_linear(
                s->law,
                v90_cp_transport_codeword(s, cp, ucodes[i], sign));
        } else {
            int output_ucode = v90_cp_codec_output_ucode(
                cp, (frame_interval + i) % V90_FRAME_LEN, ucodes[i]);
            v90_law_t output_law = cp->codec_alaw
                                 ? V90_LAW_ALAW : V90_LAW_ULAW;

            if (output_ucode < 0) {
                filter.metric = HUGE_VAL;
                return filter;
            }
            x = (double)v90_pcm_to_linear(
                output_law,
                v90_pcm_signed_codeword(output_law, output_ucode, sign));
        }
        y = x - b1 * filter.x1 + a1 * filter.y1;
        v = y - b2 * filter.y1 + a2 * filter.v1;

        filter.metric += v * v;
        filter.x1 = x;
        filter.y1 = y;
        filter.v1 = v;
    }
    return filter;
}

static void v90_build_initial_shaping_signs(v90_shaper_state_t *shaper,
                                            int shaping_redundancy,
                                            const uint8_t *sign_bits,
                                            uint8_t signs[V90_FRAME_LEN])
{
    int frame_length = V90_FRAME_LEN / shaping_redundancy;
    int sign_pos = 0;

    for (int frame = 0; frame < shaping_redundancy; frame++) {
        uint8_t p[V90_FRAME_LEN] = {0};

        for (int k = 1; k < frame_length; k++)
            p[k] = sign_bits[sign_pos++];
        for (int k = 0; k < frame_length; k++) {
            int p_prime = p[k];
            int t;

            if (k & 1) {
                p_prime ^= shaper->prev_odd;
                shaper->prev_odd = p_prime;
            }
            t = p_prime ^ shaper->prev_t[k];
            shaper->prev_t[k] = (uint8_t)t;
            signs[frame * frame_length + k] = (uint8_t)t;
        }
    }
}

static double v90_preview_shaper_rules(v90_state_t *s,
                                       const vpcm_cp_frame_t *cp,
                                       const int *ucodes,
                                       const uint8_t *initial,
                                       int frame_interval,
                                       int frame_length,
                                       int frame_count,
                                       int trellis_state,
                                       v90_shaper_filter_state_t filter)
{
    double best_metric = HUGE_VAL;
    int rules[2] = {
        trellis_state == 0 ? 0 : 2,
        trellis_state == 0 ? 1 : 3
    };

    if (frame_count <= 0)
        return 0.0;
    for (int choice = 0; choice < 2; choice++) {
        int rule = rules[choice];
        int next_state = (rule == 1 || rule == 3) ? 1 : 0;
        v90_shaper_filter_state_t next = v90_evaluate_shaper_rule(
            s, cp, ucodes, initial, frame_interval,
            frame_length, rule, filter);
        double metric = next.metric;

        if (frame_count > 1) {
            metric += v90_preview_shaper_rules(
                s,
                cp,
                ucodes + frame_length,
                initial + frame_length,
                (frame_interval + frame_length) % V90_FRAME_LEN,
                frame_length,
                frame_count - 1,
                next_state,
                next);
        }
        if (metric < best_metric)
            best_metric = metric;
    }
    return best_metric;
}

static int v90_select_shaper_rule(v90_state_t *s,
                                  const vpcm_cp_frame_t *cp,
                                  const v90_shaper_state_t *shaper,
                                  const int *ucodes,
                                  const uint8_t *initial,
                                  int frame_interval,
                                  int frame_length,
                                  int lookahead,
                                  v90_shaper_filter_state_t *selected_filter)
{
    v90_shaper_filter_state_t base = {
        shaper->x1,
        shaper->y1,
        shaper->v1,
        0.0
    };
    double best_metric = HUGE_VAL;
    int first_rules[2];
    int best_rule = 0;

    first_rules[0] = (shaper->trellis_state == 0) ? 0 : 2;
    first_rules[1] = (shaper->trellis_state == 0) ? 1 : 3;
    for (int first_idx = 0; first_idx < 2; first_idx++) {
        int first_rule = first_rules[first_idx];
        int next_state = (first_rule == 1 || first_rule == 3) ? 1 : 0;
        v90_shaper_filter_state_t current;
        double metric;

        current = v90_evaluate_shaper_rule(s,
                                           cp,
                                           ucodes,
                                           initial,
                                           frame_interval,
                                           frame_length,
                                           first_rule,
                                           base);
        metric = current.metric;
        if (lookahead > 0)
            metric += v90_preview_shaper_rules(s,
                                               cp,
                                               ucodes + frame_length,
                                               initial + frame_length,
                                               (frame_interval + frame_length)
                                                   % V90_FRAME_LEN,
                                               frame_length,
                                               lookahead,
                                               next_state,
                                               current);
        if (metric < best_metric) {
            best_metric = metric;
            best_rule = first_rule;
            *selected_filter = current;
        }
    }
    return best_rule;
}

static void v90_shape_data_signs(v90_state_t *s,
                                 const vpcm_cp_frame_t *cp,
                                 int shaping_redundancy,
                                 v90_shaper_state_t *shaper,
                                 const int *ucodes,
                                 const uint8_t *initial,
                                 uint8_t signs[V90_FRAME_LEN])
{
    int frame_length = V90_FRAME_LEN / shaping_redundancy;

    for (int frame = 0; frame < shaping_redundancy; frame++) {
        int offset = frame * frame_length;
        v90_shaper_filter_state_t selected;
        int rule;

        rule = v90_select_shaper_rule(s,
                                      cp,
                                      shaper,
                                      ucodes + offset,
                                      initial + offset,
                                      offset,
                                      frame_length,
                                      cp->shaping_lookahead,
                                      &selected);
        for (int k = 0; k < frame_length; k++)
            signs[offset + k] = initial[offset + k]
                              ^ v90_shaper_rule_inverts(rule, k);
        shaper->trellis_state = (rule == 1 || rule == 3) ? 1 : 0;
        shaper->x1 = selected.x1;
        shaper->y1 = selected.y1;
        shaper->v1 = selected.v1;
    }
}

static bool v90_map_shaped_scrambled_frame(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    int shaping_redundancy,
    int sign_bits,
    int modulus_bits,
    const uint8_t *scrambled,
    v90_shaper_state_t *shaper,
    uint8_t frame[V90_FRAME_LEN])
{
    uint8_t initial_signs[V90_FRAME_LEN];
    uint8_t shaped_signs[V90_FRAME_LEN];
    int ucodes[V90_FRAME_LEN];
    int delay_frames;
    uint64_t r = 0;

    if (!s || !cp || !scrambled || !shaper || !frame
        || shaping_redundancy < 1 || shaping_redundancy > 3
        || sign_bits != V90_FRAME_LEN - shaping_redundancy
        || modulus_bits < 0 || modulus_bits > 56)
        return false;
    for (int i = 0; i < modulus_bits; i++)
        r |= (uint64_t)scrambled[sign_bits + i] << i;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m = vpcm_cp_mask_population(cp->masks[constellation]);
        int label = (int)(r % (uint64_t)m);

        r /= (uint64_t)m;
        ucodes[i] = v90_cp_constellation_ucode(cp, i, label);
        if (ucodes[i] < 0)
            return false;
    }
    if (r != 0)
        return false;
    v90_build_initial_shaping_signs(shaper,
                                    shaping_redundancy,
                                    scrambled,
                                    initial_signs);
    delay_frames = v90_shaper_delay_frames(cp);
    if (shaper->pending_count >= (int)(sizeof(shaper->pending_ucodes)
                                      / sizeof(shaper->pending_ucodes[0])))
        return false;
    memcpy(shaper->pending_ucodes[shaper->pending_count],
           ucodes,
           sizeof(ucodes));
    memcpy(shaper->pending_signs[shaper->pending_count],
           initial_signs,
           sizeof(initial_signs));
    shaper->pending_count++;
    if (shaper->pending_count <= delay_frames)
        return false;

    v90_shape_data_signs(s,
                         cp,
                         shaping_redundancy,
                         shaper,
                         &shaper->pending_ucodes[0][0],
                         &shaper->pending_signs[0][0],
                         shaped_signs);
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        frame[i] = v90_cp_transport_codeword(s,
                                             cp,
                                             shaper->pending_ucodes[0][i],
                                             shaped_signs[i]);
    }
    shaper->pending_count--;
    if (shaper->pending_count > 0) {
        memmove(shaper->pending_ucodes[0],
                shaper->pending_ucodes[1],
                (size_t)shaper->pending_count
                    * sizeof(shaper->pending_ucodes[0]));
        memmove(shaper->pending_signs[0],
                shaper->pending_signs[1],
                (size_t)shaper->pending_count
                    * sizeof(shaper->pending_signs[0]));
    }
    return true;
}

static bool v90_data_mapper_fill_frame(v90_state_t *s, uint64_t input_bits)
{
    uint8_t scrambled[48];

    if (!s || !s->data_mapper_ready
        || s->data_mapper_d <= 0
        || s->data_mapper_d > (int)sizeof(scrambled))
        return false;
    for (int i = 0; i < s->data_mapper_d; i++) {
        int bit = (int)((input_bits >> i) & 1U);

        scrambled[i] = (uint8_t)v90_scramble_bit(&s->data_mapper_scrambler, bit);
    }
    if (s->data_mapper_sr == 0) {
        if (!v90_map_scrambled_frame(s,
                                     &s->data_cp_frame,
                                     s->data_mapper_k,
                                     scrambled,
                                     &s->data_mapper_prev_sign,
                                     s->data_mapper_frame))
            return false;
    } else if (!v90_map_shaped_scrambled_frame(s,
                                                &s->data_cp_frame,
                                                s->data_mapper_sr,
                                                s->data_mapper_s,
                                                s->data_mapper_k,
                                                scrambled,
                                                &s->data_shaper,
                                                s->data_mapper_frame)) {
        return false;
    }
    s->data_mapper_frame_pos = 0;
    return true;
}

static uint8_t v90_data_mapper_ones_codeword(v90_state_t *s)
{
    if (s->data_mapper_frame_pos >= V90_FRAME_LEN) {
        uint64_t ones = (1ULL << s->data_mapper_d) - 1ULL;
        int delay_frames = v90_shaper_delay_frames(&s->data_cp_frame);
        int attempts = 0;

        while (!v90_data_mapper_fill_frame(s, ones)) {
            attempts++;
            if (attempts > delay_frames)
                return v90_pcm_idle(s->law);
        }
    }
    return s->data_mapper_frame[s->data_mapper_frame_pos++];
}

void v90_info0a_init(v90_info0a_t *info)
{
    if (!info)
        return;
    memset(info, 0, sizeof(*info));
    info->support_2743 = true;
    info->support_2800 = true;
    info->support_3429 = true;
    info->support_3000_low = true;
    info->support_3000_high = true;
    info->support_3200_low = true;
    info->support_3200_high = true;
    info->rate_3429_allowed = true;
    info->support_power_reduction = true;
    info->max_baud_rate_difference = 0;
    info->from_cme_modem = false;
    info->support_1664_point_constellation = true;
    info->tx_clock_source = 0;
    info->acknowledge_info0d = false;
}

void v90_info1a_init(v90_info1a_t *info)
{
    if (!info)
        return;
    memset(info, 0, sizeof(*info));
    info->md = 0;
    info->u_info = 78;
    info->upstream_symbol_rate_code = 4;
    info->downstream_rate_code = 6;
    info->freq_offset = 0;
}

bool v90_info0a_validate(const v90_info0a_t *info)
{
    if (!info)
        return false;
    return info->max_baud_rate_difference <= 7
        && info->tx_clock_source <= 3;
}

bool v90_info1a_validate(const v90_info1a_t *info)
{
    if (!info)
        return false;
    return info->md <= 0x7F
        && info->u_info <= 0x7F
        && info->upstream_symbol_rate_code <= 0x7
        && info->downstream_rate_code <= 0x7
        && info->freq_offset >= -512
        && info->freq_offset <= 511;
}

bool v90_build_info0a_bits(uint8_t *buf, int buf_len, const v90_info0a_t *info)
{
    int bit_pos;
    uint16_t crc;

    if (!buf || !v90_info0a_validate(info) || buf_len < ((V90_INFO0A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, V90_INFO_FILL_AND_SYNC_BITS, 12);
    v90_bits_put(buf, &bit_pos, info->support_2743 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_2800 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3429 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3000_low ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3000_high ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3200_low ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3200_high ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->rate_3429_allowed ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_power_reduction ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->max_baud_rate_difference & 0x7U, 3);
    v90_bits_put(buf, &bit_pos, info->from_cme_modem ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_1664_point_constellation ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->tx_clock_source & 0x3U, 2);
    v90_bits_put(buf, &bit_pos, info->acknowledge_info0d ? 1U : 0U, 1);
    crc = v90_crc_bit_block(buf, 12, 28, 0xFFFF);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == V90_INFO0A_BITS;
}

bool v90_build_info1a_bits(uint8_t *buf, int buf_len, const v90_info1a_t *info)
{
    int bit_pos;
    uint16_t crc;
    uint16_t freq_bits;

    if (!buf || !v90_info1a_validate(info) || buf_len < ((V90_INFO1A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, V90_INFO_FILL_AND_SYNC_BITS, 12);
    v90_bits_put(buf, &bit_pos, 0, 6);
    v90_bits_put(buf, &bit_pos, info->md & 0x7FU, 7);
    v90_bits_put(buf, &bit_pos, info->u_info & 0x7FU, 7);
    v90_bits_put(buf, &bit_pos, 0, 2);
    v90_bits_put(buf, &bit_pos, info->upstream_symbol_rate_code & 0x7U, 3);
    v90_bits_put(buf, &bit_pos, info->downstream_rate_code & 0x7U, 3);
    freq_bits = (uint16_t) info->freq_offset;
    if (info->freq_offset < 0)
        freq_bits = (uint16_t) (0x400 + info->freq_offset);
    v90_bits_put(buf, &bit_pos, freq_bits & 0x3FFU, 10);
    crc = v90_crc_bit_block(buf, 12, 49, 0xFFFF);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == V90_INFO1A_BITS;
}

bool v90_parse_info0a_bits(v90_info0a_t *out, const uint8_t *bits, int bit_len)
{
    v90_info0a_t parsed;
    uint16_t crc_field;
    uint16_t crc_remainder;

    if (!out || !bits || bit_len < V90_INFO0A_BITS)
        return false;
    if (!v90_info_fill_and_sync_ok(bits, bit_len))
        return false;
    if (v90_bits_get(bits, 45, 4) != 0xF)
        return false;

    memset(&parsed, 0, sizeof(parsed));
    parsed.support_2743 = v90_bits_get(bits, 12, 1) != 0;
    parsed.support_2800 = v90_bits_get(bits, 13, 1) != 0;
    parsed.support_3429 = v90_bits_get(bits, 14, 1) != 0;
    parsed.support_3000_low = v90_bits_get(bits, 15, 1) != 0;
    parsed.support_3000_high = v90_bits_get(bits, 16, 1) != 0;
    parsed.support_3200_low = v90_bits_get(bits, 17, 1) != 0;
    parsed.support_3200_high = v90_bits_get(bits, 18, 1) != 0;
    parsed.rate_3429_allowed = v90_bits_get(bits, 19, 1) != 0;
    parsed.support_power_reduction = v90_bits_get(bits, 20, 1) != 0;
    parsed.max_baud_rate_difference = (uint8_t) v90_bits_get(bits, 21, 3);
    parsed.from_cme_modem = v90_bits_get(bits, 24, 1) != 0;
    parsed.support_1664_point_constellation = v90_bits_get(bits, 25, 1) != 0;
    parsed.tx_clock_source = (uint8_t) v90_bits_get(bits, 26, 2);
    parsed.acknowledge_info0d = v90_bits_get(bits, 28, 1) != 0;

    if (!v90_info0a_validate(&parsed))
        return false;
    crc_field = (uint16_t) v90_bits_get(bits, 29, 16);
    crc_remainder = v90_crc_bit_block(bits, 12, 28, 0xFFFF);
    if (crc_field != crc_remainder)
        return false;

    *out = parsed;
    return true;
}

bool v90_parse_info1a_bits(v90_info1a_t *out, const uint8_t *bits, int bit_len)
{
    v90_info1a_t parsed;
    int raw_freq;
    uint16_t crc_field;
    uint16_t crc_remainder;

    if (!out || !bits || bit_len < V90_INFO1A_BITS)
        return false;
    if (!v90_info_fill_and_sync_ok(bits, bit_len))
        return false;
    if (v90_bits_get(bits, 66, 4) != 0xF)
        return false;
    if (v90_bits_get(bits, 12, 6) != 0 || v90_bits_get(bits, 32, 2) != 0)
        return false;

    memset(&parsed, 0, sizeof(parsed));
    parsed.md = (uint8_t) v90_bits_get(bits, 18, 7);
    parsed.u_info = (uint8_t) v90_bits_get(bits, 25, 7);
    parsed.upstream_symbol_rate_code = (uint8_t) v90_bits_get(bits, 34, 3);
    parsed.downstream_rate_code = (uint8_t) v90_bits_get(bits, 37, 3);
    raw_freq = v90_bits_get(bits, 40, 10);
    if (raw_freq & 0x200)
        raw_freq -= 0x400;
    parsed.freq_offset = (int16_t) raw_freq;

    if (!v90_info1a_validate(&parsed))
        return false;
    crc_field = (uint16_t) v90_bits_get(bits, 50, 16);
    crc_remainder = v90_crc_bit_block(bits, 12, 49, 0xFFFF);
    if (crc_field != crc_remainder)
        return false;

    *out = parsed;
    return true;
}

bool v90_info0a_build_diag(const v90_info0a_t *info, v90_info0a_diag_t *diag)
{
    int i;
    uint8_t packed[(V90_INFO0A_BITS + 7) / 8];

    if (!diag || !v90_build_info0a_bits(packed, (int) sizeof(packed), info))
        return false;
    memset(diag, 0, sizeof(*diag));
    diag->frame = *info;
    for (i = 0; i < V90_INFO0A_BITS; i++)
        diag->bits[i] = (uint8_t) ((packed[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(packed, 29, 16);
    diag->crc_remainder = v90_crc_bit_block(packed, 12, 28, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(packed, V90_INFO0A_BITS)
                          && v90_bits_get(packed, 45, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    return true;
}

bool v90_info1a_build_diag(const v90_info1a_t *info, v90_info1a_diag_t *diag)
{
    int i;
    uint8_t packed[(V90_INFO1A_BITS + 7) / 8];

    if (!diag || !v90_build_info1a_bits(packed, (int) sizeof(packed), info))
        return false;
    memset(diag, 0, sizeof(*diag));
    diag->frame = *info;
    for (i = 0; i < V90_INFO1A_BITS; i++)
        diag->bits[i] = (uint8_t) ((packed[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(packed, 50, 16);
    diag->crc_remainder = v90_crc_bit_block(packed, 12, 49, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(packed, V90_INFO1A_BITS)
                          && v90_bits_get(packed, 66, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    return true;
}

bool v90_info0a_decode_diag(const uint8_t *bits, int bit_len, v90_info0a_diag_t *diag)
{
    int i;
    v90_info0a_t parsed;

    if (!diag || !bits || bit_len < V90_INFO0A_BITS)
        return false;
    memset(diag, 0, sizeof(*diag));
    for (i = 0; i < V90_INFO0A_BITS; i++)
        diag->bits[i] = (uint8_t) ((bits[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(bits, 29, 16);
    diag->crc_remainder = v90_crc_bit_block(bits, 12, 28, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(bits, bit_len)
                          && v90_bits_get(bits, 45, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    if (!diag->valid || !v90_parse_info0a_bits(&parsed, bits, bit_len))
        return false;
    diag->frame = parsed;
    return true;
}

bool v90_info1a_decode_diag(const uint8_t *bits, int bit_len, v90_info1a_diag_t *diag)
{
    int i;
    v90_info1a_t parsed;

    if (!diag || !bits || bit_len < V90_INFO1A_BITS)
        return false;
    memset(diag, 0, sizeof(*diag));
    for (i = 0; i < V90_INFO1A_BITS; i++)
        diag->bits[i] = (uint8_t) ((bits[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(bits, 50, 16);
    diag->crc_remainder = v90_crc_bit_block(bits, 12, 49, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(bits, bit_len)
                          && v90_bits_get(bits, 66, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    if (!diag->valid || !v90_parse_info1a_bits(&parsed, bits, bit_len))
        return false;
    diag->frame = parsed;
    return true;
}

/* ---- Jd frame construction (Table 13) ---- */

/* Interop backoff knob.  ITU-T V.90 Table 13 bit k (k=0..22) of the downstream
 * data-signalling-rate capability mask advertises rate 28000 + k*8000/6 bps.
 * The analogue peer must not request a downstream constellation above what we
 * advertise here, so capping the mask makes it select a sparser, more robust
 * CPt drn that both ends agree on -- letting its Phase-4 PDSNR gate clear when
 * the maximal constellation's minimum distance is too small for our TX signal.
 * ME_V90_MAX_DOWNSTREAM_RATE is the cap in bps; 0/unset advertises the full
 * 0x7FFFFF mask (the standards-compliant default). */
static int v90_max_downstream_rate_bps(void)
{
    const char *value = getenv("ME_V90_MAX_DOWNSTREAM_RATE");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 28000 && parsed <= 56000)
            return (int) parsed;
    }
    return 0;
}

/* Rate advertised by downstream-rate-mask bit k, in bps (Table 13). */
static int v90_jd_rate_bit_bps(int k)
{
    return 28000 + (k * 8000) / 6;
}

/* Whether mask bit k should be set given a cap.  The floor rate (k=0, 28000)
 * is always advertised so the mask stays non-empty and contiguous. */
static bool v90_jd_rate_bit_enabled(int k, int cap_bps)
{
    if (cap_bps <= 0 || k == 0)
        return true;
    return v90_jd_rate_bit_bps(k) <= cap_bps;
}

/* Jd bits 49:50, Table 13: 1..3.  Default unchanged at 1 pending a live result;
 * ME_V90_JD_SHAPING_LOOKAHEAD=3 matches the Eicon card's working downstream. */
static int v90_jd_shaping_lookahead(void)
{
    static int cached;

    if (cached == 0) {
        const char *value = getenv("ME_V90_JD_SHAPING_LOOKAHEAD");

        cached = 1;
        if (value && *value) {
            char *end;
            long parsed = strtol(value, &end, 10);

            if (end != value && *end == '\0' && parsed >= 1 && parsed <= 3)
                cached = (int) parsed;
        }
    }
    return cached;
}

static void v90_build_jd(v90_state_t *s)
{
    /* Build the 72-bit Jd frame per V.90 Table 13.
     * Bits 0:16   = Frame sync (17 ones)
     * Bit  17     = Start bit (0)
     * Bits 18:33  = Data signalling rate capability mask
     * Bit  34     = Start bit (0)
     * Bits 35:46  = Rate capability mask continued + reserved
     * Bit  47     = Constellation size for training (0=4pt, 1=16pt)
     * Bit  48     = Constellation size for renegotiation
     * Bits 49:50  = Spectral shaping lookahead (1-3)
     * Bit  51     = Start bit (0)
     * Bits 52:67  = CRC
     * Bits 68:71  = Fill (0000)
     */
    memset(s->jd_bits, 0, sizeof(s->jd_bits));

    /* We pack bit-by-bit, LSB first within each byte */
    int pos = 0;

    /* Bits 0:16 — 17 sync bits (all 1) */
    for (int i = 0; i < 17; i++)
        s->jd_bits[pos/8] |= (1 << (pos%8)), pos++;

    /* Bit 17 — start bit (0) */
    pos++;

    /* Bits 18:33 — data signalling rate capability mask.
     * Mask bit k (Jd bit 18+k) advertises rate 28000 + k*8000/6 bps, so this
     * first group covers 28000 (bit 18) through 48000 (bit 33).  Uncapped we
     * enable all; ME_V90_MAX_DOWNSTREAM_RATE clears the bits above the cap so
     * the peer selects a lower, more robust downstream constellation. */
    int rate_cap = v90_max_downstream_rate_bps();
    int rate_mask_top = 0;
    for (int i = 18; i <= 33; i++) {
        if (v90_jd_rate_bit_enabled(i - 18, rate_cap)) {
            s->jd_bits[pos/8] |= (1 << (pos%8));
            rate_mask_top = i - 18;
        }
        pos++;
    }

    /* Bit 34 — start bit (0) */
    pos++;

    /* Bits 35:46 — continued rate mask + reserved.
     * Bits 35:40 are the final six bits of the 22-bit downstream-rate
     * capability mask (mask bits k=16..21, i.e. Jd bit 19+k).  Bits 41:46 are
     * reserved by Table 13 and must be zero. */
    for (int i = 35; i <= 40; i++) {
        if (v90_jd_rate_bit_enabled(i - 19, rate_cap)) {
            s->jd_bits[pos/8] |= (1 << (pos%8));
            rate_mask_top = i - 19;
        }
        pos++;
    }
    pos += 6; /* bits 41:46 reserved = 0 */

    if (rate_cap > 0 && !s->jd_rate_cap_logged) {
        fprintf(stderr,
                "[V90] Jd downstream-rate cap: %d bps -> mask top bit k=%d "
                "(%d bps); full mask disabled\n",
                rate_cap, rate_mask_top, v90_jd_rate_bit_bps(rate_mask_top));
        s->jd_rate_cap_logged = true;
    }

    /* Bit 47 — constellation size for training: 0=4-point */
    pos++;

    /* Bit 48 — constellation size for renegotiation: 0=4-point */
    pos++;

    /* Bits 49:50 — Table 13: "A number between 1 and 3 indicating the digital
     * modem's maximum lookahead for spectral shaping", LSB first.
     *
     * This was hardcoded to 1 as "the minimum mandatory" value.  1 is legal,
     * but the only foreign V.90 digital modem we have -- the Eicon Diva Server,
     * captured in artifacts/eicon-digital-downstream/, whose downstream a USR
     * Courier V.Everything answered with CONNECT -- sends **3**, and that is the
     * *only* field in which its Jd frame differs from ours.  Both frames are
     * otherwise bit-identical: same 17-ones sync, same rate masks, same 4-point
     * constellation bits, same fill, valid CRC.  Against the same modem model
     * ours is received for 4.7 s and never answered (docs section 25).
     *
     * The parallel worth noting is §5.4.5's Sr on this project's own analogue
     * side: a legal-looking shaping parameter (Sr = 1) made every Phase 4
     * constellation unbuildable and the peer simply never advanced.  A
     * lookahead the peer cannot plan around would fail the same silent way. */
    {
        int ld = v90_jd_shaping_lookahead();

        if (ld & 1)
            s->jd_bits[pos/8] |= (1 << (pos%8));
        pos++;
        if (ld & 2)
            s->jd_bits[pos/8] |= (1 << (pos%8));
        pos++;
    }

    /* Bit 51 — start bit (0) */
    pos++;

    /* Bits 52:67 — CRC.  V.34 10.1.2.3.2 excludes frame sync and
     * start/fill bits, so only the two 16-bit information groups enter the
     * generator.  The CRC field is serialized LSB first. */
    {
        uint16_t crc = 0xFFFF;
        for (int i = 18; i <= 33; i++)
            crc = crc_itu16_bits(
                (s->jd_bits[i / 8] >> (i % 8)) & 1, 1, crc);
        for (int i = 35; i <= 50; i++)
            crc = crc_itu16_bits(
                (s->jd_bits[i / 8] >> (i % 8)) & 1, 1, crc);
        for (int i = 0; i < 16; i++) {
            if ((crc >> i) & 1)
                s->jd_bits[(52+i)/8] |= (1 << ((52+i)%8));
        }
    }

    /* Bits 68:71 — fill (0000), already zero */
}

/* V.92 Table 22.  Jp has the same framing and CRC coverage as Jd, but
 * replaces the downstream-rate mask with a fractional Su extension and
 * advertises the 4-point TRN2u choice used by this implementation. */
static void v90_build_jp(v90_state_t *s)
{
    int pos = 0;
    uint16_t crc = 0xFFFF;

    memset(s->jp_bits, 0, sizeof(s->jp_bits));
    for (int i = 0; i < 17; i++)
        s->jp_bits[pos / 8] |= (uint8_t)(1U << (pos % 8)), pos++;
    pos++; /* start bit 17 */
    pos += 16; /* 18:33 fractional Su extension: zero */
    pos++; /* start bit 34 */
    pos += 12; /* 35:46 reserved */
    s->jp_bits[pos / 8] |= (uint8_t)(1U << (pos % 8));
    pos++; /* 47: Jd/Jp identifier = Jp */
    pos++; /* 48: 4-point training constellation */
    pos++; /* 49: 4-point renegotiation constellation */
    pos++; /* 50 reserved */
    pos++; /* start bit 51 */

    for (int i = 18; i <= 33; i++)
        crc = crc_itu16_bits((s->jp_bits[i / 8] >> (i % 8)) & 1, 1, crc);
    for (int i = 35; i <= 50; i++)
        crc = crc_itu16_bits((s->jp_bits[i / 8] >> (i % 8)) & 1, 1, crc);
    for (int i = 0; i < 16; i++) {
        if ((crc >> i) & 1)
            s->jp_bits[(52 + i) / 8] |= (uint8_t)(1U << ((52 + i) % 8));
    }
}

/* ---- Phase 3 TX sample generation ---- */

/* Get next Jd bit, wrap around for continuous repetition */
static int v90_get_jd_bit(v90_state_t *s)
{
    int bit = (s->jd_bits[s->jd_bit_pos / 8] >> (s->jd_bit_pos % 8)) & 1;
    s->jd_bit_pos++;
    if (s->jd_bit_pos >= V90_JD_BITS)
        s->jd_bit_pos = 0;
    return bit;
}

static int v90_get_jp_bit(v90_state_t *s)
{
    int bit = (s->jp_bits[s->jp_bit_pos / 8] >> (s->jp_bit_pos % 8)) & 1;

    s->jp_bit_pos++;
    if (s->jp_bit_pos >= V90_JD_BITS)
        s->jp_bit_pos = 0;
    return bit;
}

static int v90_get_packed_bit(const uint8_t *bits, int bit_pos)
{
    return (bits[bit_pos / 8] >> (bit_pos % 8)) & 1;
}

static uint32_t v90_get_packed_bits(const uint8_t *bits, int bit_pos, int bit_count)
{
    uint32_t value = 0;

    for (int i = 0; i < bit_count; i++)
        value |= (uint32_t)(v90_get_packed_bit(bits, bit_pos + i) << i);
    return value;
}

static bool v90_expect_zero_bit(const uint8_t *bits, int bit_len, int bit_pos)
{
    return bit_pos < bit_len && v90_get_packed_bit(bits, bit_pos) == 0;
}

static bool v90_expect_zero_range(const uint8_t *bits, int bit_len, int bit_pos, int bit_count)
{
    if (bit_pos + bit_count > bit_len)
        return false;
    for (int i = 0; i < bit_count; i++) {
        if (v90_get_packed_bit(bits, bit_pos + i) != 0)
            return false;
    }
    return true;
}

static bool v90_copy_framed_pattern(uint8_t *out,
                                    int out_len,
                                    const uint8_t *bits,
                                    int bit_len,
                                    int start_bit_pos)
{
    int pos = start_bit_pos;
    int copied = 0;

    while (copied < out_len) {
        int chunk = out_len - copied;
        int pad;
        if (chunk > 16)
            chunk = 16;
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + chunk > bit_len)
            return false;
        for (int i = 0; i < chunk; i++)
            out[copied + i] = (uint8_t)v90_get_packed_bit(bits, pos + i);
        pos += chunk;
        pad = 16 - chunk;
        if (pad > 0) {
            if (!v90_expect_zero_range(bits, bit_len, pos, pad))
                return false;
            pos += pad;
        }
        copied += chunk;
    }
    return true;
}

static bool v90_parse_table12_byte_pairs(uint8_t *out,
                                         int out_count,
                                         const uint8_t *bits,
                                         int bit_len,
                                         int start_bit_pos)
{
    int pos = start_bit_pos;
    int index = 0;

    while (index < out_count) {
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + 8 > bit_len)
            return false;
        out[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
        pos += 7;
        if (index < out_count) {
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
            if (pos + 8 > bit_len)
                return false;
            out[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
            pos += 7;
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
        } else {
            if (!v90_expect_zero_range(bits, bit_len, pos, 9))
                return false;
            pos += 9;
        }
    }

    return true;
}

static bool v90_parse_table12_training_ucodes(v90_dil_desc_t *out,
                                              const uint8_t *bits,
                                              int bit_len,
                                              int start_bit_pos)
{
    int pos = start_bit_pos;
    int index = 0;

    while (index < out->n) {
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + 8 > bit_len)
            return false;
        out->train_u[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
        pos += 7;
        if (index < out->n) {
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
            if (pos + 8 > bit_len)
                return false;
            out->train_u[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
            pos += 7;
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
        } else {
            if (!v90_expect_zero_range(bits, bit_len, pos, 9))
                return false;
            pos += 9;
        }
    }

    if (!v90_expect_zero_bit(bits, bit_len, pos))
        return false;
    pos++;
    if (pos + 16 > bit_len)
        return false;

    return true;
}

/* V.34 §10.1.2.3.2 CRC used by the Ja/DIL descriptor.  Frame-sync,
 * start, and final fill bits do not enter the generator.  From Table 12,
 * every seventeenth bit beginning at bit 51 is a start bit. */
static uint16_t v90_dil_crc16_bits(const uint8_t *bits, int crc_start)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < crc_start; i++) {
        if (i < 18 || i == 34 || (i >= 51 && ((i - 51) % 17) == 0))
            continue;
        crc = crc_itu16_bits((uint8_t)v90_get_packed_bit(bits, i), 1, crc);
    }
    return crc;
}

static void v90_put_zero_range(uint8_t *buf, int *bit_pos, int count)
{
    if (!buf || !bit_pos || count <= 0)
        return;
    *bit_pos += count;
}

static void v90_put_framed_pattern(uint8_t *buf, int *bit_pos, const uint8_t *pattern, int pattern_len)
{
    int copied;

    if (!buf || !bit_pos || !pattern || pattern_len < 0)
        return;

    copied = 0;
    while (copied < pattern_len) {
        int chunk;

        chunk = pattern_len - copied;
        if (chunk > 16)
            chunk = 16;
        v90_bits_put(buf, bit_pos, 0, 1);
        for (int i = 0; i < chunk; i++)
            v90_bits_put(buf, bit_pos, pattern[copied + i] ? 1U : 0U, 1);
        v90_put_zero_range(buf, bit_pos, 16 - chunk);
        copied += chunk;
    }
}

static void v90_put_framed_byte_pairs(uint8_t *buf, int *bit_pos, const uint8_t *values, int value_count)
{
    int index;

    if (!buf || !bit_pos || !values || value_count < 0)
        return;

    index = 0;
    while (index < value_count) {
        v90_bits_put(buf, bit_pos, 0, 1);
        v90_bits_put(buf, bit_pos, values[index++] & 0x7FU, 7);
        if (index < value_count) {
            v90_bits_put(buf, bit_pos, 0, 1);
            v90_bits_put(buf, bit_pos, values[index++] & 0x7FU, 7);
            v90_bits_put(buf, bit_pos, 0, 1);
        } else {
            v90_put_zero_range(buf, bit_pos, 9);
        }
    }
}

static void v90_put_framed_training_ucodes(uint8_t *buf, int *bit_pos, const v90_dil_desc_t *desc)
{
    int index;

    if (!buf || !bit_pos || !desc)
        return;

    index = 0;
    while (index < desc->n) {
        int have_second;

        v90_bits_put(buf, bit_pos, 0, 1);
        v90_bits_put(buf, bit_pos, desc->train_u[index++] & 0x7FU, 7);
        have_second = (index < desc->n);
        if (have_second) {
            v90_bits_put(buf, bit_pos, 0, 1);
            v90_bits_put(buf, bit_pos, desc->train_u[index++] & 0x7FU, 7);
            v90_bits_put(buf, bit_pos, 0, 1);
        } else {
            v90_put_zero_range(buf, bit_pos, 9);
        }
    }
}

static inline int v90_clamp_positive(int v, int max_v)
{
    if (v < 1)
        return 1;
    if (v > max_v)
        return max_v;
    return v;
}

static inline int v90_dil_uchord_index(int training_ucode)
{
    int idx = (training_ucode >> 4);
    if (idx < 0)
        idx = 0;
    if (idx > 7)
        idx = 7;
    return idx;
}

static uint8_t v90_count_distinct_train_u(const v90_dil_desc_t *desc)
{
    bool seen[128];
    uint8_t count;
    int i;

    memset(seen, 0, sizeof(seen));
    count = 0;
    for (i = 0; i < desc->n; i++) {
        int ucode;

        ucode = desc->train_u[i] & 0x7F;
        if (!seen[ucode]) {
            seen[ucode] = true;
            count++;
        }
    }
    return count;
}

static void v90_dil_reset_tx(v90_state_t *s)
{
    s->dil_segment_index = 0;
    s->dil_pos_in_segment = 0;
    s->dil_terminate_requested = false;
}

/* Length in symbols of DIL-segment seg_idx (§8.4.1: Lc = (Hc + 1) * 6). */
static int v90_dil_segment_len(const v90_dil_desc_t *desc, int seg_idx)
{
    int training_ucode = desc->train_u[seg_idx] & 0x7F;
    int uchord_idx = v90_dil_uchord_index(training_ucode);

    return (int)(desc->h[uchord_idx] + 1) * 6;
}

/* Pure §8.4.1 DIL symbol: segment seg_idx, position pos within the segment. */
static uint8_t v90_dil_symbol_codeword(v90_law_t law,
                                       const v90_dil_desc_t *desc,
                                       int seg_idx,
                                       int pos)
{
    int training_ucode = desc->train_u[seg_idx] & 0x7F;
    int uchord_idx = v90_dil_uchord_index(training_ucode);
    int lsp = v90_clamp_positive(desc->lsp, V90_DIL_MAX_PAT_BITS);
    int ltp = v90_clamp_positive(desc->ltp, V90_DIL_MAX_PAT_BITS);
    int sp_bit = desc->sp[pos % lsp] ? 1 : 0;
    int tp_bit = desc->tp[pos % ltp] ? 1 : 0;
    int ucode = tp_bit ? training_ucode : (desc->ref[uchord_idx] & 0x7F);

    return v90_pcm_signed_codeword(law, ucode, sp_bit);
}

int v90_dil_cycle_len(const v90_dil_desc_t *desc)
{
    int total;
    int i;

    if (!desc || desc->n <= 0)
        return 0;
    total = 0;
    for (i = 0; i < desc->n; i++)
        total += v90_dil_segment_len(desc, i);
    return total;
}

int v90_dil_generate_codewords(v90_law_t law,
                               const v90_dil_desc_t *desc,
                               uint8_t *out,
                               int len)
{
    int seg_idx;
    int pos;
    int seg_len;
    int i;

    if (!desc || desc->n <= 0 || !out || len <= 0)
        return 0;

    seg_idx = 0;
    pos = 0;
    seg_len = v90_dil_segment_len(desc, 0);
    for (i = 0; i < len; i++) {
        out[i] = v90_dil_symbol_codeword(law, desc, seg_idx, pos);
        if (++pos >= seg_len) {
            pos = 0;
            seg_idx = (seg_idx + 1) % desc->n;
            seg_len = v90_dil_segment_len(desc, seg_idx);
        }
    }
    return len;
}

static uint8_t v90_dil_codeword(v90_state_t *s)
{
    int seg_idx;
    int n;
    int seg_len;
    uint8_t codeword;

    n = s->dil.n;
    if (n <= 0) {
        s->tx_phase = V90_TX_RI;
        s->sample_count = 0;
        s->phase4_hold_logged = false;
        return v90_pcm_idle(s->law);
    }

    seg_idx = s->dil_segment_index % n;
    seg_len = v90_dil_segment_len(&s->dil, seg_idx);
    codeword = v90_dil_symbol_codeword(s->law, &s->dil, seg_idx, s->dil_pos_in_segment);

    s->sample_count++;
    s->dil_pos_in_segment++;
    if (s->dil_pos_in_segment >= seg_len) {
        s->dil_pos_in_segment = 0;
        s->dil_segment_index++;
        if (s->dil_terminate_requested) {
            fprintf(stderr, "[V90] Phase 3: DIL termination requested, completed segment %d/%d and entering Phase 4\n",
                    seg_idx + 1, n);
            s->tx_phase = V90_TX_RI;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
        } else if ((s->dil_segment_index % n) == 0) {
            int cycles = s->dil_segment_index / n;
            int cycle_limit = v90_dil_autoterminate_cycles();

            if (cycle_limit > 0 && cycles >= cycle_limit) {
                fprintf(stderr,
                        "[V90] Phase 3: completed %d full DIL cycle%s; interop fallback entering Phase 4\n",
                        cycles, cycles == 1 ? "" : "s");
                s->tx_phase = V90_TX_RI;
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            } else {
                fprintf(stderr,
                        "[V90] Phase 3: completed one full DIL cycle (%d segments), repeating\n",
                        n);
            }
        } else {
            int cycle_limit = v90_dil_autoterminate_cycles();
            int early_symbols = v90_dil_autoterminate_early_symbols();
            int target_symbols = cycle_limit * v90_dil_cycle_len(&s->dil);

            if (cycle_limit > 0 && early_symbols > 0
                && s->sample_count < target_symbols
                && s->sample_count + early_symbols >= target_symbols) {
                fprintf(stderr,
                        "[V90] Phase 3: DIL interop fallback ending %d symbols "
                        "before %d-cycle target at segment %d/%d; entering Phase 4\n",
                        target_symbols - s->sample_count,
                        cycle_limit,
                        s->dil_segment_index,
                        n);
                s->tx_phase = V90_TX_RI;
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            }
        }
    }

    return codeword;
}

bool v90_parse_dil_descriptor(v90_dil_desc_t *out, const uint8_t *bits, int bit_len)
{
    int alpha;
    int beta;
    int training_start;
    int training_bits;
    int crc_start;
    int descriptor_bits;
    uint16_t expected_crc;
    uint16_t actual_crc;

    if (!out || !bits || bit_len < 206)
        return false;

    memset(out, 0, sizeof(*out));

    for (int i = 0; i < 17; i++) {
        if (v90_get_packed_bit(bits, i) == 0)
            return false;
    }
    if (!v90_expect_zero_bit(bits, bit_len, 17))
        return false;

    out->n = (uint8_t)v90_get_packed_bits(bits, 18, 8);
    if (!v90_expect_zero_range(bits, bit_len, 26, 8))
        return false;
    if (!v90_expect_zero_bit(bits, bit_len, 34))
        return false;

    out->lsp = (uint8_t)(v90_get_packed_bits(bits, 35, 7) + 1);
    if (!v90_expect_zero_bit(bits, bit_len, 42))
        return false;
    out->ltp = (uint8_t)(v90_get_packed_bits(bits, 43, 7) + 1);
    if (!v90_expect_zero_bit(bits, bit_len, 50))
        return false;

    if (out->n == 0) {
        if (out->lsp != 1 || out->ltp != 1)
            return false;
    }

    alpha = ((int)out->lsp + 15) / 16 * 17;
    beta = alpha + (((int)out->ltp + 15) / 16) * 17;
    training_start = 187 + beta;
    training_bits = (((int)out->n + 1) / 2) * 17;
    crc_start = training_start + training_bits;
    descriptor_bits = crc_start + 18;

    if (bit_len < descriptor_bits)
        return false;

    if (!v90_copy_framed_pattern(out->sp, out->lsp, bits, bit_len, 51))
        return false;
    if (!v90_copy_framed_pattern(out->tp, out->ltp, bits, bit_len, 51 + alpha))
        return false;
    if (!v90_parse_table12_byte_pairs(out->h, 8, bits, bit_len, 51 + beta))
        return false;
    if (!v90_parse_table12_byte_pairs(out->ref, 8, bits, bit_len, 119 + beta))
        return false;
    if (!v90_parse_table12_training_ucodes(out, bits, bit_len, training_start))
        return false;

    expected_crc = (uint16_t)v90_get_packed_bits(bits, crc_start + 1, 16);
    actual_crc = v90_dil_crc16_bits(bits, crc_start);
    if (expected_crc != actual_crc)
        return false;

    if (!v90_expect_zero_bit(bits, bit_len, crc_start + 17))
        return false;
    /*
     * A standalone V.90 descriptor may add a zero to make its length even,
     * while V.92 Table 20 extends the base descriptor at this point.  Do not
     * constrain or require a following bit here.
     */

    return true;
}

bool v90_dil_load_smartlink_adi_qc(v90_dil_desc_t *out)
{
    /* SLModem 2.9.11 setDilDescriptor(ADI_QC), captured before Ja packing.
     * The packed Table 12 descriptor is 1702 bits, LSB first.  Keeping the
     * peer's CRC-protected descriptor intact lets the strict parser validate
     * this fallback exactly as it validates a descriptor recovered from Ja. */
    static const uint8_t descriptor_bits[] = {
        0xff, 0xff, 0x41, 0x02, 0xb8, 0xbb, 0x13, 0x5f, 0x80, 0xc6, 0x97, 0x66,
        0x14, 0xb3, 0x3b, 0x8a, 0x09, 0x78, 0x77, 0xd4, 0x0a, 0xea, 0x04, 0x60,
        0xad, 0x80, 0x3f, 0x90, 0x52, 0x01, 0xdf, 0x1e, 0xd4, 0x4a, 0x8c, 0x35,
        0x65, 0xb5, 0xa2, 0x07, 0x30, 0x71, 0xe2, 0xe4, 0xc4, 0xc9, 0x89, 0x93,
        0x09, 0x4e, 0x4e, 0x9c, 0x9c, 0x38, 0x39, 0x71, 0xca, 0xf0, 0xe4, 0xa4,
        0x89, 0xc9, 0x92, 0x92, 0x24, 0x24, 0x47, 0x46, 0x8a, 0x88, 0x0c, 0x09,
        0x09, 0x02, 0xf2, 0xd3, 0x63, 0x27, 0xc7, 0x4d, 0x8d, 0x99, 0x18, 0x2f,
        0x2d, 0x56, 0x52, 0x9c, 0x94, 0x18, 0x09, 0xf1, 0xd1, 0x61, 0x23, 0xc3,
        0x45, 0x85, 0x89, 0x08, 0x0f, 0x0d, 0x16, 0x12, 0x1c, 0x14, 0x18, 0x10,
        0x40, 0x60, 0x00, 0x41, 0x01, 0x83, 0x03, 0x08, 0x09, 0x14, 0x16, 0x30,
        0x34, 0x70, 0x78, 0x00, 0x11, 0x41, 0x62, 0x02, 0x45, 0x05, 0x8b, 0x0b,
        0x18, 0x19, 0x34, 0x36, 0x70, 0x74, 0xf0, 0xf8, 0x00, 0x0a, 0x22, 0x34,
        0x84, 0xa8, 0x88, 0xd1, 0x11, 0xa4, 0x24, 0x4a, 0x4b, 0x98, 0x9a, 0x38,
        0x3d, 0x81, 0x8a, 0x22, 0x35, 0x85, 0xaa, 0x8a, 0xd5, 0x15, 0xac, 0x2c,
        0x5a, 0x5b, 0xb8, 0xba, 0x78, 0x7d, 0x01, 0x0b, 0x23, 0x36, 0x86, 0xac,
        0x8c, 0xd9, 0x19, 0xb4, 0x34, 0x6a, 0x6b, 0xd8, 0xda, 0xb8, 0xbd, 0x81,
        0x8b, 0x23, 0x37, 0x87, 0x8e, 0x8c, 0x16, 0x94, 0x27, 0x27, 0x4d, 0x4c,
        0x96, 0x94, 0x24, 0x21, 0x39, 0x32, 0x42, 0x2a, 0x0c,
    };

    return v90_parse_dil_descriptor(out, descriptor_bits, 1702);
}

bool v90_dil_load_smartlink_adi(v90_dil_desc_t *out)
{
    /* Alternate SLModem 2.9.11 setDilDescriptor(ADI) profile. */
    static const uint8_t descriptor_bits[] = {
        0xff, 0xff, 0x41, 0x02, 0xd8, 0xd9, 0xb1, 0x13, 0xe8, 0x72, 0x53, 0xe4,
        0x96, 0x95, 0x00, 0xd6, 0x8a, 0xf8, 0x03, 0x29, 0x15, 0xf0, 0x6b, 0x90,
        0x30, 0x61, 0x62, 0xc2, 0xc4, 0x84, 0x89, 0x04, 0x4e, 0x4e, 0x9c, 0x9c,
        0x38, 0x39, 0x71, 0xca, 0xf0, 0xe4, 0xa4, 0x89, 0xc9, 0x92, 0x92, 0x24,
        0x24, 0x47, 0x46, 0x8a, 0x88, 0x0c, 0x09, 0x09, 0x02, 0xf2, 0xd3, 0x63,
        0x27, 0xc7, 0x4d, 0x8d, 0x99, 0x18, 0x2f, 0x2d, 0x56, 0x52, 0x9c, 0x94,
        0x18, 0x09, 0xf1, 0xd1, 0x61, 0x23, 0xc3, 0x45, 0x85, 0x89, 0x08, 0x0f,
        0x0d, 0x16, 0x12, 0x1c, 0x14, 0x18, 0x10, 0x40, 0x60, 0x00, 0x41, 0x01,
        0x83, 0x03, 0x08, 0x09, 0x14, 0x16, 0x30, 0x34, 0x70, 0x78, 0x00, 0x11,
        0x41, 0x62, 0x02, 0x45, 0x05, 0x8b, 0x0b, 0x18, 0x19, 0x34, 0x36, 0x70,
        0x74, 0xf0, 0xf8, 0x00, 0x0a, 0x22, 0x34, 0x84, 0xa8, 0x88, 0xd1, 0x11,
        0xa4, 0x24, 0x4a, 0x4b, 0x98, 0x9a, 0x38, 0x3d, 0x81, 0x8a, 0x22, 0x35,
        0x85, 0xaa, 0x8a, 0xd5, 0x15, 0xac, 0x2c, 0x5a, 0x5b, 0xb8, 0xba, 0x78,
        0x7d, 0x01, 0x0b, 0x23, 0x36, 0x86, 0xac, 0x8c, 0xd9, 0x19, 0xb4, 0x34,
        0x6a, 0x6b, 0xd8, 0xda, 0xb8, 0xbd, 0x81, 0x8b, 0x23, 0x37, 0x87, 0x8e,
        0x8c, 0x16, 0x94, 0x27, 0x27, 0x4d, 0x4c, 0x96, 0x94, 0x24, 0x21, 0x39,
        0x32, 0xc2, 0x14, 0x03,
    };

    return v90_parse_dil_descriptor(out, descriptor_bits, 1566);
}

int v90_dil_descriptor_bit_len(const v90_dil_desc_t *desc)
{
    int alpha;
    int beta;
    int training_bits;

    if (!desc)
        return 0;
    if (desc->n == 0 && (desc->lsp != 1 || desc->ltp != 1))
        return 0;
    if (desc->lsp > V90_DIL_MAX_PAT_BITS || desc->ltp > V90_DIL_MAX_PAT_BITS)
        return 0;
    if (desc->lsp < 1 || desc->ltp < 1)
        return 0;

    alpha = ((int) desc->lsp + 15) / 16 * 17;
    beta = ((int) desc->ltp + 15) / 16 * 17;
    training_bits = (((int) desc->n + 1) / 2) * 17;
    {
        int bit_len = 187 + alpha + beta + training_bits + 18;

        if (bit_len & 1)
            bit_len++;
        return bit_len;
    }
}

bool v90_build_dil_descriptor_bits(uint8_t *buf,
                                   int buf_len,
                                   int *bit_len_out,
                                   const v90_dil_desc_t *desc)
{
    int bit_len;
    int bit_pos;
    int crc_start;
    uint16_t crc;

    if (!buf || !desc || buf_len <= 0)
        return false;

    bit_len = v90_dil_descriptor_bit_len(desc);
    if (bit_len <= 0 || buf_len < ((bit_len + 7) / 8))
        return false;

    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, 0x1FFFFU, 17);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, desc->n, 8);
    v90_put_zero_range(buf, &bit_pos, 8);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, (uint32_t) (desc->lsp - 1), 7);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, (uint32_t) (desc->ltp - 1), 7);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_put_framed_pattern(buf, &bit_pos, desc->sp, desc->lsp);
    v90_put_framed_pattern(buf, &bit_pos, desc->tp, desc->ltp);
    v90_put_framed_byte_pairs(buf, &bit_pos, desc->h, 8);
    v90_put_framed_byte_pairs(buf, &bit_pos, desc->ref, 8);
    v90_put_framed_training_ucodes(buf, &bit_pos, desc);
    crc_start = bit_pos;
    v90_bits_put(buf, &bit_pos, 0, 1);
    crc = v90_dil_crc16_bits(buf, crc_start);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0, 1);
    if (bit_pos < bit_len)
        v90_put_zero_range(buf, &bit_pos, bit_len - bit_pos);

    if (bit_pos != bit_len) {
        return false;
    }
    if (bit_len_out)
        *bit_len_out = bit_len;
    return true;
}

bool v90_analyse_dil_descriptor(const v90_dil_desc_t *desc, v90_dil_analysis_t *analysis_out)
{
    v90_dil_analysis_t analysis;
    bool seen_uchord[8];
    int i;

    if (!desc || !analysis_out)
        return false;

    memset(&analysis, 0, sizeof(analysis));
    memset(seen_uchord, 0, sizeof(seen_uchord));
    analysis.n = desc->n;
    analysis.lsp = desc->lsp;
    analysis.ltp = desc->ltp;
    analysis.unique_train_u = v90_count_distinct_train_u(desc);

    for (i = 0; i < 8; i++) {
        if (desc->ref[i] != 0)
            analysis.non_default_refs++;
        if (desc->h[i] != 1)
            analysis.non_default_h++;
    }
    for (i = 0; i < desc->n; i++) {
        int uchord_idx;

        uchord_idx = v90_dil_uchord_index(desc->train_u[i] & 0x7F);
        seen_uchord[uchord_idx] = true;
    }
    for (i = 0; i < 8; i++) {
        if (seen_uchord[i])
            analysis.used_uchords++;
    }

    analysis.looks_default_125x12 = (desc->n == 125
                                     && desc->lsp == 12
                                     && desc->ltp == 12
                                     && analysis.used_uchords >= 6
                                     && analysis.non_default_h == 0);
    analysis.robbed_bit_limited = (desc->n == 125
                                   && desc->lsp == 12
                                   && desc->ltp == 6
                                   && analysis.used_uchords >= 6
                                   && analysis.non_default_h == 0);

    if (desc->n < 125)
        analysis.impairment_score++;
    if (desc->n < 100)
        analysis.impairment_score++;
    if (desc->lsp != 12 || desc->ltp != 12)
        analysis.impairment_score++;
    if (desc->lsp < 12 || desc->ltp < 12)
        analysis.impairment_score++;
    if (analysis.used_uchords < 6)
        analysis.impairment_score++;
    if (analysis.used_uchords < 3)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 32)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 12)
        analysis.impairment_score++;
    if (analysis.non_default_h != 0)
        analysis.impairment_score++;

    analysis.echo_limited = (!analysis.robbed_bit_limited && analysis.impairment_score >= 3);
    if (analysis.robbed_bit_limited) {
        analysis.recommended_downstream_drn = 22;
        analysis.recommended_upstream_drn = 22;
    } else if (analysis.impairment_score >= 5) {
        analysis.recommended_downstream_drn = 13;
        analysis.recommended_upstream_drn = 7;
    } else if (analysis.echo_limited) {
        analysis.recommended_downstream_drn = 16;
        analysis.recommended_upstream_drn = 10;
    } else {
        analysis.recommended_downstream_drn = 19;
        analysis.recommended_upstream_drn = 16;
    }

    *analysis_out = analysis;
    return true;
}

/* Generate one raw G.711 codeword for the Phase 3/4 transmit sequence. */
static uint8_t v90_phase3_codeword(v90_state_t *s)
{
    int sign;

    /* §9.6.1: the digital modem shall initiate a retrain if it does not
     * receive an E sequence within 5000 ms plus two round-trip delays after
     * transmitting the Rd→R̄d transition.  Run the clock here, where every
     * transmitted symbol passes, and let the engine own the retrain itself. */
    if (s->reneg_active) {
        s->reneg_symbol_clock++;
        if (!s->reneg_timed_out
            && s->reneg_rbar_symbol >= 0
            && !s->e_received
            && s->reneg_symbol_clock - s->reneg_rbar_symbol
                   > V90_RENEG_E_TIMEOUT_SYMBOLS) {
            s->reneg_timed_out = true;
            fprintf(stderr,
                    "[V90] Rate renegotiation %d: no E within %d symbols of "
                    "the Rd->R-bar-d transition; a retrain is owed "
                    "(§9.6.1/§9.5.1.1)\n",
                    s->reneg_count, V90_RENEG_E_TIMEOUT_SYMBOLS);
        }
        /* §9.6.1's own timeout runs from the Rd→R̄d transition, and that
         * transition needs the analogue modem's CPt.  A peer that does not
         * answer Rd at all therefore never starts that clock, and the
         * transmitter holds Rd for the rest of the call -- measured against
         * the SmartLink rig, which then declares SILENCERETRAIN and retrains,
         * costing everything after the loss that provoked the renegotiation.
         * §9.6.1 also says the digital modem "may initiate a retrain at any
         * time during a rate renegotiation according to 9.5.1.1", so bound
         * the wait for an answer as well as the wait for E. */
        if (!s->reneg_timed_out
            && s->reneg_rbar_symbol < 0
            && s->reneg_symbol_clock > V90_RENEG_ANSWER_TIMEOUT_SYMBOLS) {
            s->reneg_timed_out = true;
            fprintf(stderr,
                    "[V90] Rate renegotiation %d: no CP within %d symbols of "
                    "Rd; the peer is not answering, a retrain is owed "
                    "(§9.6.1/§9.5.1.1)\n",
                    s->reneg_count, V90_RENEG_ANSWER_TIMEOUT_SYMBOLS);
        }
    }

    switch (s->tx_phase) {
    case V90_TX_WAIT_JA: {
        int wait_limit = s->jd_resync_wait
                       ? v90_wait_ja_resync_samples()
                       : V90_WAIT_JA_FALLBACK_SAMPLES;

        if (!s->v92_phase3 && ++s->sample_count >= wait_limit) {
            fprintf(stderr,
                    "[V90] Phase 3: %s after %.1f ms, starting Sd via interop fallback\n",
                    s->jd_resync_wait ? "Jd resync silence elapsed"
                                      : "Ja decode timeout",
                    1000.0 * s->sample_count / 8000.0);
            s->jd_resync_wait = false;
            s->tx_phase = V90_TX_SD;
            s->sample_count = 0;
            s->rep_count = 0;
        }
        return v90_pcm_idle(s->law);
    }

    case V90_TX_SD_DELAY:
        /* §9.3.1.3: "After receiving Ja, the digital modem may wait for up
         * to 500ms and then transmit signal Sd." Measured live against the
         * d-modem/slmodemd rig: with immediate (0ms) Sd, we were sending
         * Sd/S-bar-d (54ms total) ~763ms before SmartLink's own
         * Phase3Demodulator armed its WaitForSd receiver (its own JaTXMIT
         * start + arm is not instantaneous either) -- so it never saw any
         * of it and retrained after its own ~3.1s local timeout, every
         * time. Default 0 preserves prior (immediate) behaviour for
         * everything else; env-tunable per peer. */
        if (++s->sample_count >= v90_sd_delay_samples(s)) {
            s->tx_phase = V90_TX_SD;
            s->sample_count = 0;
            s->rep_count = 0;
        }
        return v90_pcm_idle(s->law);

    case V90_TX_SD:
        /* §8.4.4: Sd = 64 reps of {+W, +0, +W, -W, -0, -W}
         * W = Ucode(16 + U_INFO), 0 = Ucode 0
         * §9.3.1.3: Sent first after receiving analog modem's Ja */
        {
            int w_ucode = 16 + s->u_info;
            int pos_in_pattern = s->sample_count % 6;
            s->sample_count++;

            switch (pos_in_pattern) {
            case 0: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 1: return v90_pcm_signed_codeword(s->law, 0, 1);       /* +0 */
            case 2: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 3: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 4: return v90_pcm_signed_codeword(s->law, 0, 0);       /* -0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_REPS) {
                    fprintf(stderr, "[V90] Phase 3: Sd complete (%d reps), starting S̄d\n",
                            s->rep_count);
                    s->tx_phase = V90_TX_SD_BAR;
                    s->sample_count = 0;
                    s->rep_count = 0;
                }
                return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            }
            break;  /* unreachable */
        }

    case V90_TX_SD_BAR:
        /* §8.4.4: S̄d = 8 reps of {-W, -0, -W, +W, +0, +W} */
        {
            int w_ucode = 16 + s->u_info;
            int pos_in_pattern = s->sample_count % 6;
            s->sample_count++;

            switch (pos_in_pattern) {
            case 0: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 1: return v90_pcm_signed_codeword(s->law, 0, 0);       /* -0 */
            case 2: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 3: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 4: return v90_pcm_signed_codeword(s->law, 0, 1);       /* +0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_BAR_REPS) {
                    fprintf(stderr, "[V90] Phase 3: S̄d complete, starting TRN1d\n");
                    s->tx_phase = V90_TX_TRN1D;
                    s->sample_count = 0;
                    /* §8.4.5: scrambler initialized to zero for TRN1d */
                    v90_scrambler_init(&s->scrambler);
                }
                return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            }
            break;  /* unreachable */
        }

    case V90_TX_TRN1D:
        /* §8.4.5: TRN1d = U_INFO codeword with signs from scrambled ones.
         * Scrambler initialized to zero.
         * §9.3.1.4: ≥2040T, then Jd within 4000ms of starting TRN1d */
        {
            int scrambled = v90_scramble_bit(&s->scrambler, 1);
            int shape_amp = v90_trn1d_shape_amplitude();
            int phase = s->sample_count % 6;   /* DS0 phase, pre-increment */
            int ucode = s->u_info;
            sign = scrambled;  /* sign=0 → negative, sign=1 → positive */
            if (shape_amp > 0) {
                /* Dither the magnitude per DS0 phase so the peer's per-phase
                 * variance array has non-zero spread and its trn1Sigma stays
                 * finite (see v90_trn1d_shape_amplitude).  Symmetric, so the
                 * mean codeword remains U_INFO. */
                ucode += v90_trn1d_phase_dither(phase, shape_amp);
                if (ucode < 1)   ucode = 1;
                if (ucode > 127) ucode = 127;
                if (s->sample_count == 0)
                    fprintf(stderr, "[V90] Phase 3: TRN1d per-phase dither "
                            "active (peak radius=%d, per-phase radii "
                            "3 2 1 2 3 1 scaled)\n", shape_amp);
            }
            s->sample_count++;
            if (s->sample_count >= v90_trn1d_len()) {
                fprintf(stderr, "[V90] Phase 3: TRN1d complete (%d symbols), starting Jd\n",
                        s->sample_count);
                s->tx_phase = V90_TX_JD;
                s->sample_count = 0;
                /* §8.4.2: differential encoder initialized with final symbol of TRN1d */
                s->diff_enc = sign;
                s->jd_bit_pos = 0;
                /* Scrambler continues from TRN1d into Jd (not reinitialized) */
            }
            return v90_pcm_signed_codeword(s->law, ucode, sign);
        }

    case V90_TX_JD:
        /* §8.4.2: Jd bits are scrambled and differentially encoded,
         * transmitted as sign of U_INFO PCM codeword.
         * §9.3.1.4/§9.3.1.5: Sent after TRN1d and repeated until S is seen.
         * Once S is detected, complete the current Jd repetition and send J'd. */
        {
            int bit = v90_get_jd_bit(s);
            int scrambled = v90_scramble_bit(&s->scrambler, bit);
            /* Differential encoding: sign = scrambled XOR previous sign */
            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            /* Jd runs until the analogue modem answers our Sd->S-bar
             * transition with S (§9.3.1.5).  Both expiries below are the same
             * §9.3.1.5 deadline by default, so check the interop fallback
             * first: pushing on to J'd/DIL is strictly more useful than tearing
             * Phase 3 down, and if the fallback is disabled the resync still
             * fires on the same symbol. */
            if (!s->v92_phase3
                && !s->jd_terminate_requested
                && v90_jd_autoterminate_symbols() > 0
                && s->sample_count >= v90_jd_autoterminate_symbols()) {
                fprintf(stderr,
                        "[V90] Phase 3: Jd interop timeout after %d symbols, terminating at the next frame boundary\n",
                        s->sample_count);
                s->jd_terminate_requested = true;
            }
            /* Interop resync: with no S and no fallback, the peer did not lock
             * our Sd and will retrain.  Continuing to transmit Jd (and then
             * Phase 4) here pours a modem-like signal over the peer's fresh
             * Phase 1/2, corrupting its bulk-delay/RTD estimate so its next Sd
             * search window is mis-placed.  Fall back to the silent WAIT_JA
             * state, re-arming the Ja detector so we re-emit Sd cleanly when
             * the peer next reaches Ja. */
            if (!s->v92_phase3
                && !s->jd_terminate_requested
                && v90_jd_resync_symbols() > 0
                && s->sample_count >= v90_jd_resync_symbols()) {
                fprintf(stderr,
                        "[V90] Phase 3: no S after %d Jd symbols; resyncing to WAIT_JA (peer likely retrained)\n",
                        s->sample_count);
                s->tx_phase = V90_TX_WAIT_JA;
                s->jd_resync_wait = true;
                s->sample_count = 0;
                s->rep_count = 0;
                s->jd_bit_pos = 0;
                s->jd_terminate_requested = false;
                s->jd_terminated_by_s = false;
                s->jd_terminated_by_su = false;
                return v90_pcm_idle(s->law);
            }
            if (s->jd_terminate_requested && s->jd_bit_pos == 0) {
                /* Say which one actually terminated Jd.  This used to print
                 * "S detected" unconditionally, including when the interop
                 * timeout above set jd_terminate_requested with no S ever
                 * arriving -- which reads in the log as real progress and
                 * cost a full investigation before the timeout was spotted
                 * on the preceding line. */
                if (s->v92_phase3) {
                    fprintf(stderr,
                            "[V92] Phase 3: Su sequence complete, finished Jd after %d symbols; starting Jp\n",
                            s->sample_count);
                    s->tx_phase = V90_TX_JP;
                    s->jp_bit_pos = 0;
                } else {
                    fprintf(stderr,
                            "[V90] Phase 3: %s, completed current Jd repetition after %d symbols, starting J'd\n",
                            s->jd_terminated_by_s ? "S detected"
                                                  : "NO S RECEIVED (terminated by interop timeout)",
                            s->sample_count);
                    s->tx_phase = V90_TX_JD_PRIME;
                }
                s->sample_count = 0;
            }
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
        }

    case V90_TX_JD_PRIME:
        /* §8.4.3: J'd = 12 scrambled zeros as sign of U_INFO */
        {
            int jd_prime_symbols = v90_jd_prime_symbols();
            int scrambled = v90_scramble_bit(&s->scrambler, 0);
            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            if (s->sample_count >= jd_prime_symbols) {
                if (jd_prime_symbols != 12) {
                    fprintf(stderr,
                            "[V90] Phase 3: diagnostic extended J'd complete (%d symbols)\n",
                            jd_prime_symbols);
                }
                if (s->dil_requested) {
                    fprintf(stderr, "[V90] Phase 3: J'd complete, entering DIL placeholder state\n");
                    s->tx_phase = V90_TX_DIL;
                } else {
                    fprintf(stderr, "[V90] Phase 3: J'd complete, entering Phase 4\n");
                    s->tx_phase = V90_TX_RI;
                }
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            }
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
        }

    case V90_TX_JP:
        /* V.92 §8.6.3/Table 22: repeat Jp until the analogue modem has
         * switched from Su to S-bar-u after recognising Jp. */
        {
            int bit = v90_get_jp_bit(s);
            int scrambled = v90_scramble_bit(&s->scrambler, bit);

            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            if (s->jp_terminate_requested && s->jp_bit_pos == 0) {
                fprintf(stderr,
                        "[V92] Phase 3: Su final transition, completed Jp after %d symbols; starting Jp'\n",
                        s->sample_count);
                s->tx_phase = V90_TX_JP_PRIME;
                s->sample_count = 0;
            }
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
        }

    case V90_TX_JP_PRIME:
        /* V.92 §8.6.4: Jp' is exactly twelve scrambled, differentially
         * encoded zeroes. */
        {
            int scrambled = v90_scramble_bit(&s->scrambler, 0);

            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            if (s->sample_count >= 12) {
                s->tx_phase = s->dil_requested ? V90_TX_DIL : V90_TX_SCR;
                s->sample_count = 0;
                s->phase4_hold_logged = false;
                fprintf(stderr,
                        "[V92] Phase 3: Jp' complete, entering %s while waiting for CPt\n",
                        s->dil_requested ? "DIL" : "SCR");
            }
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
        }

    case V90_TX_DIL:
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 3: sending DIL (%d segments, LSP=%u, LTP=%u)\n",
                    s->dil.n, s->dil.lsp, s->dil.ltp);
            s->phase4_hold_logged = true;
        }
        return v90_dil_codeword(s);

    case V90_TX_SCR:
        /* V.92 §8.6.6: zero-DIL calls send the continuing scrambled-one
         * source while the upstream receiver acquires CPt. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V92] Phase 3: sending SCR while waiting for CPt\n");
            s->phase4_hold_logged = true;
        }
        sign = v90_scramble_bit(&s->scrambler, 1);
        return v90_pcm_signed_codeword(s->law, s->u_info, sign);

    case V90_TX_RI:
        /* §8.6.4/§9.4.1.1: Ri is U_INFO with +++--- signs, not idle.
         * Send at least 192T before allowing the analogue modem's CPt.  Do
         * not change to the TRN2d state on that timer: CPt must be accepted
         * first, after which the transmitter emits the barred R-i ACK.
         * §9.6.1.1: on a rate renegotiation the same signal is Rd and runs
         * for 384T. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] %s: %s (%d symbols)\n",
                    s->reneg_active ? "Rate renegotiation" : "Phase 4",
                    s->reneg_active ? "Rd" : "Ri",
                    v90_ri_length(s));
            s->phase4_hold_logged = true;
        }
        {
            uint8_t codeword = v90_ri_codeword(s, s->sample_count, false);

            s->sample_count++;
            return codeword;
        }

    case V90_TX_RI_ACK:
    case V90_TX_TRN2D:
        /* §9.4.1.1/§9.4.1.2: remain in Ri while acquiring CPt.  After
         * accepting CPt, send 24T of barred Ri followed by at least 2040T of
         * TRN2d using the negotiated six-interval modulus mapper. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: waiting for valid CPt%s\n",
                    s->v92_mode ? " (V.92 compatibility path)" : "");
            s->phase4_hold_logged = true;
        }
        if (!s->cp_ready) {
            if (s->sample_count == 0) {
                fprintf(stderr,
                        "[V90] Phase 4: holding unbarred Ri while waiting for CPt\n");
            }
            uint8_t codeword = v90_ri_codeword(s, s->sample_count, false);

            s->sample_count++;
            return codeword;
        }

        if (s->v92_mode && !s->v92_native_cpu_rx) {
            /* Compatibility path: no negotiated Phase 4 mapper; jump
             * straight to sign-modulated SUVd at U_INFO. */
            v90_build_suvd(s->suv_bits, false);
            s->suv_bit_pos = 0;
            s->tx_phase = V90_TX_SUVD;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return v90_pcm_idle(s->law);
        }

        /* CPt reception is asynchronous to our six-symbol Ri period.  Finish
         * the current Ri period before beginning barred Ri; resetting the
         * sign phase at the receive callback produces a partial period that
         * an analogue modem cannot recognize as the Ri-to-barred-Ri event. */
        if (s->phase4_ri_align_remaining > 0) {
            int ri_pos = V90_FRAME_LEN - s->phase4_ri_align_remaining;
            uint8_t codeword = v90_ri_codeword(s, ri_pos, false);

            s->phase4_ri_align_remaining--;
            return codeword;
        }

        if (s->sample_count < V90_RI_POST_CP_SYMBOLS) {
            /* Ri is defined by U_INFO and the negotiated bearer law, not by
             * the CPt constellation law.  Its barred form must retain the
             * exact magnitude of the preceding unbarred Ri. */
            uint8_t codeword = v90_ri_codeword(s, s->sample_count, true);

            if (s->sample_count == 0 && s->reneg_active
                && s->reneg_rbar_symbol < 0) {
                /* §9.6.1's timeout is measured from the Rd→R̄d transition,
                   not from the start of the renegotiation. */
                s->reneg_rbar_symbol = s->reneg_symbol_clock;
            }
            s->sample_count++;
            if (s->sample_count == V90_RI_POST_CP_SYMBOLS) {
                fprintf(stderr,
                        "[V90] Phase 4: CPt accepted; TRN2d (%d mapped symbols, D=%d, K=%d)\n",
                        v90_trn2d_symbols(), s->phase4_d, s->phase4_k);
                /* The 24th barred R-i symbol completes the CPt
                 * acknowledgement (§9.4.1.1).  Expose TRN2d immediately
                 * after returning that symbol so the next codeword is the
                 * first mapped TRN2d symbol (§9.4.1.2). */
                if (s->tx_phase == V90_TX_RI_ACK)
                    s->tx_phase = V90_TX_TRN2D;
            }
            return codeword;
        }
        {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ONES);

            s->sample_count++;
            if (s->sample_count >= V90_RI_POST_CP_SYMBOLS + v90_trn2d_symbols()) {
                if (s->v92_mode) {
                    /* §9.6.1.1.1/V.92: TRN2d done; condition for SUVu and
                     * transmit SUVd sequences over the TRN2d mapper. */
                    (void)v90_build_v92_suvd_mapped(s, false);
                    s->tx_phase = V90_TX_SUVD;
                } else {
                    s->tx_phase = V90_TX_MP;
                }
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            }
            return codeword;
        }

    case V90_TX_MP:
        /* §9.4.1.3: repeat MP with acknowledge=0 until data-mode CP is
         * received, then repeat MP' until the analogue modem returns CP'. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: MP%s (%d bits, D=%d)\n",
                    s->mp_acknowledge ? "'" : "",
                    s->mp_nbits, s->phase4_d);
            s->phase4_hold_logged = true;
        }
        {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_MP);

            if (s->mp_bit_pos >= s->mp_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                s->phase4_hold_logged = false;
                if (!s->mp_acknowledge) {
                    if (s->data_cp_received)
                        (void)v90_build_mp_type0(s, true);
                    else
                        (void)v90_build_mp_type0(s, false);
                } else if ((s->cp_ack_received || s->e_received)
                           && s->data_mapper_ready) {
                    s->tx_phase = V90_TX_ED;
                    s->sample_count = 0;
                } else {
                    (void)v90_build_mp_type0(s, true);
                }
            }
            return codeword;
        }

    case V90_TX_SUVD:
        /* V.92 §9.6.1.1: SUVd — Short Update Values (digital→analogue).
         * Native mode transmits over the CPt-negotiated TRN2d mapper;
         * the compatibility path sign-modulates at U_INFO. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4 V.92: SUVd (%s, ack=%d)\n",
                    s->v92_native_cpu_rx ? "TRN2d mapped" : "U_INFO",
                    (s->v92_native_cpu_rx && s->v92_cpu_received) ? 1 : 0);
            s->phase4_hold_logged = true;
        }
        if (s->v92_native_cpu_rx) {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_V92);

            if (s->v92_tx_pos >= s->v92_tx_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                s->phase4_hold_logged = false;
                /* §9.6.1.1.4: after sending an acknowledged sequence and
                 * receiving CPu'/SUVu' (or E2u), move to Ed. */
                if (s->v92_ack_sent && s->v92_remote_ack_received) {
                    s->tx_phase = V90_TX_ED;
                    s->sample_count = 0;
                } else if (!s->v92_cpd_sent
                           && (s->v92_suvu_received || s->v92_cpu_received)) {
                    /* §9.6.1.1.2: a single CPd per received SUVu/CPu. */
                    if (v90_build_v92_cpd_native(s)) {
                        s->tx_phase = V90_TX_CP;
                        s->sample_count = 0;
                    } else {
                        (void)v90_build_v92_suvd_mapped(s,
                                                        s->v92_cpu_received);
                    }
                } else {
                    /* Otherwise repeat SUVd; ack tracks CPu receipt. */
                    (void)v90_build_v92_suvd_mapped(s, s->v92_cpu_received);
                }
            }
            return codeword;
        }
        if (s->suv_bit_pos >= V92_SUVD_BITS) {
            /* Compatibility path: SUVd complete → CPd; diff_enc continues */
            s->tx_phase = V90_TX_CP;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            s->cp_bit_pos = 0;
            return v90_pcm_idle(s->law);
        }
        {
            int suv_bit = s->suv_bits[s->suv_bit_pos++] & 1;
            suv_bit = v90_scramble_bit(&s->scrambler, suv_bit);
            s->diff_enc ^= suv_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_CP:
        /* Legacy V.92 harness payload. This is a V.90-shaped CP frame, not a
         * native Table 30 CPd; native CPd remains gated on real CPu-derived
         * rate, gain, filter, and constellation parameters. */
        if (!s->phase4_hold_logged) {
            if (s->v92_native_cpu_rx)
                fprintf(stderr,
                        "[V90] Phase 4 V.92: native Table 30 CPd (%d bits, TRN2d mapped, ack=%d)\n",
                        s->v92_tx_nbits, s->v92_cpu_received ? 1 : 0);
            else
                fprintf(stderr,
                        "[V90] Phase 4 V.92 compatibility: legacy CP payload (%d bits; not native CPd)\n",
                        s->cp_nbits);
            s->phase4_hold_logged = true;
        }
        if (s->v92_native_cpu_rx) {
            /* Native Table 30 CPd over the TRN2d mapper. */
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_V92);

            if (s->v92_tx_pos >= s->v92_tx_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                /* §9.6.1.1.2: single CPd, then more SUVd sequences whose
                 * ack bit is gated on a real CPu having been received. */
                s->v92_cpd_sent = true;
                s->phase4_hold_logged = false;
                s->sample_count = 0;
                (void)v90_build_v92_suvd_mapped(s, s->v92_cpu_received);
                s->tx_phase = V90_TX_SUVD;
            }
            return codeword;
        }
        if (s->cp_bit_pos >= s->cp_nbits) {
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            /* Compatibility: CPd → SUVd' (with ack bit set). */
            v90_build_suvd(s->suv_bits, true);
            s->suv_bit_pos = 0;
            s->tx_phase = V90_TX_SUVD_ACK;
            return v90_pcm_idle(s->law);
        }
        {
            int cp_bit = s->cp_bits[s->cp_bit_pos++] & 1;
            cp_bit = v90_scramble_bit(&s->scrambler, cp_bit);
            s->diff_enc ^= cp_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_SUVD_ACK:
        /* V.92 §9.6.1.1: SUVd' — Short Update Values with acknowledge bit set.
         * Signals to analogue that digital has received CPu. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4 V.92: SUVd' (%d bits, ack=1)\n", V92_SUVD_BITS);
            s->phase4_hold_logged = true;
        }
        if (s->suv_bit_pos >= V92_SUVD_BITS) {
            /* SUVd' complete → Ed */
            s->tx_phase = V90_TX_ED;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return v90_pcm_idle(s->law);
        }
        {
            int suv_bit = s->suv_bits[s->suv_bit_pos++] & 1;
            suv_bit = v90_scramble_bit(&s->scrambler, suv_bit);
            s->diff_enc ^= suv_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_ED:
        /* Ed is two downstream mapping frames of scrambled binary zeros. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4%s: Ed (%d symbols, scrambled zeros)\n",
                    s->v92_mode ? " V.92" : "", V90_ED_SYMBOLS);
            s->phase4_hold_logged = true;
        }
        {
            uint8_t codeword;
            int tx_symbols = V90_ED_SYMBOLS
                           + v90_shaper_delay_frames(&s->cp_frame)
                           * V90_FRAME_LEN;

            if (s->v92_mode && s->v92_native_cpu_rx) {
                /* §8.8.2/V.92: Ed uses the corresponding TRN2d modulation. */
                codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ZEROS);
            } else if (s->v92_mode) {
                int zero_bit = v90_scramble_bit(&s->scrambler, 0);

                tx_symbols = V90_ED_SYMBOLS;
                s->diff_enc ^= zero_bit;
                codeword = v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
            } else {
                /* Lookahead keeps ceil(ld/Sr) final MP frames pending. Feed
                 * that many additional zero frames so the wire sees every MP
                 * frame followed by both required Ed frames before switching
                 * to the independent B1d mapper. */
                codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ZEROS);
            }
            s->sample_count++;
            if (s->sample_count >= tx_symbols) {
                s->tx_phase = V90_TX_B1D;
                s->sample_count = 0;
                s->phase4_hold_logged = false;
                if (!s->v92_mode
                    || (s->v92_native_cpu_rx && s->data_mapper_ready))
                    v90_reset_negotiated_data_mapper(s);
            }
            return codeword;
        }

    case V90_TX_B1D:
        /* §8.6.1/§9.4.1.5: 48 complete data frames of scrambled ones,
         * starting from zeroed data-mode scrambler and differential state. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: B1d (%d data frames, %d symbols)\n",
                    V90_B1D_FRAMES, V90_B1D_SYMBOLS);
            s->phase4_hold_logged = true;
        }
        /* §9.6.1.1.5/V.92: in native mode B1d runs at the negotiated rate
         * using the data-mode constellation parameters received in CPu. */
        if (s->data_mapper_ready
            && (!s->v92_mode || s->v92_native_cpu_rx)) {
            uint8_t codeword = v90_data_mapper_ones_codeword(s);

            s->sample_count++;
            if (s->sample_count >= V90_B1D_SYMBOLS) {
                s->tx_phase = V90_TX_DATA;
                s->sample_count = 0;
                s->training_complete = true;
                if (s->reneg_active) {
                    fprintf(stderr,
                            "[V90] Rate renegotiation %d complete; data mode "
                            "resumed after B1d\n", s->reneg_count);
                    s->reneg_active = false;
                    s->reneg_rbar_symbol = -1;
                }
            }
            return codeword;
        }
        /* V.92 compatibility path retains the legacy marker until its
         * separate data-mode mapper is implemented. */
        s->sample_count++;
        if (s->sample_count >= V90_B1D_SYMBOLS) {
            s->tx_phase = V90_TX_DATA;
            s->sample_count = 0;
            v90_reset_data_pump_state(s);
            s->training_complete = true;
        }
        return v90_pcm_idle(s->law);

    default:
        break;
    }

    return v90_pcm_idle(s->law);
}

/* ---- Public API ---- */

v90_state_t *v90_init_with_v34(v34_state_t *v34, v90_law_t law)
{
    v90_state_t *s = (v90_state_t *)calloc(1, sizeof(*s));
    if (!s)
        return NULL;

    s->v34 = v34;
    s->law = law;
    s->sd_delay_override_samples = -1;
    s->tx_phase = V90_TX_PHASE2;
    s->u_info = 80;
    s->owns_v34 = false;
    v90_scrambler_init(&s->scrambler);
    s->diff_enc = 0;
    v90_reset_data_pump_state(s);
    s->phase4_hold_logged = false;
    s->phase4_ri_align_remaining = 0;
    s->jd_terminate_requested = false;
    s->jd_terminated_by_s = false;
    s->training_complete = false;
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    memset(&s->dil, 0, sizeof(s->dil));
    v90_dil_reset_tx(s);

    return s;
}

v90_state_t *v90_init_data_pump(v90_law_t law)
{
    return v90_init_with_v34(NULL, law);
}

v90_state_t *v90_init(int baud_rate,
                      int bit_rate,
                      bool calling_party,
                      v90_law_t law,
                      span_get_bit_func_t get_bit,
                      void *get_bit_user_data,
                      span_put_bit_func_t put_bit,
                      void *put_bit_user_data)
{
    v90_state_t *s = (v90_state_t *)calloc(1, sizeof(*s));
    if (!s)
        return NULL;

    s->law = law;
    s->tx_phase = V90_TX_PHASE2;
    s->u_info = 80;  /* Default U_INFO (safe mid-range value) */
    v90_scrambler_init(&s->scrambler);
    s->diff_enc = 0;
    v90_reset_data_pump_state(s);
    s->jd_terminate_requested = false;
    s->jd_terminated_by_s = false;
    s->training_complete = false;
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    memset(&s->dil, 0, sizeof(s->dil));
    v90_dil_reset_tx(s);

    s->owns_v34 = true;
    s->v34 = v34_init(NULL, baud_rate, bit_rate, calling_party, true,
                       get_bit, get_bit_user_data,
                       put_bit, put_bit_user_data);
    if (!s->v34) {
        free(s);
        return NULL;
    }

    /* Enable V.90 INFO0d frame generation */
    v34_set_v90_mode(s->v34, (law == V90_LAW_ALAW) ? 1 : 0);

    return s;
}

void v90_free(v90_state_t *s)
{
    if (!s)
        return;
    if (s->v34 && s->owns_v34)
        v34_free(s->v34);
    free(s);
}

v34_state_t *v90_get_v34(v90_state_t *s)
{
    return s->v34;
}

v90_tx_phase_t v90_get_tx_phase(v90_state_t *s)
{
    return s->tx_phase;
}

bool v90_phase3_active(v90_state_t *s)
{
    return s->tx_phase >= V90_TX_WAIT_JA && s->tx_phase <= V90_TX_SCR;
}

void v90_start_phase3(v90_state_t *s, int u_info)
{
    if (u_info > 0 && u_info < 128)
        s->u_info = u_info;

    fprintf(stderr, "[V90] Starting Phase 3 TX (U_INFO=%d, law=%s)\n",
            s->u_info, s->law == V90_LAW_ALAW ? "A-law" : "u-law");

    /* Build Jd frame (used later after TRN1d) */
    v90_build_jd(s);
    if (s->v92_phase3)
        v90_build_jp(s);

    /* V.90 §9.3.1.3: After receiving analog Ja, send Sd first.
     * Sd does not use scrambler or differential encoder.
     * Scrambler is initialized to zero for TRN1d (done at Sd→S̄d→TRN1d transition). */
    s->sample_count = 0;
    s->rep_count = 0;
    s->diff_enc = 0;
    s->jd_bit_pos = 0;
    s->jp_bit_pos = 0;
    s->phase4_hold_logged = false;
    s->phase4_ri_align_remaining = 0;
    s->jd_terminate_requested = false;
    s->jd_terminated_by_s = false;
    s->jd_terminated_by_su = false;
    s->jd_resync_wait = false;
    s->jp_terminate_requested = false;
    s->v92_su_seen = false;
    s->v92_su_bar_seen = false;
    s->training_complete = false;
    s->dil_terminate_requested = false;
    s->cp_ready = false;
    s->cp_ack_received = false;
    s->e_received = false;
    s->b1_received = false;
    s->phase4_mapper_ready = false;
    s->phase4_k = 0;
    s->phase4_d = 0;
    s->phase4_s = 0;
    s->phase4_sr = 0;
    memset(&s->phase4_shaper, 0, sizeof(s->phase4_shaper));
    s->phase4_frame_pos = V90_FRAME_LEN;
    s->mp_nbits = 0;
    s->mp_bit_pos = 0;
    s->data_cp_received = false;
    s->data_mapper_ready = false;
    s->data_mapper_k = 0;
    s->data_mapper_d = 0;
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    s->data_input_bits = 0;
    s->data_input_bit_count = 0;
    s->v92_suvu_received = false;
    s->v92_cpu_received = false;
    s->v92_remote_ack_received = false;
    s->v92_cpd_sent = false;
    s->v92_ack_sent = false;
    s->v92_tx_nbits = 0;
    s->v92_tx_pos = 0;
    v90_reset_data_pump_state(s);
    v90_dil_reset_tx(s);

    /* V.90 §9.3.1.1-.3: remain silent while the analogue modem sends
     * S/S-bar, PP, TRN and Ja. Ja detection is the trigger for Sd. */
    s->tx_phase = V90_TX_WAIT_JA;
}

void v90_enable_v92_phase3(v90_state_t *s)
{
    if (!s)
        return;
    s->v92_phase3 = true;
    v90_build_jp(s);
}

void v90_set_dil_descriptor(v90_state_t *s, const v90_dil_desc_t *desc)
{
    if (!s)
        return;

    memset(&s->dil, 0, sizeof(s->dil));
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    v90_dil_reset_tx(s);

    if (!desc)
        return;

    s->dil.n = desc->n;
    s->dil.lsp = (uint8_t)v90_clamp_positive(desc->lsp, V90_DIL_MAX_PAT_BITS);
    s->dil.ltp = (uint8_t)v90_clamp_positive(desc->ltp, V90_DIL_MAX_PAT_BITS);
    memcpy(s->dil.sp, desc->sp, sizeof(s->dil.sp));
    memcpy(s->dil.tp, desc->tp, sizeof(s->dil.tp));
    memcpy(s->dil.h, desc->h, sizeof(s->dil.h));
    memcpy(s->dil.ref, desc->ref, sizeof(s->dil.ref));
    memcpy(s->dil.train_u, desc->train_u, sizeof(s->dil.train_u));
    s->dil_requested = (s->dil.n > 0);
}

const char *v90_rx_event_name(v90_rx_event_t event)
{
    switch (event) {
    case V90_RX_EVENT_NONE:           return "NONE";
    case V90_RX_EVENT_INFO1A_VALID:   return "INFO1A_VALID";
    case V90_RX_EVENT_INFO1A_INVALID: return "INFO1A_INVALID";
    case V90_RX_EVENT_S:              return "S";
    case V90_RX_EVENT_TRN_LOCK:       return "TRN_LOCK";
    case V90_RX_EVENT_J:              return "J";
    case V90_RX_EVENT_J_PRIME:        return "J_PRIME";
    case V90_RX_EVENT_SU:             return "SU";
    case V90_RX_EVENT_SU_BAR:         return "SU_BAR";
    case V90_RX_EVENT_SU_FINAL:       return "SU_FINAL";
    case V90_RX_EVENT_CP_VALID:       return "CP_VALID";
    case V90_RX_EVENT_CP_INVALID:     return "CP_INVALID";
    case V90_RX_EVENT_E:              return "E";
    case V90_RX_EVENT_B1:             return "B1";
    case V90_RX_EVENT_FAILURE:        return "FAILURE";
    case V90_RX_EVENT_RETRAIN:        return "RETRAIN";
    case V90_RX_EVENT_TIMEOUT:        return "TIMEOUT";
    }
    return "UNKNOWN";
}

bool v90_handle_rx_event(v90_state_t *s, v90_rx_event_t event)
{
    if (!s)
        return false;

    switch (event) {
    case V90_RX_EVENT_J:
        if (s->tx_phase == V90_TX_WAIT_JA) {
            int delay_samples = v90_sd_delay_samples(s);

            if (delay_samples > 0) {
                fprintf(stderr,
                        "[V90] Phase 3: analogue Ja detected, delaying Sd by %d ms (ME_V90_SD_DELAY_MS)\n",
                        delay_samples / 8);
                s->tx_phase = V90_TX_SD_DELAY;
            } else {
                fprintf(stderr, "[V90] Phase 3: analogue Ja detected, starting Sd\n");
                s->tx_phase = V90_TX_SD;
            }
            s->sample_count = 0;
            s->rep_count = 0;
            return true;
        }
        return false;

    case V90_RX_EVENT_S:
        if (s->v92_phase3)
            return false;
        if (s->tx_phase == V90_TX_JD
            && s->sample_count >= v90_min_jd_symbols()
            && !s->jd_terminate_requested) {
            fprintf(stderr,
                    "[V90] Phase 3: far-end S detected after %d Jd symbols, "
                    "terminating at the next frame boundary\n",
                    s->sample_count);
            s->jd_terminate_requested = true;
            s->jd_terminated_by_s = true;
            return true;
        }
        if (s->tx_phase == V90_TX_JD
            && !s->jd_terminate_requested
            && s->sample_count < v90_min_jd_symbols()) {
            fprintf(stderr,
                    "[V90] Phase 3: ignored early far-end S after %d Jd symbols "
                    "(minimum %d)\n",
                    s->sample_count, v90_min_jd_symbols());
        }
        if (s->tx_phase == V90_TX_DIL && !s->dil_terminate_requested) {
            int cycle_limit = v90_dil_autoterminate_cycles();
            int minimum_cycles = cycle_limit > 0 ? cycle_limit : 1;
            int minimum_segments = minimum_cycles * s->dil.n;

            /* The analogue modem may still produce the Su/Su-bar transition
             * near the J'd -> DIL boundary.  SpanDSP exposes both polarities
             * through the same phase-3 S event, and SCR or silence can also
             * resemble a sustained S rotation.  Hold DIL for at least one
             * complete descriptor cycle before allowing that ambiguous event
             * to end it. */
            if (s->dil_segment_index < minimum_segments) {
                fprintf(stderr,
                        "[V90] Phase 3: ignored early S/S-bar candidate "
                        "during DIL at segment %d/%d (minimum %d cycle%s)\n",
                        s->dil_segment_index + 1,
                        s->dil.n,
                        minimum_cycles,
                        minimum_cycles == 1 ? "" : "s");
                return false;
            }
            fprintf(stderr,
                    "[V90] Phase 3: S/S-bar received after minimum DIL "
                    "duration; terminating at the next segment boundary\n");
            s->dil_terminate_requested = true;
            return true;
        }
        return false;

    case V90_RX_EVENT_SU:
        if (s->v92_phase3 && s->tx_phase == V90_TX_JD && !s->v92_su_seen) {
            s->v92_su_seen = true;
            fprintf(stderr, "[V92] Phase 3: initial Su detected during Jd\n");
            return true;
        }
        return false;

    case V90_RX_EVENT_SU_BAR:
        if (s->v92_phase3 && s->tx_phase == V90_TX_JD
            && s->v92_su_seen && !s->v92_su_bar_seen) {
            s->v92_su_bar_seen = true;
            s->jd_terminate_requested = true;
            s->jd_terminated_by_su = true;
            fprintf(stderr,
                    "[V92] Phase 3: Su-to-S-bar-u detected; completing current Jd\n");
            return true;
        }
        return false;

    case V90_RX_EVENT_SU_FINAL:
        if (s->v92_phase3 && s->tx_phase == V90_TX_JP
            && !s->jp_terminate_requested) {
            s->jp_terminate_requested = true;
            fprintf(stderr,
                    "[V92] Phase 3: final Su transition detected; completing current Jp\n");
            return true;
        }
        return false;

    case V90_RX_EVENT_CP_VALID:
        if (s->v92_phase3
            && (s->tx_phase == V90_TX_DIL || s->tx_phase == V90_TX_SCR)
            && s->phase4_mapper_ready) {
            /* V.92 §9.5.1.1.12/.13: CPt ends the Phase-3 DIL/SCR wait and
             * starts Ri.  The following E1u/TRN2u stream is handled by the
             * native Phase-4 receiver once Ri reaches TRN2d. */
            fprintf(stderr,
                    "[V92] Phase 3: valid 2-point CPt received; starting Ri\n");
            s->tx_phase = V90_TX_RI;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return true;
        }
        if ((s->tx_phase == V90_TX_RI && s->sample_count >= v90_ri_length(s)
             && (s->v92_mode
                 ? (s->v92_native_cpu_rx ? s->phase4_mapper_ready
                                         : (s->cp_nbits > 0))
                 : s->phase4_mapper_ready))
            && !s->cp_ready) {
            fprintf(stderr, "[V90] Phase 4: valid far-end CPt received\n");
            s->cp_ready = true;
            s->phase4_ri_align_remaining =
                (V90_FRAME_LEN - (s->sample_count % V90_FRAME_LEN))
                % V90_FRAME_LEN;
            if (s->phase4_ri_align_remaining > 0) {
                fprintf(stderr,
                        "[V90] Phase 4: completing %d Ri symbols before barred Ri\n",
                        s->phase4_ri_align_remaining);
            }
            s->sample_count = 0;
            /* CPt is acknowledged by the following barred Ri (R-i).  Only
             * after that acknowledgement may the transmitter enter TRN2d. */
            s->tx_phase = V90_TX_RI_ACK;
            s->phase4_hold_logged = false;
            return true;
        }
        if (!s->v92_mode && s->data_cp_received
            && (s->tx_phase == V90_TX_TRN2D || s->tx_phase == V90_TX_MP)) {
            fprintf(stderr, "[V90] Phase 4: valid far-end %s received\n",
                    s->cp_ack_received ? "CP'" : "data-mode CP");
            return true;
        }
        return false;

    case V90_RX_EVENT_E:
        if (s->v92_mode && s->v92_native_cpu_rx) {
            /* §9.6.1.1.4/V.92: E2u counts as remote acknowledgement. */
            if (s->tx_phase == V90_TX_SUVD || s->tx_phase == V90_TX_CP) {
                fprintf(stderr,
                        "[V90] Phase 4 V.92: far-end E2u received; completing current sequence\n");
                s->e_received = true;
                s->v92_remote_ack_received = true;
                return true;
            }
            return false;
        }
        if (s->tx_phase == V90_TX_MP && s->mp_acknowledge
            && s->data_mapper_ready) {
            fprintf(stderr,
                    "[V90] Phase 4: far-end 20-bit E received; completing current MP'\n");
            s->e_received = true;
            return true;
        }
        return false;

    case V90_RX_EVENT_B1:
        if (s->tx_phase == V90_TX_B1D || s->tx_phase == V90_TX_DATA) {
            fprintf(stderr, "[V90] Phase 4: far-end B1 received\n");
            s->b1_received = true;
            return true;
        }
        return false;

    case V90_RX_EVENT_RETRAIN:
        /* The analogue modem abandoned Phase 3/4 and restarted its handshake.
         * Tear our Phase 3/4 state down and fall back to the silent WAIT_JA
         * state so we re-emit Sd cleanly when it next reaches Ja.  Continuing
         * to transmit Phase 3/4 here is actively harmful: it pours a
         * modem-like signal over the peer's fresh Phase 1/2 and corrupts the
         * bulk-delay/RTD estimate that places its next Sd search window. */
        if (s->tx_phase >= V90_TX_SD && s->tx_phase <= V90_TX_DATA) {
            fprintf(stderr,
                    "[V90] Peer retrained during tx_phase=%d; dropping to WAIT_JA\n",
                    (int) s->tx_phase);
            s->tx_phase = V90_TX_WAIT_JA;
            s->sample_count = 0;
            s->rep_count = 0;
            s->jd_bit_pos = 0;
            s->jd_terminate_requested = false;
            s->jd_terminated_by_s = false;
            s->jd_terminated_by_su = false;
    s->jd_resync_wait = false;
            s->jp_terminate_requested = false;
            s->v92_su_seen = false;
            s->v92_su_bar_seen = false;
            s->dil_terminate_requested = false;
            s->dil_requested = false;
            s->dil_segment_index = 0;
            s->phase4_hold_logged = false;
            s->cp_ready = false;
            s->cp_ack_received = false;
            s->phase4_mapper_ready = false;
            s->data_cp_received = false;
            s->data_mapper_ready = false;
            s->e_received = false;
            s->b1_received = false;
            return true;
        }
        /*endif*/
        return false;

    default:
        return false;
    }
}

void v90_notify_s_detected(v90_state_t *s)
{
    (void)v90_handle_rx_event(s, V90_RX_EVENT_S);
}

bool v90_training_complete(v90_state_t *s)
{
    return s ? s->training_complete : false;
}

bool v90_set_phase4_cp(v90_state_t *s, const vpcm_cp_frame_t *cp)
{
    vpcm_cp_frame_t expected;
    vpcm_cp_frame_t received;

    if (!s || !cp)
        return false;
    if (!vpcm_cp_encode_bits(cp, s->cp_bits, &s->cp_nbits))
        return false;
    s->cp_bit_pos = 0;

    if (s->v92_mode) {
        /* Native mode: a CPt (training parameters) configures the TRN2d
         * mapper used for the mapped SUVd/CPd/Ed transmit path. */
        if (s->v92_native_cpu_rx && !cp->v90_compatibility)
            return v90_configure_phase4_mapper(s, cp);
        s->cp_frame = *cp;
        return true;
    }

    /* Table 14 bit 19: 0 = CPt training parameters, 1 = data-mode CP. */
    if (!cp->v90_compatibility) {
        if (!s->phase4_mapper_ready)
            return v90_configure_phase4_mapper(s, cp);

        expected = s->cp_frame;
        received = *cp;
        expected.acknowledge = false;
        received.acknowledge = false;
        return !cp->acknowledge && vpcm_cp_frames_equal(&expected, &received);
    }

    if (!s->phase4_mapper_ready)
        return false;
    /* Acknowledged CP (CP') is valid only after the analogue modem has
     * received MP (§9.4.2).  Reject it before configuring the data mapper:
     * a rejected early CP' must not leave data_cp_received latched and cause
     * the first transmitted MP frame to become MP'. */
    if (cp->acknowledge && s->tx_phase != V90_TX_MP)
        return false;
    if (!s->data_mapper_ready) {
        vpcm_cp_frame_t base;

        /* §9.4.2: the analogue modem sets acknowledge in every CP it sends
         * after decoding a complete MP, so a peer that validated our MP
         * during TRN2d sends CP' as its FIRST data-mode CP.  Observed live
         * (2026-07-23, batch-2 call 5): SmartLink sent CP #1-#5 all with
         * ack=1; requiring a plain CP first rejected every one, we never
         * answered Ed/B1d, and the peer hit "Phase4 TimeOut" into a DP=90
         * retrain.  Configure the mapper from the constellation content
         * with acknowledge cleared and let the shared handling below latch
         * the acknowledgement under its existing MP-phase gate. */
        base = *cp;
        base.acknowledge = false;
        if (!v90_configure_data_mapper(s, &base))
            return false;
    }

    /* Repeated data-mode CP/CP' frames may change only acknowledge. */
    expected = s->data_cp_frame;
    received = *cp;
    expected.acknowledge = false;
    received.acknowledge = false;
    if (!vpcm_cp_frames_equal(&expected, &received))
        return false;
    if (cp->acknowledge)
        s->cp_ack_received = true;
    s->data_cp_received = true;
    s->data_cp_frame.acknowledge = cp->acknowledge;
    return true;
}

int v90_copy_phase4_mp_bits(const v90_state_t *s, uint8_t *bits, int max_bits)
{
    if (!s || !bits || !s->phase4_mapper_ready
        || s->mp_nbits <= 0 || max_bits < s->mp_nbits)
        return 0;
    memcpy(bits, s->mp_bits, (size_t)s->mp_nbits);
    return s->mp_nbits;
}

int v90_data_bits_per_frame(const v90_state_t *s)
{
    return (s && s->data_mapper_ready) ? s->data_mapper_d : 0;
}

int v90_data_input_bytes_needed(const v90_state_t *s)
{
    int missing;

    if (!s || !s->data_mapper_ready)
        return 0;
    missing = s->data_mapper_d - s->data_input_bit_count;
    return (missing > 0) ? (missing + 7) / 8 : 0;
}

int v90_tx_data_frame_codewords(v90_state_t *s,
                                uint8_t codewords[V90_FRAME_LEN],
                                const uint8_t *data,
                                int data_len,
                                int *data_consumed,
                                bool fill_with_ones)
{
    uint64_t frame_bits;
    uint64_t frame_mask;
    int needed;
    int consumed = 0;

    if (data_consumed)
        *data_consumed = 0;
    if (!s || !codewords || !s->data_mapper_ready || data_len < 0
        || (data_len > 0 && !data))
        return 0;

    needed = v90_data_input_bytes_needed(s);
    while (consumed < data_len && consumed < needed) {
        s->data_input_bits |= (uint64_t)data[consumed] << s->data_input_bit_count;
        s->data_input_bit_count += 8;
        consumed++;
    }
    if (data_consumed)
        *data_consumed = consumed;

    if (s->data_input_bit_count < s->data_mapper_d) {
        int missing;

        if (!fill_with_ones)
            return 0;
        missing = s->data_mapper_d - s->data_input_bit_count;
        s->data_input_bits |= ((1ULL << missing) - 1ULL)
                              << s->data_input_bit_count;
        s->data_input_bit_count = s->data_mapper_d;
    }

    frame_mask = (1ULL << s->data_mapper_d) - 1ULL;
    frame_bits = s->data_input_bits & frame_mask;
    s->data_input_bits >>= s->data_mapper_d;
    s->data_input_bit_count -= s->data_mapper_d;
    if (!v90_data_mapper_fill_frame(s, frame_bits))
        return 0;
    memcpy(codewords, s->data_mapper_frame, V90_FRAME_LEN);
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    return V90_FRAME_LEN;
}

void v90_notify_cp_ready(v90_state_t *s)
{
    (void)v90_handle_rx_event(s, V90_RX_EVENT_CP_VALID);
}

static int v90_codeword_to_ucode_law(v90_law_t law, uint8_t codeword)
{
    uint8_t magnitude = (uint8_t)(codeword & 0x7F);

    if (law == V90_LAW_ULAW)
        return 0x7F - magnitude;
    for (int ucode = 0; ucode < 128; ucode++) {
        if ((v90_ucode_to_alaw[ucode] & 0x7F) == magnitude)
            return ucode;
    }
    return -1;
}

int v90_demap_mapped_frame(v90_law_t law,
                           const vpcm_cp_frame_t *cp,
                           int bits_per_frame,
                           uint32_t *descramble_reg,
                           int *prev_sign,
                           const uint8_t codewords[V90_FRAME_LEN],
                           uint8_t bits_out[])
{
    int labels[V90_FRAME_LEN];
    int moduli[V90_FRAME_LEN];
    uint8_t scrambled[64];
    uint64_t r;
    int sign_prev;
    int k;

    if (!cp || !descramble_reg || !prev_sign || !codewords || !bits_out
        || cp->shaping_redundancy != 0
        || bits_per_frame < V90_FRAME_LEN
        || bits_per_frame > (int)sizeof(scrambled))
        return 0;
    k = bits_per_frame - V90_FRAME_LEN;

    sign_prev = *prev_sign;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int sign = (codewords[i] & 0x80) ? 1 : 0;
        int ucode = v90_codeword_to_ucode_law(law, codewords[i]);
        int label = 0;
        int m = 0;

        if (ucode < 0 || constellation >= cp->constellation_count
            || !vpcm_cp_mask_get(cp->masks[constellation], ucode))
            return 0;
        for (int u = 0; u < VPCM_CP_MASK_BITS; u++) {
            if (vpcm_cp_mask_get(cp->masks[constellation], u)) {
                /* The transmitter assigns labels in descending U-code
                   order; count selected values above this U-code. */
                if (u > ucode)
                    label++;
                m++;
            }
        }
        labels[i] = label;
        moduli[i] = m;
        scrambled[i] = (uint8_t)((sign ^ sign_prev) & 1);
        sign_prev = sign;
    }

    r = 0;
    for (int i = V90_FRAME_LEN - 1; i >= 0; i--)
        r = (uint64_t)moduli[i] * r + (uint64_t)labels[i];
    if (k < 64 && (r >> k) != 0)
        return 0;
    for (int i = 0; i < k; i++)
        scrambled[V90_FRAME_LEN + i] = (uint8_t)((r >> i) & 1);

    for (int i = 0; i < bits_per_frame; i++)
        bits_out[i] = (uint8_t)v90_descramble_reg_bit(descramble_reg,
                                                      scrambled[i]);
    *prev_sign = sign_prev;
    return bits_per_frame;
}

int v90_demap_shaped_frame(v90_law_t law,
                           const vpcm_cp_frame_t *cp,
                           int bits_per_frame,
                           uint32_t *descramble_reg,
                           v90_shaped_rx_state_t *shaper,
                           const uint8_t codewords[V90_FRAME_LEN],
                           uint8_t bits_out[])
{
    int labels[V90_FRAME_LEN];
    int moduli[V90_FRAME_LEN];
    uint8_t observed_signs[V90_FRAME_LEN];
    uint8_t scrambled[64];
    uint64_t r;
    int sr;
    int sign_bits;
    int modulus_bits;
    int recovered_word = -1;
    int recovered_trellis = 0;
    uint8_t recovered_prev_odd = 0;
    uint8_t recovered_prev_t[6] = {0};
    int matches = 0;

    if (!cp || !descramble_reg || !shaper || !codewords || !bits_out
        || bits_per_frame < V90_FRAME_LEN
        || bits_per_frame > (int)sizeof(scrambled))
        return 0;
    sr = cp->shaping_redundancy;
    if (sr < 1 || sr > 3 || (V90_FRAME_LEN % sr) != 0)
        return 0;
    sign_bits = V90_FRAME_LEN - sr;
    modulus_bits = bits_per_frame - sign_bits;
    if (modulus_bits < 0 || modulus_bits > 56)
        return 0;

    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int ucode = v90_codeword_to_ucode_law(law, codewords[i]);
        int label = 0;
        int m = 0;

        if (ucode < 0 || constellation >= cp->constellation_count
            || !vpcm_cp_mask_get(cp->masks[constellation], ucode))
            return 0;
        for (int u = 0; u < VPCM_CP_MASK_BITS; u++) {
            if (vpcm_cp_mask_get(cp->masks[constellation], u)) {
                if (u > ucode)
                    label++;
                m++;
            }
        }
        labels[i] = label;
        moduli[i] = m;
        observed_signs[i] = (uint8_t)((codewords[i] >> 7) & 1U);
    }
    r = 0;
    for (int i = V90_FRAME_LEN - 1; i >= 0; i--)
        r = (uint64_t)moduli[i] * r + (uint64_t)labels[i];
    if (modulus_bits < 64 && (r >> modulus_bits) != 0)
        return 0;

    for (int word = 0; word < (1 << sign_bits); word++) {
        v90_shaper_state_t trial;
        uint8_t input[V90_FRAME_LEN] = {0};
        uint8_t initial[V90_FRAME_LEN];

        memset(&trial, 0, sizeof(trial));
        trial.prev_odd = shaper->prev_odd;
        memcpy(trial.prev_t, shaper->prev_t, sizeof(shaper->prev_t));
        trial.trellis_state = shaper->trellis_state;
        for (int bit = 0; bit < sign_bits; bit++)
            input[bit] = (uint8_t)((word >> bit) & 1);
        v90_build_initial_shaping_signs(&trial, sr, input, initial);
        for (int choices = 0; choices < (1 << sr); choices++) {
            int trellis = shaper->trellis_state;
            int frame_length = V90_FRAME_LEN / sr;
            bool match = true;

            for (int frame = 0; frame < sr && match; frame++) {
                int rule = trellis == 0
                         ? ((choices >> frame) & 1)
                         : (2 + ((choices >> frame) & 1));

                for (int k = 0; k < frame_length; k++) {
                    int pos = frame * frame_length + k;
                    int sign = initial[pos]
                             ^ v90_shaper_rule_inverts(rule, k);

                    if (sign != observed_signs[pos]) {
                        match = false;
                        break;
                    }
                }
                trellis = (rule == 1 || rule == 3) ? 1 : 0;
            }
            if (match) {
                recovered_word = word;
                recovered_trellis = trellis;
                recovered_prev_odd = trial.prev_odd;
                memcpy(recovered_prev_t,
                       trial.prev_t,
                       sizeof(recovered_prev_t));
                matches++;
            }
        }
        if (matches > 1)
            return 0;
    }
    if (matches != 1)
        return 0;
    shaper->prev_odd = recovered_prev_odd;
    memcpy(shaper->prev_t,
           recovered_prev_t,
           sizeof(shaper->prev_t));
    shaper->trellis_state = (uint8_t)recovered_trellis;
    for (int bit = 0; bit < sign_bits; bit++)
        scrambled[bit] = (uint8_t)((recovered_word >> bit) & 1);
    for (int bit = 0; bit < modulus_bits; bit++)
        scrambled[sign_bits + bit] = (uint8_t)((r >> bit) & 1);
    for (int bit = 0; bit < bits_per_frame; bit++) {
        bits_out[bit] = (uint8_t)v90_descramble_reg_bit(
            descramble_reg,
            scrambled[bit]);
    }
    return bits_per_frame;
}

int v90_demap_shaped_sign_frame(const vpcm_cp_frame_t *cp,
                                v90_shaped_rx_state_t *shaper,
                                const uint8_t signs[6],
                                uint8_t scrambled_sign_bits[5])
{
    int sr;
    int sign_bits;
    int recovered_word = -1;
    int recovered_trellis = 0;
    uint8_t recovered_prev_odd = 0;
    uint8_t recovered_prev_t[6] = {0};
    int matches = 0;

    if (!cp || !shaper || !signs || !scrambled_sign_bits)
        return 0;
    sr = cp->shaping_redundancy;
    if (sr < 1 || sr > 3 || V90_FRAME_LEN % sr != 0)
        return 0;
    sign_bits = V90_FRAME_LEN - sr;
    if (sr == 1) {
        uint8_t recovered[5] = {0};
        uint8_t recovered_t[6] = {0};
        uint8_t recovered_odd = 0;

        for (int choice = 0; choice < 2; choice++) {
            int rule = shaper->trellis_state == 0
                     ? choice : 2 + choice;
            uint8_t t[6];
            uint8_t p_prime[6];
            uint8_t bits[5];

            for (int k = 0; k < 6; k++) {
                t[k] = (uint8_t)((signs[k] & 1U)
                     ^ v90_shaper_rule_inverts(rule, k));
                p_prime[k] = t[k] ^ shaper->prev_t[k];
            }
            if (p_prime[0] != 0)
                continue;
            bits[0] = p_prime[1] ^ shaper->prev_odd;
            bits[1] = p_prime[2];
            bits[2] = p_prime[3] ^ p_prime[1];
            bits[3] = p_prime[4];
            bits[4] = p_prime[5] ^ p_prime[3];
            memcpy(recovered, bits, sizeof(recovered));
            memcpy(recovered_t, t, sizeof(recovered_t));
            recovered_odd = p_prime[5];
            recovered_trellis = (rule == 1 || rule == 3) ? 1 : 0;
            matches++;
        }
        if (matches != 1)
            return 0;
        memcpy(scrambled_sign_bits, recovered, sizeof(recovered));
        memcpy(shaper->prev_t, recovered_t, sizeof(shaper->prev_t));
        shaper->prev_odd = recovered_odd;
        shaper->trellis_state = (uint8_t)recovered_trellis;
        return sign_bits;
    }
    for (int word = 0; word < (1 << sign_bits); word++) {
        v90_shaper_state_t trial;
        uint8_t input[V90_FRAME_LEN] = {0};
        uint8_t initial[V90_FRAME_LEN];

        memset(&trial, 0, sizeof(trial));
        trial.prev_odd = shaper->prev_odd;
        memcpy(trial.prev_t, shaper->prev_t, sizeof(shaper->prev_t));
        trial.trellis_state = shaper->trellis_state;
        for (int bit = 0; bit < sign_bits; bit++)
            input[bit] = (uint8_t)((word >> bit) & 1);
        v90_build_initial_shaping_signs(&trial, sr, input, initial);
        for (int choices = 0; choices < (1 << sr); choices++) {
            int trellis = shaper->trellis_state;
            int frame_length = V90_FRAME_LEN / sr;
            bool match = true;

            for (int frame = 0; frame < sr && match; frame++) {
                int rule = trellis == 0
                         ? ((choices >> frame) & 1)
                         : (2 + ((choices >> frame) & 1));

                for (int k = 0; k < frame_length; k++) {
                    int pos = frame * frame_length + k;
                    int sign = initial[pos]
                             ^ v90_shaper_rule_inverts(rule, k);

                    if (sign != (signs[pos] & 1U)) {
                        match = false;
                        break;
                    }
                }
                trellis = (rule == 1 || rule == 3) ? 1 : 0;
            }
            if (match) {
                recovered_word = word;
                recovered_trellis = trellis;
                recovered_prev_odd = trial.prev_odd;
                memcpy(recovered_prev_t,
                       trial.prev_t,
                       sizeof(recovered_prev_t));
                matches++;
            }
        }
    }
    if (matches != 1)
        return 0;
    shaper->prev_odd = recovered_prev_odd;
    memcpy(shaper->prev_t, recovered_prev_t, sizeof(shaper->prev_t));
    shaper->trellis_state = (uint8_t)recovered_trellis;
    for (int bit = 0; bit < sign_bits; bit++)
        scrambled_sign_bits[bit] = (uint8_t)((recovered_word >> bit) & 1);
    return sign_bits;
}

int v90_track_known_shaped_sign_frame(
                                const vpcm_cp_frame_t *cp,
                                v90_shaped_rx_state_t *shaper,
                                const uint8_t scrambled_sign_bits[6],
                                const uint8_t observed_signs[6])
{
    v90_shaper_state_t trial;
    uint8_t initial[V90_FRAME_LEN];
    int sr;
    int frame_length;
    int best_errors = INT_MAX;
    int best_trellis = 0;

    if (!cp || !shaper || !scrambled_sign_bits || !observed_signs)
        return -1;
    sr = cp->shaping_redundancy;
    if (sr < 1 || sr > 3 || V90_FRAME_LEN % sr != 0)
        return -1;
    frame_length = V90_FRAME_LEN / sr;
    memset(&trial, 0, sizeof(trial));
    trial.prev_odd = shaper->prev_odd;
    memcpy(trial.prev_t, shaper->prev_t, sizeof(trial.prev_t));
    trial.trellis_state = shaper->trellis_state;
    v90_build_initial_shaping_signs(&trial,
                                    sr,
                                    scrambled_sign_bits,
                                    initial);
    for (int choices = 0; choices < (1 << sr); choices++) {
        int trellis = shaper->trellis_state;
        int errors = 0;

        for (int frame = 0; frame < sr; frame++) {
            int rule = trellis == 0
                     ? ((choices >> frame) & 1)
                     : (2 + ((choices >> frame) & 1));

            for (int k = 0; k < frame_length; k++) {
                int pos = frame * frame_length + k;
                int sign = initial[pos]
                         ^ v90_shaper_rule_inverts(rule, k);

                errors += sign != (observed_signs[pos] & 1U);
            }
            trellis = (rule == 1 || rule == 3) ? 1 : 0;
        }
        if (errors < best_errors) {
            best_errors = errors;
            best_trellis = trellis;
        }
    }
    shaper->prev_odd = (uint8_t)trial.prev_odd;
    memcpy(shaper->prev_t, trial.prev_t, sizeof(shaper->prev_t));
    shaper->trellis_state = (uint8_t)best_trellis;
    return best_errors;
}

int v90_generate_trn2d_codewords(v90_law_t law,
                                 const vpcm_cp_frame_t *cp,
                                 const v90_shaped_rx_state_t *initial_state,
                                 int frames,
                                 uint8_t codewords_out[],
                                 int codewords_max)
{
    v90_state_t local;
    v90_scrambler_t scrambler;
    v90_shaper_state_t shaper;
    int bits_per_frame;
    int sign_bits;
    int modulus_bits;
    int prev_sign = 0;
    int output_frame = 0;
    int input_frames;

    if (!cp || !initial_state || !codewords_out || frames <= 0
        || codewords_max < frames * V90_FRAME_LEN
        || cp->shaping_redundancy > 3
        || cp->shaping_lookahead > 3)
        return 0;
    bits_per_frame = cp->drn + 8;
    sign_bits = V90_FRAME_LEN - cp->shaping_redundancy;
    modulus_bits = bits_per_frame - sign_bits;
    if (modulus_bits < 0 || modulus_bits > 56)
        return 0;
    memset(&local, 0, sizeof(local));
    memset(&shaper, 0, sizeof(shaper));
    local.law = law;
    shaper.prev_odd = initial_state->prev_odd;
    memcpy(shaper.prev_t,
           initial_state->prev_t,
           sizeof(initial_state->prev_t));
    shaper.trellis_state = initial_state->trellis_state;
    v90_scrambler_init(&scrambler);
    /* ld is measured in shaping frames. Buffer enough six-symbol PCM frames
     * to expose ld future shaping frames, then feed the same number of extra
     * all-ones frames to emit the requested final TRN2d frame.  With §5.4.5's
     * shaping disabled (Sr = 0) there is no look-ahead and no delay. */
    input_frames = frames + v90_shaper_delay_frames(cp);
    for (int input_frame = 0; input_frame < input_frames; input_frame++) {
        uint8_t scrambled[64];

        for (int bit = 0; bit < bits_per_frame; bit++)
            scrambled[bit] = (uint8_t)v90_scramble_bit(&scrambler, 1);
        if (cp->shaping_redundancy == 0) {
            if (!v90_map_scrambled_frame(
                    &local,
                    cp,
                    modulus_bits,
                    scrambled,
                    &prev_sign,
                    codewords_out + output_frame * V90_FRAME_LEN))
                return 0;
            output_frame++;
            continue;
        }
        if (!v90_map_shaped_scrambled_frame(
                &local,
                cp,
                cp->shaping_redundancy,
                sign_bits,
                modulus_bits,
                scrambled,
                &shaper,
                codewords_out + output_frame * V90_FRAME_LEN)) {
            if (shaper.pending_count > 0
                && shaper.pending_count <= v90_shaper_delay_frames(cp)) {
                continue;
            }
            return 0;
        }
        output_frame++;
    }
    return output_frame * V90_FRAME_LEN;
}

int v90_generate_phase4_codewords(v90_law_t law,
                                  const vpcm_cp_frame_t *cp,
                                  const v90_shaped_rx_state_t *initial_state,
                                  const uint8_t plain_bits[],
                                  int frames,
                                  uint8_t codewords_out[],
                                  int codewords_max)
{
    v90_state_t local;
    v90_scrambler_t scrambler;
    v90_shaper_state_t shaper;
    int bits_per_frame;
    int sign_bits;
    int modulus_bits;
    int prev_sign = 0;
    int output_frame = 0;
    int input_frames;

    if (!cp || !initial_state || !plain_bits || !codewords_out || frames <= 0
        || codewords_max < frames * V90_FRAME_LEN
        || cp->shaping_redundancy > 3
        || cp->shaping_lookahead > 3)
        return 0;
    /* Table 14: CPt carries drn+8 bits while data-mode CP carries drn+20.
     * The public generator is used for both; treating CP as CPt made offline
     * B1d vectors exercise the wrong mapper rate. */
    bits_per_frame = cp->drn + (cp->v90_compatibility ? 20 : 8);
    sign_bits = V90_FRAME_LEN - cp->shaping_redundancy;
    modulus_bits = bits_per_frame - sign_bits;
    if (modulus_bits < 0 || modulus_bits > 56)
        return 0;
    memset(&local, 0, sizeof(local));
    memset(&shaper, 0, sizeof(shaper));
    local.law = law;
    shaper.prev_odd = initial_state->prev_odd;
    memcpy(shaper.prev_t,
           initial_state->prev_t,
           sizeof(initial_state->prev_t));
    shaper.trellis_state = initial_state->trellis_state;
    v90_scrambler_init(&scrambler);
    input_frames = frames + v90_shaper_delay_frames(cp);
    for (int input_frame = 0; input_frame < input_frames; input_frame++) {
        uint8_t scrambled[64];

        for (int bit = 0; bit < bits_per_frame; bit++) {
            int source_frame = input_frame < frames
                             ? input_frame : frames - 1;
            int plain = plain_bits[source_frame * bits_per_frame + bit] & 1;

            scrambled[bit] = (uint8_t)v90_scramble_bit(&scrambler, plain);
        }
        if (cp->shaping_redundancy == 0) {
            if (!v90_map_scrambled_frame(
                    &local,
                    cp,
                    modulus_bits,
                    scrambled,
                    &prev_sign,
                    codewords_out + output_frame * V90_FRAME_LEN))
                return 0;
            output_frame++;
            continue;
        }
        if (!v90_map_shaped_scrambled_frame(
                &local,
                cp,
                cp->shaping_redundancy,
                sign_bits,
                modulus_bits,
                scrambled,
                &shaper,
                codewords_out + output_frame * V90_FRAME_LEN)) {
            if (shaper.pending_count > 0
                && shaper.pending_count <= v90_shaper_delay_frames(cp)) {
                continue;
            }
            return 0;
        }
        output_frame++;
    }
    return output_frame * V90_FRAME_LEN;
}

void v90_enable_v92_mode(v90_state_t *s)
{
    if (!s)
        return;
    s->v92_mode = true;
    /* Default Table 30 CPd profile until real upstream measurements or an
     * explicit v90_set_v92_cpd_profile() call refine it. */
    if (s->v92_gain_q0_16 == 0) {
        s->v92_upstream_drn = 14;      /* (14 + 17) x 8000 / 6 bps */
        s->v92_trellis_select = 0;     /* 16-state */
        s->v92_gain_q0_16 = 0x8000;    /* 4G = 0.5 -> G = 0.125 */
    }
}

bool v90_set_v92_cpd_profile(v90_state_t *s,
                             uint8_t upstream_drn,
                             uint8_t trellis_select,
                             uint16_t gain_q0_16)
{
    if (!s || upstream_drn > 19 || trellis_select > 2 || gain_q0_16 == 0)
        return false;
    s->v92_upstream_drn = upstream_drn;
    s->v92_trellis_select = trellis_select;
    s->v92_gain_q0_16 = gain_q0_16;
    return true;
}

void v90_enable_v92_native_cpu_rx(v90_state_t *s)
{
    if (s && s->v92_mode)
        s->v92_native_cpu_rx = true;
}

bool v90_set_v92_suvu(v90_state_t *s, bool acknowledge)
{
    if (!s || !s->v92_mode || !s->v92_native_cpu_rx)
        return false;
    s->v92_suvu_received = true;
    if (acknowledge)
        s->v92_remote_ack_received = true;
    return true;
}

bool v90_set_v92_cpu(v90_state_t *s, const vpcm_cp_frame_t *cpu)
{
    vpcm_cp_frame_t expected;
    vpcm_cp_frame_t received;

    if (!s || !cpu || !s->v92_mode || !s->v92_native_cpu_rx)
        return false;
    /* CPu carries data-mode parameters (d = drn + 20). */
    if (!cpu->v90_compatibility)
        return false;

    if (!s->v92_cpu_received) {
        expected = *cpu;
        expected.acknowledge = false;
        if (!v90_configure_data_mapper(s, &expected))
            return false;
        s->data_cp_frame.acknowledge = cpu->acknowledge;
        s->v92_cpu_received = true;
        if (cpu->acknowledge)
            s->v92_remote_ack_received = true;
        fprintf(stderr,
                "[V90] Phase 4 V.92: CPu%s accepted (drn=%u, D=%d, K=%d, Sr=%d)\n",
                cpu->acknowledge ? "'" : "", (unsigned)cpu->drn,
                s->data_mapper_d, s->data_mapper_k, s->data_mapper_sr);
        return true;
    }

    /* Repeated CPu/CPu' frames may change only the acknowledge bit. */
    expected = s->data_cp_frame;
    received = *cpu;
    expected.acknowledge = false;
    received.acknowledge = false;
    if (!vpcm_cp_frames_equal(&expected, &received))
        return false;
    if (cpu->acknowledge) {
        s->v92_remote_ack_received = true;
        fprintf(stderr, "[V90] Phase 4 V.92: CPu' acknowledgement received\n");
    }
    s->data_cp_frame.acknowledge = cpu->acknowledge;
    return true;
}

bool v90_get_v92_cpu(const v90_state_t *s, vpcm_cp_frame_t *out)
{
    if (!s || !out || !s->v92_cpu_received)
        return false;
    *out = s->data_cp_frame;
    return true;
}

/* ---- V.90 §9.6 rate renegotiation (digital modem) ---- */

/* §9.6: "The rate renegotiation procedure can be initiated at any time during
 * data mode.  Data signalling rate and spectral shaping parameters may change
 * as a result of rate renegotiation."
 *
 * For this modem it is also the recovery from a lost upstream.  §9.6.1.1 has
 * the digital modem send Rd, R̄d, optional TRN2d and then MP, and the analogue
 * modem answer with S, S̄, SCR, CP and -- after E -- a fresh B1.  That B1 is
 * what our upstream receiver acquires against, so a renegotiation gives it the
 * same reach it had at startup.  Nudging the state it is already in does not:
 * measured twice, no sampling offset within half a symbol and no rotation
 * recovers the constellation after one of this peer's one-sample timing slips
 * (docs/v90_upstream_data_path.md).
 *
 * The request is only armed here.  §9.6 requires that "rate renegotiation
 * shall be initiated by the digital modem's transmitter only on the boundary
 * of a data frame", so the transition itself happens in
 * v90_rate_renegotiation_due(), which the transmit path calls between frames.
 */
bool v90_request_rate_renegotiation(v90_state_t *s)
{
    if (!s || !s->training_complete || s->tx_phase != V90_TX_DATA)
        return false;
    if (s->reneg_pending || s->reneg_active)
        return false;
    s->reneg_pending = true;
    fprintf(stderr,
            "[V90] Rate renegotiation requested; starting at the next data "
            "frame boundary (§9.6.1.1)\n");
    return true;
}

bool v90_rate_renegotiation_pending(const v90_state_t *s)
{
    return s && s->reneg_pending;
}

bool v90_rate_renegotiation_active(const v90_state_t *s)
{
    return s && (s->reneg_pending || s->reneg_active);
}

int v90_rate_renegotiation_count(const v90_state_t *s)
{
    return s ? s->reneg_count : 0;
}

/* §9.6.1: "The digital modem shall initiate a retrain according to 9.5.1.1 if
 * it does not receive an E sequence within 5000 ms plus 2 round-trip delays
 * after transmitting the Rd - to - Rd transition."  The engine owns the
 * retrain; this only reports that one is owed. */
bool v90_rate_renegotiation_timed_out(const v90_state_t *s)
{
    return s && s->reneg_timed_out;
}

/* Begin the renegotiation.  Call only on a data frame boundary: §9.6 requires
 * it, and the analogue modem's receiver keeps data frame synchronization
 * through the whole procedure on the strength of it. */
bool v90_rate_renegotiation_start(v90_state_t *s)
{
    if (!s || !s->reneg_pending)
        return false;
    s->reneg_pending = false;
    s->reneg_active = true;
    s->reneg_timed_out = false;
    s->reneg_rbar_symbol = -1;
    s->reneg_symbol_clock = 0;
    s->reneg_count++;
    /* Re-enter the Phase 4 sequence at Rd.  Everything from here -- Rd, the
     * barred Rd, TRN2d, MP/MP', Ed, B1d -- is the same machinery as startup,
     * which is exactly what §9.6.1.1 describes. */
    s->tx_phase = V90_TX_RI;
    s->sample_count = 0;
    s->cp_ready = false;
    s->phase4_hold_logged = false;
    s->phase4_ri_align_remaining = 0;
    s->training_complete = false;
    fprintf(stderr,
            "[V90] Rate renegotiation %d: sending Rd for %dT on a data frame "
            "boundary (§9.6.1.1)\n",
            s->reneg_count, V90_RD_RENEG_SYMBOLS);
    return true;
}

void v90_reset_data_mode(v90_state_t *s)
{
    if (!s)
        return;
    if (s->data_mapper_ready)
        v90_reset_negotiated_data_mapper(s);
    else
        v90_reset_data_pump_state(s);
}

int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len)
{
    for (int i = 0; i < len; i++) {
        uint8_t codeword = v90_phase3_codeword(s);
        amp[i] = v90_pcm_to_linear(s->law, codeword);
    }
    return len;
}

int v90_phase3_tx_codewords(v90_state_t *s, uint8_t codewords[], int len)
{
    if (!s || !codewords || len <= 0)
        return 0;
    for (int i = 0; i < len; i++)
        codewords[i] = v90_phase3_codeword(s);
    return len;
}

uint8_t v90_idle_codeword(v90_law_t law)
{
    return v90_pcm_idle(law);
}

/*
 * Encode one 6-symbol data frame.
 * Fills pcm_out[0..5] with G.711 codewords.
 */
static void v90_encode_frame(v90_state_t *s, const uint8_t *data_in,
                             uint8_t *pcm_out)
{
    for (int i = 0; i < V90_FRAME_LEN; i++)
        pcm_out[i] = v90_encode_octet_to_codeword(s, data_in[i]);
}

int v90_tx_codewords(v90_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len)
{
    int i;
    int count;

    if (!s || !g711_out || !data_in || g711_max <= 0 || data_len <= 0)
        return 0;

    count = (data_len < g711_max) ? data_len : g711_max;
    for (i = 0; i < count; i++)
        g711_out[i] = v90_encode_octet_to_codeword(s, data_in[i]);
    return count;
}

int v90_rx_codewords(v90_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len)
{
    int i;
    int count;

    if (!s || !data_out || !g711_in || data_max <= 0 || g711_len <= 0)
        return 0;

    count = (g711_len < data_max) ? g711_len : data_max;
    for (i = 0; i < count; i++) {
        if (!v90_decode_codeword_to_octet(s, g711_in[i], &data_out[i]))
            return i;
    }
    return count;
}

int v90_tx_data(v90_state_t *s, int16_t amp[], int len,
                const uint8_t *data_in, int data_len)
{
    int pos = 0;
    int consumed = 0;

    while (pos + V90_FRAME_LEN <= len) {
        if (consumed + V90_FRAME_LEN > data_len) {
            /* Not enough data — fill with idle */
            uint8_t idle = v90_idle_codeword(s->law);
            for (int i = 0; i < V90_FRAME_LEN && pos < len; i++)
                amp[pos++] = v90_pcm_to_linear(s->law, idle);
            continue;
        }

        uint8_t pcm_out[V90_FRAME_LEN];
        v90_encode_frame(s, &data_in[consumed], pcm_out);
        for (int i = 0; i < V90_FRAME_LEN; i++)
            amp[pos++] = v90_pcm_to_linear(s->law, pcm_out[i]);
        consumed += V90_FRAME_LEN;
    }

    return consumed;
}

void v90_tx_idle(v90_state_t *s, int16_t amp[], int len)
{
    uint8_t idle = v90_idle_codeword(s->law);
    int16_t sample = v90_pcm_to_linear(s->law, idle);
    for (int i = 0; i < len; i++)
        amp[i] = sample;
}

logging_state_t *v90_get_logging_state(v90_state_t *s)
{
    return v34_get_logging_state(s->v34);
}
