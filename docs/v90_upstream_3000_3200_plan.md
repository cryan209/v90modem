# V.90 upstream 3000/3200 symbol-rate plan

## Scope and requirements

This plan covers the V.34 analogue-to-digital channel used as the V.90
upstream.  “Carrier support” means both required symbol rates and both V.34
carrier choices:

| Symbol rate | low carrier | high carrier |
|---|---:|---:|
| 3000 | 1800 Hz | 2000 Hz |
| 3200 | 12800/7 Hz (about 1829 Hz) | 1920 Hz |

V.90 §6.2 requires the digital modem to support **both 3000 and 3200
symbols/s**.  The analogue modem must support 3200 and may support 3000.
The analogue modem selects the upstream symbol rate during Phase 2.  Per
§8.2.3.2 Tables 9 and 10, that selection must be one of the rates enabled by
the digital modem's INFO1d, and the carrier and pre-emphasis are the values
INFO1d gave for that rate.

The bearer remains exactly 8000 G.711 codewords/s.  Internal resampling must
not alter external sample accounting.

## Current baseline

- V.34 TX/RX RRC tables exist for 3000-low/high and 3200-low/high.
- INFO0 and INFO1 encode/decode all four combinations.
- The analogue-role Phase 3 transmitter follows received INFO1d and has been
  observed live at 3200-low.
- The digital-role DATA receiver has a supervised B1/T/3 path, but it is
  hard-coded to 3200, 9.6 kHz and the 3200 carrier table.
- The clean PCMU 3200/21600 regression recovers at least 16000 payload bits
  without error.  Hardware payload interoperability is not yet established.

## Implementation steps

### 1. Make Phase-2 negotiation authoritative

1. Preserve, in the digital receiver, the per-rate carrier choices actually
   transmitted in INFO1d.
2. When INFO1a selects 3000 or 3200, configure the receiver from that stored
   INFO1d row rather than from Table 10 reserved bits or the initial V.34
   caller/answerer default.
3. Make the live Phase-3 confirmation path try the negotiated carrier and the
   alternate carrier, matching the Ja scanner's acquisition policy.
4. Pass the selected symbol-rate code explicitly through the CP-to-E/DATA
   handoff.  Remove the hidden 3200 default and diagnostic environment
   override from that protocol decision.
5. Cap the MP offer and selected upstream bit rate to a combination legal for
   the selected symbol rate (28800 at 3000; 31200 at 3200).

Acceptance:

- Unit tests preserve 3000 and 3200 across `V90_CP -> E -> DATA`.
- Invalid 3000/31200 preparation is rejected.
- Low and high carrier selections survive INFO1d/INFO1a processing.

### 2. Generalize the supervised B1/DATA receiver

1. Keep three internal samples per symbol, but derive the internal rate from
   the negotiated baud: 9000 Hz for 3000 and 9600 Hz for 3200.
2. Generalize the rational 8 kHz resampler using the exact internal rate.
3. Mix with the selected baud/carrier pair rather than the 3200 table row.
4. Build the reset-state B1 reference with the negotiated baud and mapper
   parameters.
5. Preserve one continuous resampler, carrier, timing and equalizer state from
   B1 into DATA.

Acceptance:

- External input count remains exactly the supplied 8 kHz sample count.
- Internal count approaches 9/8 of input at 3000 and 6/5 at 3200.
- Both rates acquire complete B1 and recover a deterministic payload without
  bit errors over a G.711 round trip.

### 3. Add the rate/carrier/law regression matrix

Run the native waveform test for:

- 3000-low and 3000-high at a legal data rate;
- 3200-low and 3200-high at the same data rate;
- PCMU and PCMA quantization.

Also assert the legal maximum-rate boundary separately for 3000 and 3200.
Tests must report symbol rate, carrier, law, external/internal sample counts,
alignment and compared payload bits.

### 4. Hardware qualification

Test 3200 first, then force or negotiate 3000 as the fallback.  For each call
record:

- INFO1d rows and selected INFO1a rate;
- selected low/high carrier and measured carrier offset;
- CP/CP-prime, E and complete-B1 acquisition;
- upstream/downstream negotiated bit rates;
- first LAPM frame and sustained upstream BER;
- any rate-change or retrain reason.

Synthetic success is not hardware interoperability.  Completion requires an
FCS-valid upstream LAPM exchange against a foreign modem at each claimed
symbol rate.

## Implementation status

Steps 1–3 are implemented offline:

- the digital RX retains and applies its transmitted INFO1d carrier row;
- INFO1a's selected 3000/3200 rate and carrier cross the CP-to-DATA seam
  explicitly;
- 3000 is capped at 28800 and 3200 at 31200;
- the T/3 receiver runs at 9000 or 9600 Hz from the negotiated symbol rate;
- all eight rate/carrier/law combinations recover 16000 deterministic payload
  bits without error through G.711 quantization.

Step 4, foreign-modem hardware qualification and LAPM evidence, remains open.

### 2026-08-11 d-modem acquisition result

A 3200-low call against the SmartLink `d-modem` rig exposed a prerequisite
before DATA: INFO1d correctly selected 1829 Hz, but the ordinary V.34 echo
front end erased Ja and left the peer in `WaitForSd` with zero error energy.
Acquisition is now multi-evidence rather than one fixed Table-18 threshold:

- a clean short Table-18 match is accepted immediately;
- a weaker candidate must persist as a periodic J run, preventing the old
  48-symbol TRN false positive;
- the rolling grader retains 800 ms and tests 3000/3200 low/high only;
- the physical signal/gap/Ja edge is graded before echo cancellation, while
  payload reception remains on the filtered path;
- a CRC-valid Ja descriptor remains authoritative whenever it arrives first.

Live evidence in
`artifacts/dmodem-3000-3200-live/v90-dmodem-adaptive5-20260811T214619Z/`
selected 3200-low, drove SmartLink's error energy from zero to about 9, caused
it to build its 428-bit CPt, and moved this receiver into `V90_CP`.  It then
retrained before CP validation, so the next foreign-modem blocker is CPt
acquisition rather than Ja/Sd acquisition.

### 2026-08-11 CPt replay result

The independent strict CP receiver was still hard-coded to 2400 baud even
though Table 10/INFO1a had selected 3200.  Its fixed RRC/DQPSK path now derives
samples per symbol and low/high carrier frequencies from that selected baud,
and its carrier-rise optimization no longer truncates the search eight seconds
after an earlier Phase-3 rise.  Replaying the exact live RX above now recovers
a CRC-clean 428-bit CPt at 3200 baud (`drn=15`, `Sr=1`, `ld=1`, PCMU, one
transmitter constellation with a separate codec mask).  The prior code reports
`no strict frame` on the same waveform.

A fresh post-restart run then validated the same path live.  The batch receiver
armed at upstream sample 128073, recovered the peer's CRC-clean 428-bit CPt at
sample 134211, and `v90.c` accepted it and began barred Ri followed by 2040
mapped TRN2d symbols (`D=23`, `K=18`).  SmartLink set `CPnot`, completed its
current CPt, and enabled its TRN2d study, proving that it recognized barred Ri.
Its internal known-TRN2 oracle produces the same signed 16-level sequence as
our transmitted stream, codeword for codeword.  The remaining failure is MP:
the peer enabled TRN2d about 240 ms after barred Ri, so the old 2040T minimum
left only about 120 observed conditioning symbols before MP and no MP decoded.
A controlled live A/B then bounded the usable interval.  At 8000T (1 s), MP
began at SmartLink's short Phase-4 timeout and it retrained.  At 4000T (500 ms),
SmartLink decoded MP and MP′, sent a CRC-clean 700-bit CP′, detected Ed, sent E
and B1, and both ends entered DATA at 31200 bit/s upstream and 52000 bit/s
downstream.  The default is therefore 4000T.  This proves the complete physical
startup through B1; FCS-valid LAPM traffic remains the hardware qualification
criterion.  Evidence:

- CPt/TRN2 oracle: `artifacts/dmodem-3000-3200-live/v90-dmodem-cpt2-20260811T220326Z/`
- successful 4000T DATA entry: `artifacts/dmodem-3000-3200-live/v90-dmodem-trn2-4000b-20260812T002922Z/`
