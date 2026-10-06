# V.90 Phase 3/4: what the Recommendation says each side sends and listens for, against what the code assumes

Started 2026-10-06 after the Intel V92 modem (ext 8411) showed that most of the
recent walls were not DSP faults but **a signal we were not listening for in the
form the peer sends it**. Text is V.90 (09/98) clause 9; `ITU Docs/` has the PDF.
The method for each row is the same: quote what the clause obliges the *other*
end to put on the wire, find the code that is supposed to notice it, and say
what that code actually requires. "Evidence" is a recording or a measurement,
never a reading of our own logs (see the trap in row 3).

## Rows

| # | Clause | What the peer transmits | What we assumed | Status |
|---|---|---|---|---|
| 1 | 9.3.2.7 / 9.3.1.5 | After Jd the analogue modem sends **S, held until it detects J'd** (the first one is ~0.3 s long against a 128T = 37 ms nominal S), then 9.3.2.8's S-bar for 16T | The S watch (`v34_rx_watch_v90_jd_s`) assumed a low/high carrier pair at 3000/3200 and required "this set high, the other low". **3429 baud has ONE carrier**, so the test was unsatisfiable. | Fixed 281d7910. Evidence: `serial-flex-a2` rx tap, lines 245/1959/3674 Hz at 0.72-0.81 |
| 2 | 9.3.1.6 / 9.3.2.10 | During DIL the analogue modem sends **S for 128T then S-bar for 16T** ("S-to-S-bar transition") to say it has enough; NOTE: the digital modem can miss it | The same watch was armed **only while Jd was on air**, and its `phase3_s_present` gate stayed true after Jd's S, so nothing listened for the second S at all. | Fixed dcdaa967 (DIL mode: stays armed, 200 ms holdoff) |
| 3 | 9.3.2.10 | The S above is the only thing that ends DIL | After the S was found, our own `p3_demod` structural check (`rejected_p3_structural`) rejected it, and the modem's **retrain Tone A (a constant dibit, which that check passes)** was accepted as the S/S-bar, so we sent Ri into a modem that had already retrained. Every log line said "S/S-bar received ... entering Phase 4". | Fixed dcdaa967: events the line watch found skip the structural check. **Trap: read what the accepted event IS on the rx tap before believing a stage log.** |
| 4 | 9.3.1.6 | "After receiving a *subsequent* S-to-S-bar transition" -- i.e. the 9.3.2.8 S-bar that follows J'd is NOT the terminator; the next one is | We approximate "subsequent" with "at least one full DIL cycle has been sent" and act on an S alone, not on the S-to-S-bar transition. A descriptor whose cycle is longer than the modem's 5000 ms window would have the real S land mid-cycle and be ignored. | Open. Replace the cycle count with a count of S-bar transitions (ignore exactly the first), and look for the transition, not the S. |
| 5 | Table 10 bits 34:36 | Upstream symbol rate 3000/3200/3429, and it "**shall be consistent with INFO1d**"; INFO0d bit 40 says whether the digital modem can run V.90 at 3429 | We advertise bit 40 = 0 and every INFO1d row for 3429 at zero -- and the modem picked 3429 anyway (INFO1a code 5) on 1 attempt in 3. `v90_selected_upstream_baud_locked()` quietly mapped 5 to 3200, so the SpanDSP receiver ran at 3429 for Phase 3 and our Phase 4 and data receivers at 3200. Measured: the modem's Phase 4 is repeated 1244-bit CPt at 3429 (white at 3200 and 3000), which no receiver of ours decodes. | Candidate fix written, **not committed, not yet tested live** (the serial adapter dropped off the Mac): treat INFO1a code 5 as invalid when we have not enabled 3429. Patch kept in the session scratchpad. |
| 6 | 9.3.1.3 | Sd may start up to 500 ms after Ja is *received* | Waited for a CRC-valid descriptor with no bound | Fixed earlier (`ME_V90_JA_HEURISTIC_FALLBACK_MS`), see CLAUDE.md |
| 7 | 9.4.1.1 / 9.4.2.1 | Digital modem sends Ri for >= 192T **and keeps sending it until CPt arrives**; the analogue modem sends CPt repeatedly and waits for Ri-to-Ri-bar | "Phase 4: Ri (192 symbols)" is only the minimum; the stage correctly holds Ri, but nothing times out on *why* no CPt arrived. A modem that sends CPt in a form we cannot decode (row 5) looks identical to one that sent nothing. | Open: log what the rx tap holds (baud/carrier hypothesis, GPA ones fraction **with the dibit histogram**) when Ri has run > 1 s with no CPt. |
| 8 | 9.5.1.2 | Tone A from the analogue modem preceded by 70 +/- 5 ms of silence | We detect it, but a Tone A can also be read as an S by the constellation detectors (constant dibit) | Row 3's guard covers DIL; check Jd, Ri and TRN2d the same way. |

## What has NOT been audited

* 9.3.2.4: Ja termination on the Sd-to-Sd-bar transition, and 1500 ms.
* 9.3.2.5 / 9.3.2.6: the 2040T minimum of TRN1d against the equaliser training we
  actually start (we send 20004T by measurement, which the clause allows).
* Jd framing against Table 13 end to end on the **receive** side (we only
  generate it).
* 9.4.1.3-9.4.1.6: MP/MP'/Ed/B1d sequencing against what a foreign analogue
  modem sends -- only the SmartLink and RasFinder peers have ever answered.
* 9.6: rate renegotiation signals (S 128T / S-bar 16T / SCR / CP) -- the same
  S-detection assumptions as rows 1-2 apply, and the 9.6 watch
  (`v34_v90_watch_reneg_s`) has the same low/high structure.
* V.92 clause 9 equivalents (QC, short Phase 2, Table 18) and V.8bis.
* INFO0/INFO1 field-by-field: bits 14, 19 and 40 above were found by accident
  (row 5), so the rest of Tables 9-11 deserve the same read.

## Method notes

* **Demodulate the peer's signal outside the receiver before blaming it, and
  print the dibit histogram beside any ones fraction.** The first attempt at
  row 5 read "92.9% ones, so SCR" off the descrambler convention that
  maximises ones; `tools/v90_phase4_capture_check.py` with the same baud and
  carrier reads 5-10% ones (CPt data) and 1244-bit frames.
* A stage log that says a thing was "accepted" says what our code decided, not
  what arrived (row 3 read as success on four consecutive calls).
* When an expectation names a rate, a carrier or a length, ask whether the
  peer is *obliged* to follow it: row 5's modem violates "shall be consistent
  with INFO1d", so a receiver that follows it silently is as wrong as one that
  ignores it.
