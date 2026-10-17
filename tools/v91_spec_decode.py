#!/usr/bin/env python3
"""Decode OUR OWN V.91 transmit taps with a receiver written from the text.

Like tools/v90_trn2d_spec_decode.py, this file imports nothing from the tree
and shares no tables with v91.c / v90.c, so it can catch a wrong-but-
self-consistent convention that the loopback tests and the two-instance call
(both of which use our own encoder on both ends) cannot.  Sources:

  V.91 7.4-7.11, 7.8 Table 6 (INFO), 7.2 Table 4 (CP), 7.3 Table 5 (default
  DIL), 6.3-6.4 Table 3 (control channel); V.90 8.4.1 (DIL), Table 1 (Ucodes),
  5.3 (GPC scrambler), 5.4.2-5.4.7 (encoder: signs d0..d5, modulus encoder with
  interval 0 least significant, labels in DESCENDING Ucode order, sign bits
  differentially coded).

Input: the two live taps of a call between two instances of this modem
(VPCM_G711_TAP_DIR=<dir> on each, file <dir>/live-tx.g711).  Stream A is
decoded with the CP that stream B sent, and vice versa: a CP configures the
PEER's transmitter (7.2), so our B1 and data follow the peer's CP.

  v91_spec_decode.py <A/live-tx.g711> <B/live-tx.g711> [ulaw|alaw] [payloadA] [payloadB]

Checked per stream: Ez, INFO frames (sync, CRC, fields), Eu, the default DIL
symbol by symbol, SCR as scrambled ones, every CP (layout, start bits,
reserved bits, length a multiple of 6, CRC), Es, B1 as descrambled ones, every
data frame fitting the CP constellations, the control channel (slip pattern and
CRC of every 96-bit sequence), and optionally a payload string found in the
primary bit stream as 8N1 start-stop characters.
"""
import sys


def crc16(bits):
    """V.34 10.1.2.3.2 in the reflected form that validates V.90 MP/CP."""
    c = 0xFFFF
    for b in bits:
        c = ((c >> 1) ^ 0x8408) if ((b ^ c) & 1) else (c >> 1)
    return c


def field(bits, first, n):
    return sum(int(bits[first + k]) << k for k in range(n))


def to_ucode_sign(cw, law):
    """Table 1/V.90: Ucode ascending with magnitude; sign 1 = positive."""
    if law == "alaw":
        x = cw ^ 0x55
        return x & 0x7F, 1 if x & 0x80 else 0
    return (~cw) & 0x7F, 1 if cw & 0x80 else 0


class Gpc:
    """V.34 7-1 GPC descrambler, 1 + x^-18 + x^-23, self-synchronising."""
    def __init__(self):
        self.reg = 0

    def push(self, scrambled):
        out = (scrambled ^ (self.reg >> 17) ^ (self.reg >> 22)) & 1
        self.reg = ((self.reg << 1) | scrambled) & 0x7FFFFF
        return out


class Report:
    def __init__(self, name):
        self.name = name
        self.fails = []
        self.notes = []

    def ok(self, msg):
        self.notes.append("  ok   " + msg)

    def fail(self, msg):
        self.fails.append(msg)
        self.notes.append("  FAIL " + msg)

    def check(self, cond, msg):
        (self.ok if cond else self.fail)(msg)
        return cond


def parse_info(syms):
    """62 symbols -> (fields, problems).  Differential coder starts at 0."""
    prev, bits = 0, []
    for u, s in syms:
        bits.append(s ^ prev)
        prev = s
    problems = []
    if any(u != 66 for u, _ in syms):
        problems.append("not all Ucode 66")
    if bits[0:4] != [1, 1, 1, 1] or field(bits, 4, 8) != 0x72:
        problems.append("fill/frame sync")
    if bits[58:62] != [1, 1, 1, 1]:
        problems.append("trailing fill")
    if crc16(bits[12:42]) != field(bits, 42, 16):
        problems.append("CRC")
    f = dict(default_dil=(bits[26] == 0), cc=bits[27], ack=bits[28],
             maxpow=field(bits, 33, 5), post=bits[38], alaw=bits[39],
             transp=bits[40], cd=bits[41])
    return f, problems


def expected_default_dil():
    """V.91 Table 5 with V.90 8.4.1: 125 segments of (H+1)*6 = 12 symbols."""
    train = []
    for k in range(62):
        train += [124 - k, k]
    train.append(62)
    out = []
    for t in train:
        for i in range(12):
            sp = (0x0FC0 >> (i % 12)) & 1
            tp = (0x0FFF >> (i % 12)) & 1
            out.append((t if tp else 0, sp))      # REFc = Ucode 0
    return out


START_BITS = {17, 34, 51, 68, 85, 102, 119}


def cp_length(n):
    return (290 + 136 * (n - 1) + 5) // 6 * 6


def parse_cp(bits, rep):
    """bits: descrambled stream starting at CP bit 0.  Returns (cp, nbits)."""
    if len(bits) < 136:
        return None, 0
    dfi_pos = [103, 107, 111, 115, 120, 124]
    dfi = [field(bits, p, 4) for p in dfi_pos]
    n = max(dfi) + 1
    total = cp_length(n)
    if len(bits) < total:
        return None, 0
    b = bits[:total]
    starts = set(START_BITS)
    for c in range(n):
        for k in range(8):
            starts.add(136 + 136 * c + 17 * k)
    crc_start_bit = 272 + 136 * (n - 1)
    starts.add(crc_start_bit)
    probs = []
    if any(b[i] != 1 for i in range(17)):
        probs.append("frame sync")
    if any(b[i] != 0 for i in starts):
        probs.append("start bit")
    for lo, hi in ((25, 29), (30, 32), (35, 50), (52, 67), (69, 84), (86, 101),
                   (129, 135)):
        if any(b[i] != 0 for i in range(lo, hi + 1)):
            probs.append("reserved/zero bits %d:%d set" % (lo, hi))
    if b[128] != 0:
        probs.append("bit 128 (V.90 compat) not 0")
    body = [b[i] for i in range(18, crc_start_bit) if i not in starts]
    crc = field(b, crc_start_bit + 1, 16)
    if crc16(body) != crc:
        probs.append("CRC")
    if b[crc_start_bit + 17] != 0 or any(b[crc_start_bit + 17:]):
        probs.append("fill bits")
    masks = []
    for c in range(n):
        m = set()
        for k in range(8):
            base = 136 + 136 * c + 17 * k
            for j in range(16):
                if b[base + 1 + j]:
                    m.add(16 * k + j)
        masks.append(m)
    cp = dict(transparent=b[18], v90compat=b[19], drn=field(b, 20, 5), ack=b[33],
              dfi=dfi, masks=masks, n=n, nbits=total)
    if probs:
        for p in probs:
            rep.fail("CP: " + p)
    return cp, total


def find_infos(syms):
    """Ez then the run of valid INFO frames: (ez_start, end_of_ez, infos, end)."""
    i, N = 0, len(syms)
    ez = None
    while i + 24 <= N:
        if all(s == syms[i] and s[0] == 66 for s in syms[i:i + 24]):
            ez = i
            break
        i += 1
    if ez is None:
        return None, None, [], 0
    i = ez
    while i < N and syms[i] == syms[ez]:
        i += 1
    ez_end = i
    infos = []
    while i + 62 <= N:
        f, probs = parse_info(syms[i:i + 62])
        if probs:
            break
        infos.append(f)
        i += 62
    return ez, ez_end, infos, i


def quick_info(path, law):
    """The INFO' fields of a stream, or None."""
    syms = [to_ucode_sign(c, law) for c in open(path, "rb").read()]
    _, _, infos, _ = find_infos(syms)
    return infos[-1] if len(infos) >= 2 else None


def j_length(bits):
    """Table 12/V.90 length from the header fields (7.9)."""
    n = field(bits, 18, 8)
    lsp = field(bits, 35, 7) + 1
    ltp = field(bits, 43, 7) + 1
    alpha = (lsp + 15) // 16 * 17
    beta = alpha + (ltp + 15) // 16 * 17
    crc_start = 187 + beta + (n + 1) // 2 * 17
    total = crc_start + 18
    return total + (total & 1), crc_start, n, lsp, ltp


def check_startup_dil(rep, syms, i, own_def, peer_def):
    """Everything between the last INFO' and the end of the DIL (8.2.1.4-8.2.1.7,
    Figures 1-3).  The DIL a modem receives is the one it asked for, so when a
    descriptor is given here it is the DEFAULT one (this harness specifies the
    default descriptor through J, which keeps the DIL check comparable).
    Returns the index after the DIL."""
    prev = syms[i - 1][1]
    gpc = Gpc()
    if own_def and peer_def:
        eu = syms[i:i + 12]
        rep.check(len(eu) == 12 and all(u == 66 for u, _ in eu) and len({s for _, s in eu}) == 1,
                  "Eu: 12 x Ucode 66, differential zeros")
        i += 12
    else:
        if not own_def:
            # J: differential coder carries on from INFO, scrambler starts at zero (7.9)
            bits = []
            total = None
            j = i
            while j < len(syms) and syms[j][0] == 66:
                bits.append(gpc.push(syms[j][1] ^ prev))
                prev = syms[j][1]
                j += 1
                if total is None and len(bits) == 51:
                    if bits[:17] != [1] * 17 or bits[17] != 0:
                        break
                    total = j_length(bits)[0]
                if total is not None and len(bits) == total:
                    break
            if not rep.check(total is not None and len(bits) == total,
                             "J present after INFO' (bit 26 = 1)"):
                return None
            _, crc_start, n, lsp, ltp = j_length(bits)
            rep.check(bits[:17] == [1] * 17 and bits[17] == 0 and not any(bits[26:35])
                      and bits[42] == 0 and bits[50] == 0,
                      "J framing: 17 ones, zero, reserved zeros (Table 12/V.90)")
            # Table 12/V.90: the 17 sync ones + zero, bit 34 and every start bit
            # (each 17th from bit 51) are outside the CRC generator.
            covered = [b for k, b in enumerate(bits[:crc_start])
                       if not (k < 18 or k == 34 or (k >= 51 and (k - 51) % 17 == 0))]
            rep.check(crc16(covered) == field(bits, crc_start + 1, 16)
                      and bits[crc_start + 17] == 0,
                      "J CRC over %d bits" % crc_start)
            rep.check((n, lsp, ltp) == (125, 12, 12),
                      "J asks for the default descriptor (n=%d lsp=%d ltp=%d)" % (n, lsp, ltp))
            rep.ok("J: %d bits" % total)
            i = j
        else:
            gpc, prev = Gpc(), 0                  # PHIL without J restarts both coders (7.10)
        # PHIL (scrambled ones), then Em (12 scrambled zeros)
        ones = 0
        zeros = 0
        j = i
        while j < len(syms) and syms[j][0] == 66:
            b = gpc.push(syms[j][1] ^ prev)
            prev = syms[j][1]
            j += 1
            if b == 1 and zeros == 0:
                ones += 1
            elif b == 0:
                zeros += 1
                if zeros == 12:
                    break
            else:
                break
        rep.check(zeros == 12, "Em: 12 descrambled zeros after %d PHIL symbol(s)" % ones)
        if not own_def and peer_def:
            rep.check(ones == 0, "no PHIL when the DIL requested is already known (8.2.1.6)")
        if own_def or not peer_def:
            rep.check(ones > 0 or not own_def, "PHIL sent while waiting for the peer's J (8.2.1.6)")
        if zeros != 12:
            return None
        i = j
    exp = expected_default_dil()
    got = syms[i:i + len(exp)]
    bad = sum(1 for a, b in zip(got, exp) if a != b)
    rep.check(len(got) == len(exp) and bad == 0,
              "default DIL: %d symbols, %d mismatches against Table 5 / V.90 8.4.1" % (len(got), bad))
    return i + len(exp)


def check_cleardown_tail(rep, syms, start, own_def, peer_def):
    """8.4.1 + 8.6: a retrain start-up (Ez, INFO, DIL, SCR) whose first CP has
    drn = 0.  Returns True when the tail is exactly that."""
    ez, ez_end, infos, i = find_infos(syms[start:])
    if ez is None or len(infos) < 1:
        return False
    base = start
    sub = syms[base:]
    end = check_startup_dil(rep, sub, i, own_def, peer_def)
    if end is None:
        return False
    gpc, prev, bits, j = Gpc(), 0, [], end
    while j < len(sub) and sub[j][0] == 66 and len(bits) < 4000:
        bits.append(gpc.push(sub[j][1] ^ prev))
        prev = sub[j][1]
        j += 1
    z = next((k for k, b in enumerate(bits) if b == 0), None)
    cp, n = (None, 0)
    if z is not None and z >= 17:
        cp, n = parse_cp(bits[z - 17:], rep)
    if cp is None:
        # The peer clears down on the first CP it sees, so this end's own
        # retrain may stop short of its CP; what was sent must still be a
        # clean start-up (checked above).
        rep.ok("retrain start-up after data (Ez, INFO, DIL checked), cut short by the peer's cleardown")
        return True
    rep.check(cp["drn"] == 0 and not cp["ack"],
              "retrain start-up after data ends in CP with drn = 0 (cleardown, 8.4.1/8.6)")
    return cp["drn"] == 0


def decode_stream(path, law, peer_cc_requested_cb, peer_cp_cb, name, payload, peer_default=True):
    rep = Report(name)
    raw = open(path, "rb").read()
    syms = [to_ucode_sign(c, law) for c in raw]
    N = len(syms)
    ez, ez_end, infos, i = find_infos(syms)
    if not rep.check(ez is not None, "Ez found"):
        return rep, None, None
    rep.ok("Ez at %d (%d symbols)" % (ez, ez_end - ez))
    rep.check(len(infos) >= 2, "%d INFO frames decoded (sync, CRC, fill valid)" % len(infos))
    if len(infos) < 2:
        return rep, None, None
    rep.check(not infos[0]["ack"] and infos[-1]["ack"],
              "INFO repeats with ack=0, ends with INFO' (ack=1)")
    own_def = infos[-1]["default_dil"]
    rep.check(all(f["default_dil"] == own_def for f in infos),
              "DIL request in INFO bit 26 (%s) is the same in every frame"
              % ("default" if own_def else "specified"))
    info = infos[-1]
    rep.ok("INFO fields: cc=%d alaw=%d maxpow=%d transparent=%d" %
           (info["cc"], info["alaw"], info["maxpow"], info["transp"]))
    rep.check(info["alaw"] == (1 if law == "alaw" else 0), "INFO bit 39 matches the law")
    i = check_startup_dil(rep, syms, i, own_def, peer_default)
    if i is None:
        return rep, None, None
    # SCR / CP / Es : descramble from here
    gpc, prev = Gpc(), 0
    bits = []
    j = i
    while j < N:
        u, s = syms[j]
        if u != 66:
            break
        bits.append(gpc.push(s ^ prev))
        prev = s
        j += 1
        # stop once Es could have ended: handled below by parsing
        if len(bits) > 40000:
            break
    # first zero ends the scrambled-ones run; CP begins 17 ones earlier
    z = next((k for k, b in enumerate(bits) if b == 0), None)
    if not rep.check(z is not None and z >= 17, "SCR: descrambled ones, then CP frame sync (first zero at %s)" % z):
        return rep, None, None
    rep.ok("SCR ones run of %d symbols before the CP sync pattern" % (z - 17))
    pos = z - 17
    rep.check(pos % 6 == 0, "CP starts on a 6-symbol boundary (SCR is a multiple of 6: %d)" % pos)
    cps = []
    while pos < len(bits):
        if bits[pos] == 0:
            break                                  # Es
        cp, n = parse_cp(bits[pos:], rep)
        if cp is None:
            rep.fail("CP truncated")
            break
        cps.append(cp)
        rep.check(n % 6 == 0, "CP #%d length %d is a multiple of 6" % (len(cps), n))
        pos += n
    rep.check(len(cps) >= 2 and not cps[0]["ack"] and cps[-1]["ack"],
              "%d CP sequences, CP then CP' (acknowledge bit)" % len(cps))
    es = bits[pos:pos + 12]
    rep.check(es == [0] * 12, "Es: 12 descrambled zeros after the last CP")
    # Everything from B1 on is mapped through the PEER's CP (7.2)
    mycp = cps[-1] if cps else None
    peer_cp = peer_cp_cb()
    if peer_cp is None:
        return rep, mycp, info
    cp = peer_cp
    if mycp and mycp["transparent"] and cp["transparent"]:
        # 6.5 / 8.7: both CPs granted bit 18, so the data bits ARE the PCM
        # codewords (8 per symbol, LSB first), B1 is 12 codewords of binary
        # ones, drn = 28, and there is no control channel.
        es_end = i + pos + 12
        rep.check(mycp["drn"] == 28 and cp["drn"] == 28,
                  "transparent mode granted by both ends with drn 28 (8.7)")
        rep.check(raw[es_end:es_end + 12] == b"\xff" * 12,
                  "transparent B1: 12 codewords of binary ones (7.1)")
        data = raw[es_end + 12:]
        primary = [(c >> t) & 1 for c in data for t in range(8)]
        rep.check(len(data) > 0, "%d transparent data codewords follow B1 directly" % len(data))
        rep.ok("no control channel in transparent mode (6.5)")
        if payload:
            pat = []
            for ch in payload.encode():
                pat += [0] + [(ch >> t) & 1 for t in range(8)] + [1]
            rep.check("".join(map(str, pat)) in "".join(map(str, primary)),
                      "payload %r found in the transparent bit stream as 8N1 characters" % payload)
        return rep, mycp, info
    drn = cp["drn"]
    K, D = drn + 14, drn + 20
    rep.ok("our transmitter uses the peer's CP: drn %d (D=%d, K=%d), %d constellation(s), dfi=%s"
           % (drn, D, K, cp["n"], cp["dfi"]))
    # B1 begins right where the Ucode-66 run ended (Es) -- but B1 symbols are
    # mapped data, not Ucode 66, so find where non-66 begins.
    es_end = i + pos + 12                  # bits[] has one entry per symbol from i
    # re-run the descrambler and differential decoder to the end of Es
    gpc2, prev2 = Gpc(), 0
    for q in range(i, es_end):
        gpc2.push(syms[q][1] ^ prev2)
        prev2 = syms[q][1]
    pos_s = es_end

    def label_of(frame_i, u):
        m = sorted(cp["masks"][cp["dfi"][frame_i]], reverse=True)
        return (m.index(u) if u in m else None), len(m)

    def decode_frame(fs, cc):
        nonlocal prev2
        bs, labels, mods = [], [], []
        for q, (u, s) in enumerate(fs):
            lab, m = label_of(q, u)
            if lab is None:
                return None, "Ucode %d not in constellation of interval %d" % (u, q)
            bs.append(s ^ prev2)
            prev2 = s
            labels.append(lab)
            mods.append(m)
        R = 0
        for q in range(5, -1, -1):
            R = R * mods[q] + labels[q]
        if R >> K:
            return None, "modulus residue needs more than K=%d bits" % K
        d = bs + [(R >> t) & 1 for t in range(K)]
        out = []
        for t, x in enumerate(d):
            if cc and t < 6:
                out.append(x)
            else:
                out.append(gpc2.push(x))
        return out, None

    # B1 = two frames of ones (7.1)
    b1 = []
    ok_b1 = True
    for fi in range(2):
        out, err = decode_frame(syms[pos_s + 6 * fi: pos_s + 6 * fi + 6], False)
        if out is None:
            rep.fail("B1 frame %d: %s" % (fi, err))
            ok_b1 = False
            break
        b1 += out
    if ok_b1:
        rep.check(all(b1), "B1: %d descrambled bits all ones through the data encoder" % len(b1))
    pos_s += 12
    # 8.2.1.11 / 7.1: data follows B1 directly, on the frame grid B1 ends on.
    # (Anything else makes a conformant receiver decode our idle as data.)
    j2 = pos_s
    while j2 < N and syms[j2] == (0, 1):
        j2 += 1
    start = pos_s
    rep.check(j2 - pos_s < 6, "data follows B1 directly (idle symbols before the first data frame: %d)"
              % (j2 - pos_s))
    cc_active = peer_cc_requested_cb()
    nframes = good = 0
    ccbits, cc_seq_ok, cc_bad, cc_flags = [], 0, 0, set()
    primary = []
    errs = []
    f = start
    while f + 6 <= N:
        cc = cc_active and (nframes % 64 == 0)
        out, err = decode_frame(syms[f:f + 6], cc)
        if out is None:
            errs.append((nframes, err, f))
            break
        nframes += 1
        good += 1
        if cc:
            ccbits += out[:6]
            primary += out[6:]
            if len(ccbits) == 96:
                seq, ccbits = ccbits, []
                pat = [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0]
                if seq[:16] == pat and crc16(seq[:78]) == field(seq, 78, 16):
                    cc_seq_ok += 1
                    if seq[16]:
                        cc_flags.add("cleardown")
                    if seq[17]:
                        cc_flags.add("retrain")
                else:
                    cc_bad += 1
        else:
            primary += out
        f += 6
    # A stream may legitimately end in idle (the call was torn down); any
    # other undecodable frame is a defect.
    tail_idle = all(s == (0, 1) for s in syms[f:])
    tail_cleardown = False
    if errs and not tail_idle:
        tail_cleardown = check_cleardown_tail(rep, syms, f, own_def, peer_default)
    rep.check(not errs or tail_idle or tail_cleardown,
              "%d data frames decoded%s" % (good, "" if not errs else
                                           " (STOPPED at frame %d: %s; symbol %d; %d symbols of non-idle follow; next 12: %s)"
                                           % (errs[0][0], errs[0][1], errs[0][2],
                                              sum(1 for x in syms[f:] if x != (0, 1)),
                                              syms[f:f + 12]) if errs else ""))
    if cc_active:
        rep.check(cc_seq_ok > 0 and cc_bad == 0,
                  "control channel: %d sequences valid (slip pattern + CRC), %d bad%s"
                  % (cc_seq_ok, cc_bad, ", flags=" + ",".join(sorted(cc_flags)) if cc_flags else ""))
    else:
        rep.ok("control channel not requested by the receiver; none expected")
    if payload:
        pat = []
        for ch in payload.encode():
            pat += [0] + [(ch >> t) & 1 for t in range(8)] + [1]     # 8N1, LSB first
        s = "".join(map(str, primary))
        p = "".join(map(str, pat))
        rep.check(p in s, "payload %r found in the primary bit stream as 8N1 characters" % payload)
    return rep, mycp, info


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("caller_tap")
    parser.add_argument("answerer_tap")
    parser.add_argument("law", choices=("ulaw", "alaw"), nargs="?", default="ulaw")
    parser.add_argument("caller_payload", nargs="?")
    parser.add_argument("answerer_payload", nargs="?")
    parser.add_argument("--specify-dil", choices=("neither", "caller", "answerer", "both"))
    args = parser.parse_args()
    a_path, b_path, law = args.caller_tap, args.answerer_tap, args.law
    payloads = {"A": args.caller_payload, "B": args.answerer_payload}

    # first pass: just our own CP and INFO from each stream (no data decode)
    class Box:
        cp = None
        info = None

    qi = {"A": quick_info(a_path, law), "B": quick_info(b_path, law)}
    pdef = {"A": qi["B"]["default_dil"] if qi["B"] else True,
            "B": qi["A"]["default_dil"] if qi["A"] else True}
    boxes = {}
    for name, path in (("A", a_path), ("B", b_path)):
        r, cp, info = decode_stream(path, law, lambda: False, lambda: None, name, None,
                                    pdef[name])
        boxes[name] = (r, cp, info)
    fails = 0
    for name, path, other in (("A", a_path, "B"), ("B", b_path, "A")):
        _, _, other_info = boxes[other]
        _, other_cp, _ = boxes[other]
        cc = bool(other_info and other_info["cc"])
        rep, _, _ = decode_stream(path, law, lambda cc=cc: cc, lambda cp=other_cp: cp,
                                  name, payloads[name], pdef[name])
        if args.specify_dil is not None:
            # V.91 7.8 Table 6, 8.2.1.5: check the requested test arm on
            # the transmit tap, before accepting its J/PHIL/Em negotiation.
            specified = args.specify_dil in ("both", "caller" if name == "A" else "answerer")
            own_info = qi[name]
            rep.check(own_info is not None and own_info["default_dil"] == (not specified),
                      "INFO bit 26 matches requested %s DIL" % ("specified" if specified else "default"))
        print("== stream %s (%s) ==" % (name, path))
        print("\n".join(rep.notes))
        fails += len(rep.fails)
    print("RESULT: %s (%d failing check(s))" % ("PASS" if fails == 0 else "FAIL", fails))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
