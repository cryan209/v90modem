#!/usr/bin/env node
/*
 * v90_ja_scan.js — find a Table 12 DIL descriptor in a demodulated Ja bit
 * stream, and check it against a captured downstream.
 *
 * Both jobs run on the parser and DIL generator in tools/v90_dil_lab.html,
 * which is a port of v90.c verified against it (see that file's header).  This
 * script only drives them, so a descriptor it reports is one v90.c would also
 * accept, CRC and framing included.
 *
 *   node tools/v90_ja_scan.js scan  <bits-file...>
 *       Each byte of the input is one bit (0/1), as the Ja replay harnesses
 *       dump them.  Every offset whose next 17 bits are ones is tried as a
 *       frame start, in both polarities.  §8.4.1's framing puts a zero at
 *       every seventeenth bit, so a false positive would have to survive
 *       ~120 forced zeros as well as the CRC — a hit is a descriptor.
 *
 *   node tools/v90_ja_scan.js align <hex> <g711-file...>
 *       Correlates the DIL that descriptor asks for against a µ-law capture,
 *       on levels rather than codewords (a pad scales every level and would
 *       break a codeword match).  Reports the best offset and score, which is
 *       how a descriptor recovered from the upstream is confirmed against the
 *       downstream it produced.
 */
'use strict';

const fs = require('fs');
const path = require('path');

const LAB = path.join(__dirname, 'v90_dil_lab.html');
const html = fs.readFileSync(LAB, 'utf8');
const core = /<script id="v90-dil-core">([\s\S]*?)<\/script>/.exec(html);
if (!core) {
  console.error('v90_dil_lab.html has no core script — cannot run.');
  process.exit(1);
}
(0, eval)(core[1]);
const V = globalThis.V90DIL;

function loadBits(file) {
  return Array.from(fs.readFileSync(file)).map((x) => (x ? 1 : 0));
}

function scanBits(bits) {
  const hits = [];
  for (let pol = 0; pol < 2; pol++) {
    const b = pol ? bits.map((x) => x ^ 1) : bits;
    for (let start = 0; start + 206 <= b.length; start++) {
      let i = 0;
      while (i < 17 && b[start + i]) i++;
      if (i !== 17 || b[start + 17]) continue;
      const n = b.length - start;
      const packed = new Uint8Array((n + 7) >> 3);
      for (let j = 0; j < n; j++) if (b[start + j]) packed[j >> 3] |= 1 << (j & 7);
      const r = V.parseDescriptor(packed, n);
      if (r.ok) hits.push({ start, polarity: pol, desc: r.desc, bitLen: r.bitLen });
    }
  }
  return hits;
}

function describe(d) {
  const c = V.validate(d);
  const packed = V.packDescriptor(d);
  return {
    n: d.n, lsp: d.lsp, ltp: d.ltp,
    h: Array.from(d.h).join(','),
    ref: Array.from(d.ref).join(','),
    train: Array.from(d.train.slice(0, d.n)).join(','),
    cycleSymbols: c.cycleSymbols, cycleMs: c.cycleMs,
    distinctUcodes: c.distinctUcodes, chords: c.chordsCovered,
    intervalsProbed: '0x' + c.intervalsProbed.toString(16),
    hex: packed ? V.bitsToHex(packed.bytes, packed.bitLen) : null,
    bitLen: packed ? packed.bitLen : 0
  };
}

/* v90_dil_measure_align(), on the whole file. */
function align(desc, rx, law) {
  const cycle = V.cycleLen(desc);
  const probe = Math.min(396, cycle, rx.length);
  if (probe < 12) return null;
  const gen = V.generate(law, desc, probe);
  const want = new Float64Array(probe);
  for (let i = 0; i < probe; i++) {
    want[i] = V.ucodeLevel(law, V.decomposeCodeword(law, gen[i]).ucode);
  }
  let best = -1, bestAt = -1;
  for (let at = 0; at + probe <= rx.length; at++) {
    let mx = 0, my = 0;
    for (let i = 0; i < probe; i++) {
      mx += want[i];
      my += V.ucodeLevel(law, V.decomposeCodeword(law, rx[at + i]).ucode);
    }
    mx /= probe; my /= probe;
    let sxy = 0, sxx = 0, syy = 0;
    for (let i = 0; i < probe; i++) {
      const dx = want[i] - mx;
      const dy = V.ucodeLevel(law, V.decomposeCodeword(law, rx[at + i]).ucode) - my;
      sxy += dx * dy; sxx += dx * dx; syy += dy * dy;
    }
    if (sxx <= 0 || syy <= 0) continue;
    const s = sxy / Math.sqrt(sxx * syy);
    if (s > best + 1e-9) { best = s; bestAt = at; }
  }
  return { score: best, offset: bestAt, probe };
}

function usage() {
  console.error('usage: v90_ja_scan.js scan <bits-file...>');
  console.error('       v90_ja_scan.js align <descriptor-hex> <g711-file...> [--alaw]');
  process.exit(2);
}

const argv = process.argv.slice(2);
const mode = argv.shift();

if (mode === 'scan') {
  if (!argv.length) usage();
  for (const f of argv) {
    const bits = loadBits(f);
    const hits = scanBits(bits);
    console.log(`${f}: ${bits.length} bits, ${hits.length} descriptor(s)`);
    for (const h of hits) {
      const info = describe(h.desc);
      console.log(`  start=${h.start} polarity=${h.polarity} bits=${info.bitLen}`);
      console.log(`  N=${info.n} LSP=${info.lsp} LTP=${info.ltp} H=${info.h} REF=${info.ref}`);
      console.log(`  cycle=${info.cycleSymbols}T (${info.cycleMs.toFixed(1)} ms) ` +
                  `distinct=${info.distinctUcodes} chords=${info.chords} probed=${info.intervalsProbed}`);
      console.log(`  train=${info.train}`);
      console.log(`  hex=${info.hex}`);
    }
  }
} else if (mode === 'align') {
  const law = argv.includes('--alaw') ? V.ALAW : V.ULAW;
  const files = argv.filter((a) => a !== '--alaw');
  const hex = files.shift();
  if (!hex || !files.length) usage();
  const bytes = V.hexToBits(hex);
  const r = V.parseDescriptor(bytes, bytes.length * 8);
  if (!r.ok) {
    console.error('descriptor rejected: ' + r.error);
    process.exit(1);
  }
  for (const f of files) {
    const rx = fs.readFileSync(f);
    const a = align(r.desc, rx, law);
    if (!a) { console.log(`${f}: too short to correlate`); continue; }
    console.log(`${f}: best ${a.score.toFixed(4)} at sample ${a.offset} ` +
                `(${(a.offset / 8).toFixed(1)} ms) over ${a.probe} symbols`);
  }
} else {
  usage();
}
