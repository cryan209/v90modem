# Artifact storage

## Layout

The packaged browser demo and its offline test also use the retained
`reference-data/artifacts/modem-lab.html` entry point.

`artifacts/` and `captures/` are disposable, gitignored output directories for
new experiments. Put durable dependencies in `reference-data/`, update their
consumers, and record why they are needed in `reference-data/index.tsv`.

## Retention rules (2026-10-30)

A citation used to oblige us to keep the whole run it named, so a doc line
saying "see `artifacts/foo/`" quietly made gigabytes permanent. At this date
only 19 of 419 retained datasets (843 MB of 7 GB) were named by a test. The
rest were kept because a doc or code comment mentioned them. From now on:

1. **Cite the finding, not the run.** A doc, comment or commit message states
   the measured value and how it was measured, inline. A run path may follow
   as provenance. It does not oblige anyone to keep that run, and readers
   must expect it to be gone.
2. **Only a consumer makes data permanent.** Data that a test, tool or the
   makefile reads goes in `reference-data/`, cut to the minimum window the
   consumer needs (one tap segment, not the whole run), with a row in
   `index.tsv` naming that consumer. Track it if it is a regression fixture.
3. **`artifacts/` and `captures/` expire.** `make expire-artifacts` lists runs
   untouched for 14 days. `make expire-artifacts EXPIRE_ARGS=--delete` removes
   them. A `KEEP` file at the top of a run (with the reason in it) or a git
   worktree inside it exempts the run. A `KEEP` is a stopgap; promote the data
   under rule 2 or let it go.
4. **Enforced in `make test-fast`.** `tools/check_reference_data.py` fails
   when tracked code, tests, tools or the makefile (comments excluded) read a
   `reference-data/` dataset that has no `index.tsv` row, or read a
   disposable `artifacts/`/`captures/` run. `--docs` lists runs that docs
   cite, for review. A tool allowed to break when its input expires (rig
   one-offs) goes in the checker's `ALLOWED` set.

The corpus retained by the 2026-10-18 cleanup below predates these rules and
stays until its datasets are reviewed. A dataset that only a doc cites can be
removed once that doc carries the finding inline.

On 2026-10-30, with the user's approval, we deleted 110 retained runs (801 MB)
and their `index.tsv` rows. None were tracked or read by code, and the only
citations were wildcards in docs or code comments. The families were
`x2-20261019-*`, `x2-s39q-*`, `x2-headroom-full-*`, `x2-higher-*`,
`x2-rx-improve-*`, `x2-ds-pace-*`, `x2-host-analog-20261006-exact`,
`rf-tower-*` (39 calls), `rf-te-ab-*`, `v34-rate-20260822T*`,
`trn1d-long-221035Z`, `trn1d-long-ab-222757Z`, `lossretrain-ab-004426Z`,
`gate-ab-28800-073139Z` and `eicon-v90a-trn1d-ab-native{3,4}-r1-20261009`.
We also deleted the unindexed, uncited `rf-echo-default-20261022`. Docs that
cite these runs now point at provenance that no longer exists; their stated
results stand as written.

## Retained corpus

`reference-data/artifacts/` and `reference-data/captures/` hold retained input
recordings, fixtures and evidence cited by source or documentation. Their
relative run layouts are unchanged. Existing regression fixtures are tracked;
other retained data remains local and gitignored. To add a regression fixture,
force-add the specific files, never the entire retained corpus.

The index records paths, file counts, original sizes and reasons for retention.
Future audits must scan vendored source as well as project source and docs,
expand wildcard citations, and inspect retained run notes before deciding what
to delete. Do not treat an untracked file as disposable just because it is absent from Git.
Hardware recordings cannot necessarily be reproduced. Historical docs may also
refer to recordings absent before this cleanup; the index lists what actually
survived locally.

## 2026-10-18 cleanup

Moved and checksum-verified 49,185 files (6,667,559,289 bytes) across 349 retained
datasets. Kept all 49 tracked files, literal and wildcard citations in project source,
tests, tools and docs, and references found transitively in retained run notes.
The branch-cleanup Git bundle stays because some snapshot tips are absent from
the local object database.

Deleted 10,562 untracked scratch and historical files (5,171,187,130 bytes). No audio
was transcoded, resampled or otherwise changed. Consumer paths now point to
`reference-data/`; fresh captures still go to `artifacts/` or `captures/`.

The local ignored `reference-data/inventory.json` records every original file,
its size and modification time, the keep/delete decision, and pre-move SHA-256
hashes for retained files. All retained bytes were checked after the move and
before deletion. Textual run notes were then updated to refer to the new paths.
This inventory is an audit record, not a backup of deleted data.

## Nine historical datasets confirmed disposable

The initial reference scan incorrectly excluded vendored source. A subsequent
scan found the following deleted runs cited in SpanDSP comments. These were
untracked historical evidence, not inputs to the regression suite. They should
have been kept. Recovery from tower was attempted on 2026-10-18, but SSH timed
out. The user then confirmed these historical runs are not needed and do not
require recovery. No regression fixtures were affected.

- `artifacts/gate-ab-31200-051616Z` — cited by `spandsp-master/src/v34rx.c`
- `artifacts/rasfinder-info0a-200334Z` — cited by `spandsp-master/src/v34rx.c`
- `artifacts/rasfinder-info0a2-201220Z` — cited by `spandsp-master/src/v34rx.c`
- `artifacts/rasfinder-p4-201857Z` — cited by `spandsp-master/src/v34rx.c`, `spandsp-master/src/v34tx.c`
- `artifacts/retrain-live-c2` — cited by `spandsp-master/src/spandsp/private/v34.h`, `spandsp-master/src/v34tx.c`
- `artifacts/rf-0929s-p10-3` — cited by `spandsp-master/src/v34rx.c`
- `artifacts/rf-v34-h1` — cited by `spandsp-master/src/v34tx.c`
- `artifacts/rf-v34-k1` — cited by `spandsp-master/src/v34tx.c`
- `artifacts/rf-v34-q1` — cited by `spandsp-master/src/v34tx.c`

## Validation

All 49 tracked files were found at their new locations; non-note fixture files
match HEAD byte for byte. All six fast-tier rows that exercise moved fixture
paths passed, including the Apple and RasFinder replays, analogue receive suite
and V.92 line capture test. The full fast tier returned 485 passes and 96
failures (DSP assertions and a sandbox-denied local socket among the failures);
it is not a clean regression baseline. Logs are in local `test-logs/`.

After the final path updates and rebuild, the focused fixture tier again passed
all six rows (logs: `test-logs/artifact-relocation/`). The optional
`make v90-apple-line-test` check could not start because the selected Python
interpreter does not have NumPy installed. Git history and the nearby local
worktree/archive locations checked did not contain copies of the nine missing
datasets. No Time Machine local snapshots were listed.

The packaged browser offline test (`node web/modem-lab/test-offline.mjs`) also
passed after its retained HTML path and the packager output were updated.
