# Artifact storage

## Layout

The packaged browser demo and its offline test also use the retained
`reference-data/artifacts/modem-lab.html` entry point.

`artifacts/` and `captures/` are disposable, gitignored output directories for
new experiments. Put durable dependencies in `reference-data/`, update their
consumers, and record why they are needed in `reference-data/index.tsv`.

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
