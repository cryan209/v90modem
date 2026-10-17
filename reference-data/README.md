# Retained reference data

Test fixtures, cited hardware recordings and a unique Git backup live here.
New runs belong in `artifacts/` or `captures/`.

- `index.tsv` lists every retained dataset and the references that justify it.
- `artifacts/` and `captures/` preserve original relative layouts under this root.
- `inventory.json` is the local 2026-10-18 cleanup inventory, including original
  paths, deleted-file metadata and SHA-256 hashes taken before moving captures.
  Run notes were updated afterward; their original hashes document that change.

The existing 49 tracked fixture files stay tracked at their new paths. Bulk
historical evidence and the inventory are local and ignored by Git; cloning
this repository does not recreate them. See `../docs/artifact_storage.md`.
