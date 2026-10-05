# Stored profiles and the `--profile` file

Hayes-style stored configuration: `AT&W<n>` stores the active configuration
as profile n (0 or 1), `ATZ<n>` restores the factory configuration and then
profile n over it, `AT&Y<n>` picks the profile restored at power-on, `AT&F`
is factory only, and `AT&V` shows the active configuration followed by the
stored profiles in the file's own syntax.

A profile holds E/Q/V/X/&C/&D/&A, tone/pulse, S0/S2-S8/S10/S12, `+VCID`, `+MS`
and every V.250 parameter (`+MR +ER +DR +ES +DS +DS44 +EB +EFCS +ETBM +EWIND
+EFRAM +IPR +ICF +IFC +ILRR +MSC`).  The stored dial strings (`&Z`, `+ASTO`)
are **not** part of a profile, as on a Hayes modem: Z and &F leave them alone,
and `&W`/`&Y` save them, as they are at that moment, beside the profiles.

## The file

`sip_v90_modem --profile <file>` loads the file at start-up and `&W`/`&Y`
rewrite it atomically (temp file + rename).  An `&W` or `&Y` that cannot be
written answers ERROR and changes nothing in memory.  Without `--profile` the
profiles last until the process exits.

Every setting is the spelling of exactly one AT command, and loading replays
it through the AT interpreter -- so the file can never hold a value the DTE
could not have typed, and validation lives in one place.  Each profile is
built as `AT&F`, then its settings, then snapshotted, exactly as
`AT&F...&Wn` would have made it.  A setting the interpreter refuses is logged
(`[DI] <file> line N: ... refused; ignored`) and skipped.  Result codes are
forced on while replaying, so a `quiet` profile cannot hide its own errors.

A file that does not **parse** (unknown keyword, bad JSON) is not loaded, and
`&W`/`&Y` then answer ERROR rather than overwrite what someone wrote; fix it
and restart.

The syntax is chosen by name when writing (`.json` is JSON, anything else is
Cisco-style) and by content when reading (`{` is JSON).  The file written by
the first `--profile` implementation, plain `AT...` lines with `#` comments,
is still read, as profile 0, and is rewritten Cisco-style by the next `&W`.

### Settings

| Setting                 | AT command          |
|-------------------------|---------------------|
| `echo` / `no echo`      | `E1` / `E0`         |
| `quiet` / `no quiet`    | `Q1` / `Q0`         |
| `verbose` / `no verbose`| `V1` / `V0`         |
| `result-codes N`        | `XN`                |
| `dcd N`, `dtr N`        | `&CN`, `&DN`        |
| `connect-suffix N`      | `&AN` (Courier: CONNECT .../ARQ/V34/LAPM/V42BIS) |
| `dial tone` / `pulse`   | `T` / `P`           |
| `s-register N V`        | `SN=V`              |
| `+NAME value`           | `+NAME=value` (any extended command) |
| `number N "digits"`     | `+ASTO=N,"digits"` (global, not per profile) |
| `at COMMANDS`           | sent as written (escape hatch) |

### Cisco-style

```
! comment lines start with ! (or #)
power-on-profile 0
!
number 2 "555"
!
profile 0
 no echo
 result-codes 4
 s-register 0 2
 +MS V90,1,0,0,0,0
 +ES 3,0,2
!
profile 1
 +MS V34,1,0,0,0,0
!
end
```

### JSON

```json
{
  "power-on-profile": 0,
  "numbers": { "2": "555" },
  "profiles": {
    "0": {
      "echo": false,
      "result-codes": 4,
      "+MS": "V90,1,0,0,0,0",
      "s-registers": { "0": 2 },
      "at": [ "&K3" ]
    }
  }
}
```

Booleans are the flags, whole numbers and strings are `key value`,
`s-registers` is the one object and `at` the one list.  Keys starting `_`
(such as the `_comment` the writer emits) are ignored.  Dial strings are
plain JSON strings; a `"` in one becomes V.250's `\22` on the AT side.

Code: `profile_file.c` (syntax only), `data_interface.c` (snapshot, apply,
replay); tests in `console_test` (`test_profile_file`, `test_profile`).
