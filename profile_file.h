/*
 * profile_file.h — the --profile configuration file (AT&W stored profiles)
 *
 * A stored profile is a list of SETTINGS, each one line of the Cisco-style
 * form and each the spelling of exactly one AT command:
 *
 *     echo | no echo                E1 / E0      (likewise quiet, verbose)
 *     result-codes N                XN
 *     dcd N                         &CN
 *     dtr N                         &DN
 *     connect-suffix N              &AN          (Courier CONNECT .../ARQ/...)
 *     dial tone | dial pulse        T / P
 *     s-register N V                SN=V
 *     +NAME value                   +NAME=value  (+MS, +ES, +VCID, ... any)
 *     number N "digits"             +ASTO=N,"digits"
 *     at COMMANDS                   COMMANDS, passed through as written
 *
 * Nothing here validates a value: the setting is turned into its AT command
 * and the interpreter accepts or refuses it, so the file can never accept a
 * setting the DTE could not have typed (data_interface.c replays them).
 *
 * Two file syntaxes carry those settings, chosen on writing by the file name
 * (".json" is JSON, anything else Cisco-style) and on reading by content:
 *
 *   Cisco-style                     JSON
 *     ! comment                       { "power-on-profile": 0,
 *     power-on-profile 0                "numbers": { "2": "555" },
 *     number 2 "555"                    "profiles": { "0": {
 *     profile 0                           "echo": false, "result-codes": 4,
 *      no echo                            "s-registers": { "0": 2 },
 *      result-codes 4                     "+MS": "V90,1,300,0,300,0",
 *      s-register 0 2                     "at": [ "&K3" ] } } }
 *      +MS V90,1,300,0,300,0
 *     end
 *
 * The file written by the first --profile implementation, AT command lines
 * with "#" comments, is still read, as profile 0.
 */
#ifndef PROFILE_FILE_H
#define PROFILE_FILE_H

#include <stdbool.h>
#include <stddef.h>

#define PF_PROFILES 10      /* profile numbers the file syntax can carry */
#define PF_LINES    64      /* settings per profile */
#define PF_LINE     200     /* one setting */

typedef struct {
    bool present;
    int n;
    char line[PF_LINES][PF_LINE];
    int src[PF_LINES];      /* line in the file it came from (0 = none) */
} pf_profile_t;

typedef struct {
    int power_on;           /* -1: the file does not say */
    pf_profile_t global;    /* settings outside any profile: the numbers */
    pf_profile_t profile[PF_PROFILES];
} pf_doc_t;

void pf_doc_init(pf_doc_t *doc);

/* Append one setting (printf-style) to a profile; false when it is full. */
bool pf_add(pf_profile_t *p, int src, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* Parse a whole file.  0 on success, else -1 with a message ("line 7: ...")
 * in err. */
int pf_parse(const char *text, pf_doc_t *doc, char *err, size_t errlen);

/* Render a document.  json selects the syntax.  Returns the length, or -1 if
 * it did not fit. */
int pf_render(const pf_doc_t *doc, bool json, char *out, size_t cap);

/* True when a file name asks for JSON (it ends ".json"). */
bool pf_path_is_json(const char *path);

/* The AT command (without "AT") one setting stands for.  0, or -1 with the
 * reason in out when the keyword is not a setting. */
int pf_setting_to_at(const char *setting, char *out, size_t cap);

#endif
