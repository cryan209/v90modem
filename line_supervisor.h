/*
 * line_supervisor.h -- run N modem lines as N worker processes.
 *
 * The engine holds one call's state in file-scope statics, so a line is a
 * process: `sip_v90_modem --lines N ...` re-executes itself N times, one
 * worker per line, each with its own SIP port, RTP port range, console and
 * stored-profile file, and supervises them (docs/multi_line.md).
 */
#ifndef LINE_SUPERVISOR_H
#define LINE_SUPERVISOR_H

#include <stdbool.h>

#define LINE_SUPERVISOR_MAX_LINES 64

/* Find --lines / --recycle in argv.  Returns the line count (0 when
 * --lines is absent, -1 on a bad value) and sets *recycle. */
int line_supervisor_parse(int argc, char **argv, bool *recycle);

/* Run the workers until SIGINT/SIGTERM; returns the process exit code.
 * argv is the supervisor's own command line, --lines and --recycle
 * included; each worker gets the rest plus its per-line overrides. */
int line_supervisor_run(int argc, char **argv, int lines, bool recycle);

#endif
