/* V.250 §§6.3.1.3–4, 6.3.3–4; Si3056 §5.17. Host pulse schedule.
 * 10 pps, 60 ms break / 40 ms make, 600 ms interdigit; zero = ten pulses.
 * Build/validate before seizing the line. Only the control thread writes USB. */
#ifndef APPLE_MODEM_DIAL_H
#define APPLE_MODEM_DIAL_H
#include <ctype.h>
#include <stddef.h>
enum apple_dial_kind { APPLE_DIAL_BREAK, APPLE_DIAL_MAKE, APPLE_DIAL_WAIT, APPLE_DIAL_TONE };
struct apple_dial_step { enum apple_dial_kind kind; unsigned ms; char digit; };
struct apple_dial_plan { struct apple_dial_step step[2400]; unsigned count, pulses, digits; };
static inline int apple_dial_build(struct apple_dial_plan *plan, const char *text,
                                  unsigned tone_ms, unsigned gap_ms)
{
    *plan = (struct apple_dial_plan){0};
    int pulse = 0;
    for (const char *p = text; *p; p++) {
        char c = (char)toupper((unsigned char)*p);
        if (c == 'P' || c == 'T') { pulse = c == 'P'; continue; }
        if (c == ' ' || c == '-') continue;
        if (plan->count+22 >= sizeof(plan->step)/sizeof(plan->step[0])) return -1;
        if (c == ',') { plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_WAIT,500,0}; continue; }
        if (c < '0' || c > '9') {
            if (pulse || !(c == '*' || c == '#' || (c >= 'A' && c <= 'D'))) return -1;
        }
        plan->digits++;
        if (pulse) {
            unsigned n = c == '0' ? 10 : (unsigned)(c-'0');
            plan->pulses += n;
            for (unsigned i = 0; i < n; i++) {
                plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_BREAK,60,c};
                plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_MAKE,40,c};
            }
            plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_WAIT,600,c};
        } else {
            plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_TONE,tone_ms,c};
            plan->step[plan->count++] = (struct apple_dial_step){APPLE_DIAL_WAIT,gap_ms,c};
        }
    }
    return plan->digits && tone_ms && gap_ms ? 0 : -1;
}
#endif
