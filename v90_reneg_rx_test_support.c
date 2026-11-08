/* Test-only inspection of the vendored receiver's actual §9.6 lifecycle.
 * No production caller, waveform synthesis or relaxed acquisition checks. */
#include <stdbool.h>
#include <stdint.h>
#include "spandsp.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/v34.h"

void v90_reneg_test_seed_old_data(v34_state_t *s);
bool v90_reneg_test_fresh_cp(const v34_state_t *s);

void v90_reneg_test_seed_old_data(v34_state_t *s)
{
    s->rx.v90_t3_active = true;
    s->rx.v90_t3_acquired = true;
    s->rx.v90_t3_prepared = true;
}

bool v90_reneg_test_fresh_cp(const v34_state_t *s)
{
    return !s->rx.v90_t3_active && !s->rx.v90_t3_acquired
        && !s->rx.v90_t3_prepared && s->rx.stage == V34_RX_STAGE_V90_CP
        && s->rx.v90_cp_stream == 1;
}
