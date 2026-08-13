/*
 * V.34 duplex MP rate-negotiation tests.
 *
 * ITU-T V.34 (10/96) 11.4.1.1.4/11.4.1.2.4: each direction selects the
 * highest rate enabled by both MP masks and bounded by both MP maxima.
 * Table 20 bit 50 permits unequal rates only when both modems set it.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <spandsp.h>

static void require_rates(const char *name,
                          v34_mp_rate_offer_t local,
                          v34_mp_rate_offer_t remote,
                          int expected_a_to_c,
                          int expected_c_to_a)
{
    int a_to_c = 0;
    int c_to_a = 0;

    if (v34_negotiate_mp_rates(&local, &remote, &a_to_c, &c_to_a) != 0
        || a_to_c != expected_a_to_c
        || c_to_a != expected_c_to_a) {
        fprintf(stderr,
                "FAIL %s: got a2c=%d c2a=%d, expected a2c=%d c2a=%d\n",
                name, a_to_c, c_to_a, expected_a_to_c, expected_c_to_a);
        exit(1);
    }
}

int main(void)
{
    v34_mp_rate_offer_t local = {14, 10, 0x3FFF, true};
    v34_mp_rate_offer_t remote = {12, 11, 0x3FFF, true};
    v34_mp_rate_offer_t none = {14, 14, 1U << 13, true};
    int a_to_c;
    int c_to_a;

    require_rates("asymmetric maxima", local, remote, 12, 10);

    remote.asymmetric_rates_allowed = false;
    require_rates("bit 50 requires bilateral enable", local, remote, 10, 10);

    local = (v34_mp_rate_offer_t){14, 9, (1U << 9) | (1U << 7), true};
    remote = (v34_mp_rate_offer_t){13, 12, (1U << 9) | (1U << 7), true};
    require_rates("sparse mask backs off independently", local, remote, 10, 8);

    /* Table 20 bit 49 is reserved; it cannot create a usable rate. */
    local = (v34_mp_rate_offer_t){14, 14, 1U << 14, true};
    remote = local;
    if (v34_negotiate_mp_rates(&local, &remote, &a_to_c, &c_to_a) == 0) {
        fprintf(stderr, "FAIL reserved mask bit accepted as a data rate\n");
        return 1;
    }

    local = (v34_mp_rate_offer_t){14, 14, 1U << 12, true};
    if (v34_negotiate_mp_rates(&local, &none, &a_to_c, &c_to_a) == 0) {
        fprintf(stderr, "FAIL disjoint masks produced a data rate\n");
        return 1;
    }

    /* Exhaust all maxima with a full mask. */
    local.signalling_rate_mask = 0x3FFF;
    remote.signalling_rate_mask = 0x3FFF;
    local.asymmetric_rates_allowed = true;
    remote.asymmetric_rates_allowed = true;
    for (int a = 1; a <= 14; a++) {
        for (int c = 1; c <= 14; c++) {
            local.max_rate_a_to_c = a;
            local.max_rate_c_to_a = c;
            remote.max_rate_a_to_c = 14;
            remote.max_rate_c_to_a = 14;
            require_rates("full-mask matrix", local, remote, a, c);
        }
    }

    printf("v34_mp_test: OK\n");
    return 0;
}
