/* Modulate a symbol file through SpanDSP's V.34 transmitter exactly as
 * k56flex_client_splice does, for tools/k56flex_lsdemod.py's basis.
 *
 *   k56flex_modulate SYMS.f32 OUT.s16 BAUD HIGH PR PE NSAMPLES
 *
 * SYMS.f32 is complex float32 (re, im) pairs, one per symbol from the first
 * symbol the modulator asks for; zeros after the file ends.  BAUD is the
 * V.34 Table 1 code, PR/PE the INFO1a power reduction and pre-emphasis.
 * OUT is NSAMPLES of native int16 at 8 kHz. */
#include <spandsp.h>
#include <stdio.h>
#include <stdlib.h>

static float *syms;
static long nsyms, pos;

static int get_bit(void *user) { (void) user; return 1; }
static void put_bit(void *user, int bit) { (void) user; (void) bit; }

static void source(void *user, float *re, float *im)
{
    (void) user;
    if (pos < nsyms) {
        *re = syms[2*pos];
        *im = syms[2*pos + 1];
    } else {
        *re = *im = 0.0f;
    }
    pos++;
}

int main(int argc, char **argv)
{
    FILE *f, *o;
    long n, total;
    v34_state_t *v34;
    int16_t amp[160];

    if (argc != 8) {
        fprintf(stderr, "usage: %s SYMS.f32 OUT.s16 BAUD HIGH PR PE NSAMPLES\n", argv[0]);
        return 2;
    }
    if (!(f = fopen(argv[1], "rb"))  ||  !(o = fopen(argv[2], "wb"))) {
        perror("open");
        return 2;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    syms = malloc(n > 0 ? (size_t) n : 1);
    if (!syms  ||  fread(syms, 1, (size_t) n, f) != (size_t) n)
        return 2;
    fclose(f);
    nsyms = n/8;
    total = atol(argv[7]);
    v34 = v34_init(NULL, 3200, 31200, true, true, get_bit, NULL, put_bit, NULL);
    v34_tx_phase3_from_info1a(v34, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]));
    v34_tx_start_external_symbols(v34, atoi(argv[3]), atoi(argv[4]), source, NULL);
    for (n = 0;  n < total;  n += 160) {
        v34_tx(v34, amp, 160);
        fwrite(amp, 2, 160, o);
    }
    fclose(o);
    v34_free(v34);
    return 0;
}
