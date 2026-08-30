#ifndef PORT_DATA_RX_H
#define PORT_DATA_RX_H

typedef struct {
    float derot;         /* accumulated derotator phase, radians */
    float derot_rate;    /* B1-measured residual carrier, rad/symbol */
    int rotation;        /* 0..3, quarter turns */
    int conjugate;
    float err_ema;       /* 256-symbol mean square distance to the lattice */
    float err_fast;      /* 16-symbol, for the adaptive gates */
    float baseline;      /* what this call settled at just after B1 */
    long count;
    double err_sum;
    double pow_sum;
} data_rx_t;

void  data_rx_init(data_rx_t *s, float derot_rate_rad, int rotation, int conjugate);
float data_rx_put(data_rx_t *s, float re, float im, float *out_re, float *out_im);
int   data_rx_healthy(const data_rx_t *s, float slack);
void  data_rx_set_baseline(data_rx_t *s);

#endif
