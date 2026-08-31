/* The fixed-point least-squares solve against the float one, on the REAL 42x42
 * systems a live acquisition produced (V90_T3_SOLVE_MATRIX dumps).
 *
 * What matters is not the solution vector matching to many digits -- it is the
 * RESIDUAL |A.x - b|, because that is what the equalizer inherits.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "v34_fixed.h"

#define NMAX 64

static double A[NMAX*NMAX], B[NMAX], A0[NMAX*NMAX], B0[NMAX], Xf[NMAX];
static int64_t Ai[NMAX*NMAX], Bi[NMAX], Xi[NMAX];

static int float_solve(double *a, double *b, double *x, int n)
{
    for (int i = 0; i < n; i++) {
        int p = i; double best = fabs(a[i*n+i]);
        for (int j = i+1; j < n; j++) { double v = fabs(a[j*n+i]); if (v > best) { best = v; p = j; } }
        if (!isfinite(best) || best < 1e-12) return 0;
        if (p != i) { for (int k = i; k < n; k++) { double t=a[i*n+k]; a[i*n+k]=a[p*n+k]; a[p*n+k]=t; }
                      double t=b[i]; b[i]=b[p]; b[p]=t; }
        double d = a[i*n+i];
        for (int k = i; k < n; k++) a[i*n+k] /= d;
        b[i] /= d;
        for (int j = 0; j < n; j++) {
            if (j == i) continue;
            double f = a[j*n+i];
            if (f == 0.0) continue;
            for (int k = i; k < n; k++) a[j*n+k] -= f*a[i*n+k];
            b[j] -= f*b[i];
        }
    }
    for (int i = 0; i < n; i++) x[i] = b[i];
    return 1;
}

static double residual(const double *a, const double *b, const double *x, int n)
{
    double r = 0.0, bn = 0.0;
    for (int i = 0; i < n; i++) {
        double s = 0.0;
        for (int j = 0; j < n; j++) s += a[i*n+j]*x[j];
        r += (s - b[i])*(s - b[i]);
        bn += b[i]*b[i];
    }
    return bn > 0 ? sqrt(r/bn) : sqrt(r);
}

int main(int argc, char *argv[])
{
    for (int f = 1; f < argc; f++) {
        FILE *fp = fopen(argv[f], "r");
        int n;
        if (!fp) { perror(argv[f]); continue; }
        if (fscanf(fp, "%d", &n) != 1 || n > NMAX) { fclose(fp); continue; }
        for (int i = 0; i < n*n; i++) if (fscanf(fp, "%lf", &A0[i]) != 1) { }
        for (int i = 0; i < n; i++)   if (fscanf(fp, "%lf", &B0[i]) != 1) { }
        fclose(fp);

        /* block scale: one shift for the whole system */
        double mx = 0.0;
        for (int i = 0; i < n*n; i++) if (fabs(A0[i]) > mx) mx = fabs(A0[i]);
        for (int i = 0; i < n; i++)   if (fabs(B0[i]) > mx) mx = fabs(B0[i]);
        double sc = mx > 0 ? 1.0/mx : 1.0;

        memcpy(A, A0, sizeof(double)*n*n); memcpy(B, B0, sizeof(double)*n);
        int okf = float_solve(A, B, Xf, n);

        for (int i = 0; i < n*n; i++)
            Ai[i] = (int64_t) llround(A0[i]*sc*(double)((int64_t)1 << V34_FX_SOLVE_SHIFT));
        for (int i = 0; i < n; i++)
            Bi[i] = (int64_t) llround(B0[i]*sc*(double)((int64_t)1 << V34_FX_SOLVE_SHIFT));
        int oki = v34_fx_solve(Ai, Bi, Xi, n);

        double Xx[NMAX];
        for (int i = 0; i < n; i++)
            Xx[i] = (double) Xi[i] / (double)((int64_t)1 << V34_FX_SOLVE_SHIFT);

        double rf = okf ? residual(A0, B0, Xf, n) : -1.0;
        double rx = oki ? residual(A0, B0, Xx, n) : -1.0;
        double dx = 0.0, xn = 0.0;
        for (int i = 0; i < n; i++) { dx += (Xx[i]-Xf[i])*(Xx[i]-Xf[i]); xn += Xf[i]*Xf[i]; }

        printf("%s  n=%d\n", argv[f], n);
        printf("   float ok=%d  relative residual %.4e\n", okf, rf);
        printf("   fixed ok=%d  relative residual %.4e\n", oki, rx);
        printf("   |x_fixed - x_float| / |x_float| = %.4e\n", xn > 0 ? sqrt(dx/xn) : 0.0);
    }
    return 0;
}
