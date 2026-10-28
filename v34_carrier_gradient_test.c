/* V.90 8.5.2/9.4.2.5: decision-directed adaptation after carrier recovery.
 * Compare the tap update with an independent finite-difference loss gradient.
 */
#include <math.h>
#include <stdio.h>
#include "spandsp-master/src/v34_carrier.h"
static double loss(double wr,double wi,double xr,double xi,double phase,
                   double target_re,double target_im)
{
    double zr=wr*xr-wi*xi, zi=wr*xi+wi*xr;
    double er=target_re-(zr*cos(phase)+zi*sin(phase));
    double ei=target_im-(zi*cos(phase)-zr*sin(phase));
    return er*er+ei*ei;
}
static double dispersion(double wr,double wi,double xr,double xi)
{
    double zr=wr*xr-wi*xi, zi=wr*xi+wi*xr;
    double residual=zr*zr+zi*zi-2.0;
    return residual*residual/2.0;
}
int main(void)
{
    const double eps=1e-5;
    for(int n=0;n<65;n++) {
        double phase=-3.141592653589793+6.283185307179586*n/64;
        double wr=.73,wi=-.21,xr=1.31,xi=-.47,tr=1,ti=-1;
        double zr=wr*xr-wi*xi,zi=wr*xi+wi*xr;
        float er=(float)(tr-zr*cos(phase)-zi*sin(phase));
        float ei=(float)(ti-zi*cos(phase)+zr*sin(phase)),rr,ri;
        v34_carrier_error_to_input((float)phase,er,ei,&rr,&ri);
        double gr=rr*xr+ri*xi,gi=ri*xr-rr*xi;
        double dr=(loss(wr+eps,wi,xr,xi,phase,tr,ti)-loss(wr-eps,wi,xr,xi,phase,tr,ti))/(2*eps);
        double di=(loss(wr,wi+eps,xr,xi,phase,tr,ti)-loss(wr,wi-eps,xr,xi,phase,tr,ti))/(2*eps);
        if(fabs(dr+2*gr)>2e-6 || fabs(di+2*gi)>2e-6) {
            fprintf(stderr,"gradient disagrees at phase %.5f: (%g,%g) vs (%g,%g)\n",phase,dr,di,-2*gr,-2*gi);return 1;
        }
        /* The blind radial cost is rotation invariant, but using its
           rotated output against raw input also gives the wrong gradient. */
        float yr=(float)(zr*cos(phase)+zi*sin(phase));
        float yi=(float)(zi*cos(phase)-zr*sin(phase));
        v34_carrier_error_to_input((float)phase,yr,yi,&rr,&ri);
        double radial=(yr*yr+yi*yi-2.0)/2.0;
        double cgr=radial*(rr*xr+ri*xi),cgi=radial*(ri*xr-rr*xi);
        double cdr=(dispersion(wr+eps,wi,xr,xi)-dispersion(wr-eps,wi,xr,xi))/(2*eps);
        double cdi=(dispersion(wr,wi+eps,xr,xi)-dispersion(wr,wi-eps,xr,xi))/(2*eps);
        if(fabs(cdr-4*cgr)>3e-6 || fabs(cdi-4*cgi)>3e-6) {
            fprintf(stderr,"radial gradient disagrees at phase %.5f\n",phase);return 1;
        }
        if(loss(wr+.05*gr,wi+.05*gi,xr,xi,phase,tr,ti)>=loss(wr,wi,xr,xi,phase,tr,ti)) {
            fprintf(stderr,"update increased loss at phase %.5f\n",phase);return 1;
        }
    }
    puts("carrier/equalizer gradient: 65 phases agree with finite differences and reduce loss");
    return 0;
}
