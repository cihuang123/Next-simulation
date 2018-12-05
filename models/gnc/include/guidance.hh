#ifndef __GUIDANCE_HH__
#define __GUIDANCE_HH__

/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Guidance Module On Board)
LIBRARY DEPENDENCY:
      ((../src/guidance.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include "Vehicle.hh"
#include "aux.hh"
#include "math_utility.hh"
#include "matrix/utility.hh"

class Guidance : public FH_module {
  Guidance();
  Guidance(const Guidance &other);

  virtual algorithm(dobule int_step);
  virtual init();

 private:
  arma::vec guidance_ltg(int &mprop, double int_step, double time_ltg);
  void guidance_ltg_tgo(double &tgo, int &nst, int &num_stages, arma::vec TAUN,
                        arma::vec VEXN, arma::vec BOTN, double delay_ignition,
                        double vgom, double amag1, double amin,
                        double time_ltg);
  void guidance_ltg_tgo(double &tgop, arma::vec &BURNTN, arma::vec &L_IGRLN,
                        arma::vec &TGON, double &l_igrl, int &nstmax,
                        double &tgo, int &nst, arma::vec &TAUN, arma::vec VEXN,
                        arma::vec BOTN, double delay_ignition, double vgom,
                        double amag1, double amin, double time_ltg,
                        int num_stages);
  void guidance_ltg_igrl(double &s_igrl, double &j_igrl, double &q_igrl,
                         double &h_igrl, double &p_igrl, double &j_over_l,
                         double &tlam, double &qprime, int nst, int nstmax,
                         arma::vec BURNTN, arma::vec L_IGRLN, arma::vec TGON,
                         arma::vec TAUN, arma::vec VEXN, double l_igrl,
                         double time_ltg);
  void guidance_ltg_trate(arma::vec &ULAM, arma::vec &LAMD, arma::vec &RGO,
                          int &ipas2, arma::vec VGO, double s_igrl,
                          double q_igrl, double j_over_l, double lamd_limit,
                          double vgom, double time_ltg, double tgo, double tgop,
                          arma::vec SDII, arma::vec SBIIC, arma::vec VBIIC,
                          arma::vec RBIAS, arma::vec UD, arma::vec UY,
                          arma::vec UZ, arma::vec &RGRAV);
  void guidance_ltg_trate_rtgo(arma::vec &RGO, arma::vec &RGRAV, double tgo,
                               double tgop, arma::vec SDII, arma::vec SBIIC,
                               arma::vec VBIIC, arma::vec RBIAS, arma::vec ULAM,
                               arma::vec UD, arma::vec UY, arma::vec UZ,
                               double s_igrl);
  void guidance_ltg_pdct(arma::vec &SPII, arma::vec &VPII, arma::vec &RGRAV,
                         arma::vec &RBIAS, arma::vec LAMD, arma::vec ULAM,
                         double l_igrl, double s_igrl, double j_igrl,
                         double q_igrl, double h_igrl, double p_igrl,
                         double j_over_l, double qprime, arma::vec SBIIC,
                         arma::vec VBIIC, arma::vec RGO, double tgo);
  void guidance_ltg_crct(arma::vec &SDII, arma::vec &UD, arma::vec &UY,
                         arma::vec &UZ, arma::vec &VMISS, arma::vec &VGO,
                         double dbi_desired, double dvbi_desired,
                         double thtvdx_desired, arma::vec SPII, arma::vec VPII,
                         arma::vec SBIIC, arma::vec VBIIC);

  int mguide;          /* ** (--)  Guidance modes */
  double time_ltg;     /* ** (s) Time since initiating LTG */
  VECTOR(UTBC, 3);     /* ** (--)  Commanded unit thrust vector in body coor */
  VECTOR(RBIAS, 3);    /* ** (m) Range-to-be-gained bias */
  int beco_flag;       /* **  (--) Boost engine cut-off flag */
  int inisw_flag;      /* ** (--) Flag to initialize */
  int skip_flag;       /* ** (--) Flag to delay output */
  int ipas_flag;       /* ** (--) Flag to initialize in '..._tgo()' */
  int ipas2_flag;      /* ** (--) Flag to initialize in '..._trat()' */
  int print_flag;      /* ** (--) Flag to cause print-out */
  int ltg_count;       /* ** (--) Counter of LTG guidance cycles */
  double ltg_step;     /* ** (s) LTG guidance time step - s */
  double dbi_desired;  /* ** (m) Desired orbital end position - m */
  double dvbi_desired; /* ** (m/s) Desired orbital end velocity - m/s */
  double thtvdx_desired; /* ** (deg) Desired orbital flight path angle - deg */
  double num_stages;     /* ** (--) Number of stages in boost phase */
  double delay_ignition; /* ** (s) Delay of motor ignition after staging - s */
  double amin;         /* ** (m/s2) Minimum longitudinal acceleration - m/s^2 */
  double char_time1;   /* ** (s) Characteristic time 'tau' of stage 1 - s */
  double char_time2;   /* ** (s) Characteristic time 'tau' of stage 2 - s */
  double char_time3;   /* ** (s) Characteristic time 'tau' of stage 3 - s */
  double exhaust_vel1; /* ** (m/s) Exhaust velocity of stage 1 - m/s */
  double exhaust_vel2; /* ** (m/s) Exhaust velocity of stage 2 - m/s */
  double exhaust_vel3; /* ** (m/s) Exhaust velocity of stage 3 - m/s */
  double burnout_epoch1; /* ** (m/s) Burn out of stage 1 at 'time_ltg' - s */
  double burnout_epoch2; /* ** (m/s) Burn out of stage 2 at 'time_ltg' - s */
  double burnout_epoch3; /* ** (m/s) Burn out of stage 3 at 'time_ltg' - s */
  double lamd_limit;     /* ** (1/s) Limiter on 'lamd' - 1/s */
  VECTOR(RGRAV, 3);      /* ** (m) Postion loss due to gravity - m */
  VECTOR(RGO, 3);        /* ** (m) Range-to-go vector - m */
  VECTOR(VGO, 3);        /* ** (m/s)  Velocity still to be gained - m/s */
  VECTOR(SDII, 3);       /* ** (m) Desired inertial position - m */
  VECTOR(UD, 3);         /* ** (--) Unit vector of SPII and SDII */
  VECTOR(UY, 3);         /* ** (--) Unit vector normal to traj plane */
  VECTOR(UZ, 3);   /* ** (--) Unit vector in traj plane, normal to SBBI_D */
  double vgom;     /* ** (m/s) Velocity to be gained magnitude - m/s */
  double tgo;      /* ** (s) Time to go to desired end state - s */
  int nst;         /* ** (--) N-th stage number */
  VECTOR(ULAM, 3); /* ** (--) Unit thrust vector in VGO direction */
  VECTOR(LAMD, 3); /* ** (1/s) Thrust vector turning rate - 1/s */
  VECTOR(UTIC, 3); /* ** (--) Commanded unit thrust vector in inertial coor */
  int nstmax;      /* ** (--) num of stages needed to meet end state - ND */
  double lamd;     /* ** (1/s) Magnitude of LAMD - 1/s */
  double
      dpd; /* ** (m) Distance of the predicted from the desired end-point - m */
  double dbd; /* ** (m) Distance of vehicle from the desired end-point - m */
  double ddb; /* ** (m) Position error at BECO - m */
  double
      dvdb; /* ** (m/s) Distance of vehicle from the desired end-point - m/s */
  double thtvddbx;  /* ** (d) Angle error at BECO - deg */
  double alphacomx; /* ** (d) Alpha command - deg */
  double betacomx;  /* ** (d) Beta command - deg */
}
#endif  // __GUIDANCE_HH__