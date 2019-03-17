#ifndef __Rocket_Flight_DM_HH__
#define __Rocket_Flight_DM_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Rocket Flgiht Dynamics Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Rocket_Flight_DM.cpp)
       (../../math/src/nrutil.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include <armadillo>
#include "Module.hh"
#include "Time_management.hh"
#include "aux.hh"
#include "global_constants.hh"
#include "icf_trx_ctrl.h"
#include "simgen_remote.h"

class Rocket_Flight_DM : public Dynamics {
  TRICK_INTERFACE(Rocket_Flight_DM);

 public:
  Rocket_Flight_DM(Data_exchang &input);
  Rocket_Flight_DM(const Rocket_Flight_DM &other);
  Rocket_Flight_DM &operator=(const Rocket_Flight_DM &other);
  struct icf_ctrlblk_t *dm_icf_info_hook;
  int enqueue_to_simgen_buffer(struct icf_ctrlblk_t *C, double ext_porlation);
  int stand_still_motion_data(struct icf_ctrlblk_t *C, double ext_porlation);

  virtual void init();
  virtual void algorithm(double int_step);
  void load_location(double lonx_in, double latx_in, double alt_in);
  void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);
  void load_angle(double yaw, double roll, double pitch);
  void load_angular_velocity(double ppx_in, double qqx_in, double rrx_in);
  void update_diagnostic_attributes(double int_step);
  void Interpolation_Extrapolation(double T, double int_step,
                                   double ext_porlation);
  void set_reference_point(double rp);
  void set_reference_point_eq_xcg();

  arma::vec3 get_VBED();
  void set_liftoff(int in);

  struct TX_data {
    double SBEE[3];
    double VBEE[3];
    double ABEE[3];
    double JBEE[3];
    double psibd;
    double thtbd;
    double phibd;
    double WBEB[3];
  } TX_data_forward;

  void collect_forces_and_propagate();
  void set_DOF(int ndof);
  void set_aero_flag(unsigned int in);

 private:
  double get_ppx();
  double get_qqx();
  double get_rrx();
  double get_psibdx();
  double get_thtbdx();
  double get_phibdx();
  double get_dvbe();
  double get_dbi();
  double get_dvbi();
  double get_thtvdx();
  double get_psivdx();
  double get_thtbdx_in(double &cthtbd);

  void propagate_position_speed_acceleration(double int_step);
  void propagate_aeroloss(double int_step);
  void propagate_gravityloss(double int_step);
  void propagate_control_loss(double int_step);
  void vibration(double int_step);
  void propagate_TBI(double int_step, arma::vec3 WBIB_in);
  void propagate_TBI_Q(double int_step, arma::vec3 WBIB_in);
  void propagate_WBIB(double int_step, arma::vec3 FMB_in, arma::mat33 IBBB);
  void orbital(arma::vec3 SBII_in, arma::vec3 VBII_in, double dbi_in);
  void build_WEII();
  void aux_calulate(arma::mat33 TEI, arma::mat33 TBI_in);
  void RK4F(std::vector<arma::vec> Var_in, std::vector<arma::vec> &Var_out);
  void Send();
  void reference_point_calc();

  double calculate_alphaix(arma::vec3 VBIB);
  double calculate_betaix(arma::vec3 VBIB);
  double calculate_alppx(arma::vec3 VBAB_in, double dvba);
  double calculate_phipx(arma::vec3 VBAB_in);
  double calculate_alphax(arma::vec3 VBAB_in);
  double calculate_betax(arma::vec3 VBAB, double dvba);

  arma::vec build_VBEB(double _alpha0x, double _beta0x, double dvbe);
  arma::mat calculate_TBD(double lonx_in, double latx_in, double alt_in);
  arma::vec3 calculate_WBII(arma::mat33 TBI_in);
  arma::vec3 calculate_fspb(arma::vec3 FAPB_in, double vmass);
  arma::vec3 calculate_WBEB(arma::mat33 TBI_in);
  arma::vec3 euler_angle(arma::mat33 TBD_in);

  void gamma_beta();
  void Gravity_Q();
  void AeroDynamics_Q();
  void calculate_I1();
  void funcv(int n, double *x, double *ff);
  void broydn(double x[], int n, int *check);
  void rsolv(double **a, int n, double d[], double b[]);
  void fdjac(int n, double x[], double fvec_in[], double **df);
  double f_min(double x[]);
  void lnsrch(int n, double xold[], double fold, double g[], double p[],
              double x[], double *f_in, double stpmax, int *check);
  void qrdcmp(double **a, int n, double *c, double *d, int *sing);
  void qrupdt(double **r, double **qt, int n, double u[], double v[]);
  void rotate(double **r, double **qt, int n, int i, double a, double b);

  MATRIX(TBI, 3, 3); /* *o (--)    Transformation Matrix of body coord wrt
                    inertia coord */

  MATRIX(TBID, 3, 3); /* *o (--)    Transformation Matrix of body coord wrt
                     inertia coord derivative */

  VECTOR(TBI_Q, 4); /* *o (--)    Transformation Matrix of body coord wrt
                      inertia coord (Quaternion) */

  VECTOR(TBID_Q, 4); /* *o (--)    Transformation Matrix of body coord wrt
                       inertia coord derivative (Quaternion) */

  MATRIX(TBD, 3, 3); /* *o (--)    Transformation Matrix of body coord wrt
                    geodetic coord */
  VECTOR(TBDQ, 4);   /* *o  (--)    Quaternion from geodetic to body */

  VECTOR(VBAB, 3); /* *o  (m/s)    Air speed in body frame */

  MATRIX(WEII_skew, 3,
         3); /* *o  (r/s)    Earth's angular velocity (skew-sym) */

  VECTOR(SBIIP, 3); /* *o  (m)      Vehicle position in inertia coord */

  VECTOR(VBIIP, 3); /* *o  (m/s)    Vehicle inertia velocity */

  VECTOR(SBII, 3); /* *o  (m)      Vehicle position in inertia coord */

  VECTOR(VBII, 3); /* *o  (m/s)    Vehicle inertia velocity */

  VECTOR(ABII, 3); /* *o  (m/s2)   Vehicle inertia acceleration */

  VECTOR(ABIB,
         3); /* *o  (m/s2)   Vehicle inertia acceleration on body coordinate */

  VECTOR(SBEE, 3); /* *o  (m)     Vehicle position in earth coord  */

  VECTOR(VBEE, 3); /* *o  (m/s)     Vehicle speed in earth coord  */

  VECTOR(ABEE, 3); /* *o  (m/s2)   Vehicle acceleration in ECEF */

  VECTOR(SBEE_old, 3); /* *o  (m)     Vehicle position in earth coord  */

  VECTOR(VBEE_old, 3); /* *o  (m/s)     Vehicle speed in earth coord  */

  VECTOR(ABEE_old, 3); /* *o  (m/s2)   Vehicle acceleration in ECEF */

  VECTOR(JBII, 3); /* *o (m/s3)    Vehicle Jerk in ECI*/

  VECTOR(JBEE, 3); /* *o (m/s3)    Vehicle Jerk in ECEF */

  MATRIX(TDI, 3, 3); /* **  (--)     Transformation Matrix of geodetic wrt
                    inertial coordinates */

  MATRIX(
      TGI, 3,
      3); /* **  (--)     Transformation Matrix geocentric wrt inertia coord */

  VECTOR(VBED, 3); /* *o (m/s)   NED velocity */

  VECTOR(FSPB, 3); /* *o  (m/s2)   Specific force in body coord */

  VECTOR(NEXT_ACC, 3); /* *o (m/s2)   New Inertial acceleration */

  MATRIX(TDE, 3, 3); /* ** (--)  T.M. from ECEF to geodetic */

  VECTOR(VBII_old, 3); /* ** (m/s)  Prior body inertia velocity */

  VECTOR(WEII, 3); /* ** (--)  Earth rate in inertia coordinate */

  VECTOR(WBII, 3); /* *o (r/s)        Vehicle's inertia angular velocity in
                     inertia coord */

  VECTOR(WBEB, 3); /* *o (r/s)        Angular velocity of vehicle wrt earth in
                     body coord */

  VECTOR(WBIB, 3); /* *o (r/s)        Augular velocity of vehicle wrt inertia in
                     body coord */

  VECTOR(WBIBD, 3); /* *o (r/s2)       Angular velocity of vehicle wrt inertia
                      in body coord - derivative */

  MATRIX(TVD, 3, 3); /* **  (--)     Transformation Matrix of geographic
                    velocity wrt geodetic coord */

  VECTOR(SBEE_test, 3); /* *o  (m)     Vehicle position in earth coord  */

  VECTOR(VBEE_test, 3); /* *o  (m/s)     Vehicle speed in earth coord  */

  VECTOR(ABEE_test, 3); /* *o  (m/s2)   Vehicle acceleration in ECEF */

  MATRIX(TLI, 3, 3); /* **  (--)  T.M. from inertia to launch site */

  VECTOR(LT_euler, 3); /* ** (rad)  Launch site coordinate Euler angle */

  VECTOR(TBLQ, 4); /* ** (--) Quaternion from launch site to body */

  VECTOR(ABID, 3); /* ** (--) Vehicle inertia body acceleration in geodetic */

  VECTOR(FAPB,
         3); /* *o (N)      Aerodynamic and propulsion forces in body axes */

  VECTOR(FAP, 3); /* *o (N)      Aerodynamic force in body axes */

  VECTOR(FMB,
         3); /* *o (N*m)    Aerodynamic and propulsion moment in body axes */

  VECTOR(FMAB,
         3); /* *o (N*m)    Aerodynamic and propulsion moment in body axes */

  VECTOR(Q_G, 6); /* *o (--)     External force generated by gravity */

  VECTOR(Q_Aero, 6); /* *o (--)  External force generated by aerodynamics */

  VECTOR(rhoC_1, 3); /* *o  (m)  Level arm from reference ponit to CG */

  MATRIX(I1, 3, 3); /* *o (kg*m2)  MOI of vehicle */

  VECTOR(ddrP_1, 3); /* *o  (m/s2)  Vehicle acceleration */

  VECTOR(ddang_1, 3); /* *o  (r/s2)  Vehicle angular acceleration */

  VECTOR(dang_1, 3); /* *o  (r/s)   Vehicle angular rate */

  VECTOR(ddrhoC_1, 3); /* *o  (m/s2)  Centrifugal acceleration and tangential
                          acceleration term */

  VECTOR(p_b1_ga,
         3); /* *o (--)   General dynamics equations 1st DoF to 3rd DoF */

  VECTOR(p_b1_be,
         3); /* *o (--)   General dynamics equations 4th DoF to 6th DoF */

  VECTOR(f, 6); /* *o  (--)  Summation of external force & internal force */

  VECTOR(gamma_b1_q1, 3); /* *o (--)  Vehicle's 1st DoF velocity coefficient */

  VECTOR(gamma_b1_q2, 3); /* *o (--)  Vehicle's 2nd DoF velocity coefficient */

  VECTOR(gamma_b1_q3, 3); /* *o (--)  Vehicle's 3rd DoF velocity coefficient */

  VECTOR(beta_b1_q4,
         3); /* *o (--)  Vehicle's 4th DoF angular velocity coefficient */

  VECTOR(beta_b1_q5,
         3); /* *o (--)  Vehicle's 5th DoF angular velocity coefficient */

  VECTOR(beta_b1_q6,
         3); /* *o (--)  Vehicle's 6th DoF angular velocity coefficient */

  /* Generating Outputs */
  double
      ortho_error; /* *o (--)    Direction cosine matrix orthogonality error*/
  double alphax;   /* *o (d)     Angle of attack */
  double betax;    /* *o (d)     Sideslip angle */
  double alppx;    /* *o (d)     Total angle of attack */
  double phipx;    /* *o (d)     Aerodynamic roll angle*/
  double alphaix;  /* *o (d)     Angle of attack, inertia velocity*/
  double betaix;   /* *o (d)     Sideslip angle, inertia velocity*/
  double psibdx; /* *o (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
  double
      thtbdx; /* *o (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
  double phibdx; /* *o (d)     Roll angle of Vehicle wrt geodetic coord - deg */
  double psibd;  /* *o (r)     Yaw angle of Vehicle wrt geodetic coord - rad */
  double thtbd; /* *o (r)     Pitch angle of Vehicle wrt geodetic coord - rad */
  double phibd; /* *o (r)     Roll angle of Vehicle wrt geodetic coord - rad */
  double alt;   /* *o  (m)      Vehicle altitude */
  double lonx;  /* *o  (d)      Vehicle longitude */
  double latx;  /* *o  (d)      Vehicle latitude */
  double _aero_loss; /* *o  (m/s)    Velocity loss caused by aerodynamic drag */
  double gravity_loss; /* *o  (m/s)    Velocity loss caused by gravity */
  // double t;            /* *o (s)       timer */
  double _grndtrck; /* *o  (m)     [DIAG] Vehicle ground track on earth */
  double _gndtrkmx; /* *o  (km)    [DIAG] Ground track - km */
  double _gndtrnmx; /* **  (nm)    [DIAG] Ground track - nm */
  double _ayx;      /* *o  (m/s2)  [DIAG] Achieved side acceleration */
  double _anx;      /* *o  (m/s2)  [DIAG] Achieved normal acceleration */
  double _dbi;    /* *o  (m)     [DIAG] Vehicle distance from center of earth */
  double _dvbi;   /* *o  (m/s)   [DIAG] Vehicle inertia speed */
  double _dvbe;   /* *o  (m/s)   [DIAG] Vehicle geographic speed */
  double _thtvdx; /* *o  (d)     [DIAG] Vehicle's flight path angle */
  double _psivdx; /* *o  (d)     [DIAG] Vehicle's heading angle */
  int liftoff;    /* *i  (--)     To check wether the rocket liftoff or
                              not: liftoff = 1, not liftoff = 0 */
  int cadorbin_flag; /* Orbit calculation status flag */
  double ppx; /* *o (d/s)        Body roll angular velocity wrt earth in body
                 axes */
  double qqx; /* *o (d/s)        Body pitch angular velocity wrt earth in body
                 axes */
  double rrx; /* *o (d/s)        Body yaw angular velocity wrt earth in body
                 axes */
  double control_loss; /* *o (--) Velocity loss due to control effect */

  /* Orbital Logging */
  double _inclination;  /* *o  (deg)   [DIAG] Orbital inclination is the minimun
                        angle between reference plane and the orbital plane or
                        direction of an object in orbit around another object */
  double _eccentricity; /* *o  (--)    [DIAG] Determines the amount by which its
                           orbit around another body deviates from a perfect
                           circle */
  double _semi_major; /* *o  (m)     [DIAG] the major axis of an ellipse is its
                         longest diameter */
  double _ha;         /* *o  (m)     [DIAG] Orbital Apogee */
  double _hp;         /* *o  (m)     [DIAG] Orbital Perigee */
  double _lon_anodex; /* *o  (deg)   [DIAG] The longitude of the ascending node
                         (☊ or Ω) is one of the orbital elements used to specify
                         the orbit of an object in space. It is the angle from a
                         reference direction, called the origin of longitude, to
                         the direction of the ascending node, measured in a
                         reference plane */
  double _arg_perix;  /* *o  (deg)   [DIAG] The argument of periapsis (also
                         called argument of perifocus or argument of pericenter),
                         symbolized as ω, is one of the orbital elements of an
                         orbiting body. Parametrically, ω is the angle from the
                         body's ascending node to its periapsis, measured in the
                         direction of motion */
  double _true_anomx; /* *o  (deg)   [DIAG] In celestial mechanics, true anomaly
                      is an angular parameter that defines the position of a
                      body moving along a Keplerian orbit. It is the angle
                      between the direction of periapsis and the current
                      position of the body, as seen from the main focus of the
                      ellipse (the point around which the object orbits) */
  double _ref_alt;    /* *o  (m)     [DIAG] */
  double reference_point; /* *o (m)    Multibody dynamics reference point */
  double Roll;
  double Pitch;
  double Yaw;
  unsigned int Interpolation_Extrapolation_flag;
  // double xp; /* *o (m) Reference point  */
  int its;                  /* *o (--) Number of iterations */
  int DOF;                  /* *o (--)  Number of Degree of Freedom */
  int reference_point_flag; /* *o (--)  check if reference point equal to xcg */
  unsigned int Aero_flag;   /* *o (-)  Aerodynamics flag */
};
#endif
