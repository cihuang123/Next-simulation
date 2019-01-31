#include <iomanip>
#include <tuple>
#include "Rocket_Flight_DM.hh"
#include "aux.hh"
#include "cad_utility.hh"
#include "integrate.hh"
#include "math_utility.hh"
#include "matrix/utility.hh"
#include "sim_services/include/simtime.h"
#define NRANSI
#include "nrutil.h"
#define MAXITS 20000000
#define ALF 1.0e-4
#define EPS 1.0e-7   // 1.0e-7
#define TOLF 1.0e-8  // 1.0e-4
#define TOLX 1.0e-9
#define STPMX 100.0
#define TOLMIN 1.0e-10  // 1.0e-6
#define FREERETURN                \
  do {                            \
    free_dvector(fvec, 1, n);     \
    free_dvector(xold, 1, n);     \
    free_dvector(w, 1, n);        \
    free_dvector(t, 1, n);        \
    free_dvector(s, 1, n);        \
    free_dmatrix(r, 1, n, 1, n);  \
    free_dmatrix(qt, 1, n, 1, n); \
    free_dvector(p, 1, n);        \
    free_dvector(g, 1, n);        \
    free_dvector(fvcold, 1, n);   \
    free_dvector(d, 1, n);        \
    free_dvector(c, 1, n);        \
    return;                       \
  } while (0);

double *fvec;
int nn;

Rocket_Flight_DM::Rocket_Flight_DM(Data_exchang &input)
    : MATRIX_INIT(TBI, 3, 3),
      MATRIX_INIT(TBID, 3, 3),
      VECTOR_INIT(TBI_Q, 4),
      VECTOR_INIT(TBID_Q, 4),
      MATRIX_INIT(TBD, 3, 3),
      VECTOR_INIT(TBDQ, 4),
      VECTOR_INIT(VBAB, 3),
      MATRIX_INIT(WEII_skew, 3, 3),
      VECTOR_INIT(SBIIP, 3),
      VECTOR_INIT(VBIIP, 3),
      VECTOR_INIT(SBII, 3),
      VECTOR_INIT(VBII, 3),
      VECTOR_INIT(ABII, 3),
      VECTOR_INIT(ABIB, 3),
      VECTOR_INIT(SBEE, 3),
      VECTOR_INIT(VBEE, 3),
      VECTOR_INIT(ABEE, 3),
      VECTOR_INIT(SBEE_old, 3),
      VECTOR_INIT(VBEE_old, 3),
      VECTOR_INIT(ABEE_old, 3),
      VECTOR_INIT(JBII, 3),
      VECTOR_INIT(JBEE, 3),
      MATRIX_INIT(TDI, 3, 3),
      MATRIX_INIT(TGI, 3, 3),
      VECTOR_INIT(VBED, 3),
      VECTOR_INIT(FSPB, 3),
      VECTOR_INIT(NEXT_ACC, 3),
      MATRIX_INIT(TDE, 3, 3),
      VECTOR_INIT(VBII_old, 3),
      VECTOR_INIT(WEII, 3),
      VECTOR_INIT(WBII, 3),
      VECTOR_INIT(WBEB, 3),
      VECTOR_INIT(WBIB, 3),
      VECTOR_INIT(WBIBD, 3),
      MATRIX_INIT(TVD, 3, 3),
      VECTOR_INIT(SBEE_test, 3),
      VECTOR_INIT(VBEE_test, 3),
      VECTOR_INIT(ABEE_test, 3),
      MATRIX_INIT(TLI, 3, 3),
      VECTOR_INIT(LT_euler, 3),
      VECTOR_INIT(TBLQ, 4),
      VECTOR_INIT(ABID, 3),
      VECTOR_INIT(FAPB, 3),
      VECTOR_INIT(FAP, 3),
      VECTOR_INIT(FMB, 3),
      VECTOR_INIT(FMAB, 3),
      VECTOR_INIT(Q_G, 6),
      VECTOR_INIT(Q_Aero, 6),
      VECTOR_INIT(rhoC_1, 3),
      MATRIX_INIT(I1, 3, 3),
      VECTOR_INIT(ddrP_1, 3),
      VECTOR_INIT(ddang_1, 3),
      VECTOR_INIT(dang_1, 3),
      VECTOR_INIT(ddrhoC_1, 3),
      VECTOR_INIT(p_b1_ga, 3),
      VECTOR_INIT(p_b1_be, 3),
      VECTOR_INIT(f, 6),
      VECTOR_INIT(gamma_b1_q1, 3),
      VECTOR_INIT(gamma_b1_q2, 3),
      VECTOR_INIT(gamma_b1_q3, 3),
      VECTOR_INIT(beta_b1_q4, 3),
      VECTOR_INIT(beta_b1_q5, 3),
      VECTOR_INIT(beta_b1_q6, 3) {
  data_exchang = &input;
}

Rocket_Flight_DM::Rocket_Flight_DM(const Rocket_Flight_DM &other)
    : MATRIX_INIT(TBI, 3, 3),
      MATRIX_INIT(TBID, 3, 3),
      VECTOR_INIT(TBI_Q, 4),
      VECTOR_INIT(TBID_Q, 4),
      MATRIX_INIT(TBD, 3, 3),
      VECTOR_INIT(TBDQ, 4),
      VECTOR_INIT(VBAB, 3),
      MATRIX_INIT(WEII_skew, 3, 3),
      VECTOR_INIT(SBIIP, 3),
      VECTOR_INIT(VBIIP, 3),
      VECTOR_INIT(SBII, 3),
      VECTOR_INIT(VBII, 3),
      VECTOR_INIT(ABII, 3),
      VECTOR_INIT(ABIB, 3),
      VECTOR_INIT(SBEE, 3),
      VECTOR_INIT(VBEE, 3),
      VECTOR_INIT(ABEE, 3),
      VECTOR_INIT(SBEE_old, 3),
      VECTOR_INIT(VBEE_old, 3),
      VECTOR_INIT(ABEE_old, 3),
      VECTOR_INIT(JBII, 3),
      VECTOR_INIT(JBEE, 3),
      MATRIX_INIT(TDI, 3, 3),
      MATRIX_INIT(TGI, 3, 3),
      VECTOR_INIT(VBED, 3),
      VECTOR_INIT(FSPB, 3),
      VECTOR_INIT(NEXT_ACC, 3),
      MATRIX_INIT(TDE, 3, 3),
      VECTOR_INIT(VBII_old, 3),
      VECTOR_INIT(WEII, 3),
      VECTOR_INIT(WBII, 3),
      VECTOR_INIT(WBEB, 3),
      VECTOR_INIT(WBIB, 3),
      VECTOR_INIT(WBIBD, 3),
      MATRIX_INIT(TVD, 3, 3),
      VECTOR_INIT(SBEE_test, 3),
      VECTOR_INIT(VBEE_test, 3),
      VECTOR_INIT(ABEE_test, 3),
      MATRIX_INIT(TLI, 3, 3),
      VECTOR_INIT(LT_euler, 3),
      VECTOR_INIT(TBLQ, 4),
      VECTOR_INIT(ABID, 3),
      VECTOR_INIT(FAPB, 3),
      VECTOR_INIT(FAP, 3),
      VECTOR_INIT(FMB, 3),
      VECTOR_INIT(FMAB, 3),
      VECTOR_INIT(Q_G, 6),
      VECTOR_INIT(Q_Aero, 6),
      VECTOR_INIT(rhoC_1, 3),
      MATRIX_INIT(I1, 3, 3),
      VECTOR_INIT(ddrP_1, 3),
      VECTOR_INIT(ddang_1, 3),
      VECTOR_INIT(dang_1, 3),
      VECTOR_INIT(ddrhoC_1, 3),
      VECTOR_INIT(p_b1_ga, 3),
      VECTOR_INIT(p_b1_be, 3),
      VECTOR_INIT(f, 6),
      VECTOR_INIT(gamma_b1_q1, 3),
      VECTOR_INIT(gamma_b1_q2, 3),
      VECTOR_INIT(gamma_b1_q3, 3),
      VECTOR_INIT(beta_b1_q4, 3),
      VECTOR_INIT(beta_b1_q5, 3),
      VECTOR_INIT(beta_b1_q6, 3) {
  this->data_exchang = other.data_exchang;
  this->TBD = other.TBD;
  this->TBI = other.TBI;
  this->TBID = other.TBID;
  this->WEII_skew = other.WEII_skew;
  this->TDI = other.TDI;
  this->TDE = other.TDE;
  this->TGI = other.TGI;
  this->TBI_Q = other.TBI_Q;
  this->TBID_Q = other.TBID_Q;
  this->TBDQ = other.TBDQ;
  this->VBAB = other.VBAB;
  this->SBII = other.SBII;
  this->VBII = other.VBII;
  this->SBIIP = other.SBIIP;
  this->VBIIP = other.VBIIP;
  this->ABII = other.ABII;
  this->FSPB = other.FSPB;
  this->ABIB = other.ABIB;
  this->SBEE = other.SBEE;
  this->VBEE = other.VBEE;
  this->ABEE = other.ABEE;
  this->JBII = other.JBII;
  this->JBEE = other.JBEE;
  this->NEXT_ACC = other.NEXT_ACC;
  this->VBED = other.VBED;
  this->VBII_old = other.VBII_old;
  this->WEII = other.WEII;
  this->WBII = other.WBII;
  this->WBIB = other.WBIB;
  this->WBIBD = other.WBIBD;
  this->WBEB = other.WBEB;
  this->TLI = other.TLI;
  this->LT_euler = other.LT_euler;
  this->TBLQ = other.TBLQ;
  this->ABID = other.ABID;
  this->ortho_error = other.ortho_error;
  this->alphax = other.alphax;
  this->betax = other.betax;
  this->alppx = other.alppx;
  this->phipx = other.phipx;
  this->alphaix = other.alphaix;
  this->betaix = other.betaix;
  this->psibdx = other.psibdx;
  this->thtbdx = other.thtbdx;
  this->phibdx = other.phibdx;
  this->psibd = other.psibd;
  this->thtbd = other.thtbd;
  this->phibd = other.phibd;
  this->alt = other.alt;
  this->lonx = other.lonx;
  this->latx = other.latx;
  this->_aero_loss = other._aero_loss;
  this->gravity_loss = other.gravity_loss;
  this->_grndtrck = other._grndtrck;
  this->_gndtrkmx = other._gndtrkmx;
  this->_gndtrnmx = other._gndtrnmx;
  this->_ayx = other._ayx;
  this->_anx = other._anx;
  this->_dbi = other._dbi;
  this->_dvbi = other._dvbi;
  this->_dvbe = other._dvbe;
  this->_thtvdx = other._thtvdx;
  this->_psivdx = other._psivdx;
  this->liftoff = other.liftoff;
  this->ppx = other.ppx;
  this->qqx = other.qqx;
  this->rrx = other.rrx;
  this->control_loss = other.control_loss;
  this->_inclination = other._inclination;
  this->_eccentricity = other._eccentricity;
  this->_semi_major = other._semi_major;
  this->_ha = other._ha;
  this->_hp = other._hp;
  this->_lon_anodex = other._lon_anodex;
  this->_arg_perix = other._arg_perix;
  this->_true_anomx = other._true_anomx;
  this->_ref_alt = other._ref_alt;
  this->reference_point = other.reference_point;
  this->Roll = other.Roll;
  this->Pitch = other.Pitch;
  this->Yaw = other.Yaw;
  this->FAP = other.FAP;
  this->FAPB = other.FAPB;
  this->FMB = other.FMB;
  this->FMAB = other.FMAB;
  this->Q_G = other.Q_G;
  this->Q_Aero = other.Q_Aero;
  this->rhoC_1 = other.rhoC_1;
  this->I1 = other.I1;
  this->ddrP_1 = other.ddrP_1;
  this->ddang_1 = other.ddang_1;
  this->dang_1 = other.dang_1;
  this->ddrhoC_1 = other.ddrhoC_1;
  this->p_b1_ga = other.p_b1_ga;
  this->p_b1_be = other.p_b1_be;
  this->f = other.f;
  this->gamma_b1_q1 = other.gamma_b1_q1;
  this->gamma_b1_q2 = other.gamma_b1_q2;
  this->gamma_b1_q3 = other.gamma_b1_q3;
  this->beta_b1_q4 = other.beta_b1_q4;
  this->beta_b1_q5 = other.beta_b1_q5;
  this->beta_b1_q6 = other.beta_b1_q6;
}

Rocket_Flight_DM &Rocket_Flight_DM::operator=(const Rocket_Flight_DM &other) {
  if (&other == this) return *this;
  this->data_exchang = other.data_exchang;
  this->TBD = other.TBD;
  this->TBI = other.TBI;
  this->TBID = other.TBID;
  this->WEII_skew = other.WEII_skew;
  this->TDI = other.TDI;
  this->TDE = other.TDE;
  this->TGI = other.TGI;
  this->TBI_Q = other.TBI_Q;
  this->TBID_Q = other.TBID_Q;
  this->TBDQ = other.TBDQ;
  this->VBAB = other.VBAB;
  this->SBII = other.SBII;
  this->VBII = other.VBII;
  this->SBIIP = other.SBIIP;
  this->VBIIP = other.VBIIP;
  this->ABII = other.ABII;
  this->FSPB = other.FSPB;
  this->ABIB = other.ABIB;
  this->SBEE = other.SBEE;
  this->VBEE = other.VBEE;
  this->ABEE = other.ABEE;
  this->JBII = other.JBII;
  this->JBEE = other.JBEE;
  this->NEXT_ACC = other.NEXT_ACC;
  this->VBED = other.VBED;
  this->VBII_old = other.VBII_old;
  this->WEII = other.WEII;
  this->WBII = other.WBII;
  this->WBIB = other.WBIB;
  this->WBIBD = other.WBIBD;
  this->WBEB = other.WBEB;
  this->TLI = other.TLI;
  this->LT_euler = other.LT_euler;
  this->TBLQ = other.TBLQ;
  this->ABID = other.ABID;
  this->ortho_error = other.ortho_error;
  this->alphax = other.alphax;
  this->betax = other.betax;
  this->alppx = other.alppx;
  this->phipx = other.phipx;
  this->alphaix = other.alphaix;
  this->betaix = other.betaix;
  this->psibdx = other.psibdx;
  this->thtbdx = other.thtbdx;
  this->phibdx = other.phibdx;
  this->psibd = other.psibd;
  this->thtbd = other.thtbd;
  this->phibd = other.phibd;
  this->alt = other.alt;
  this->lonx = other.lonx;
  this->latx = other.latx;
  this->_aero_loss = other._aero_loss;
  this->gravity_loss = other.gravity_loss;
  this->_grndtrck = other._grndtrck;
  this->_gndtrkmx = other._gndtrkmx;
  this->_gndtrnmx = other._gndtrnmx;
  this->_ayx = other._ayx;
  this->_anx = other._anx;
  this->_dbi = other._dbi;
  this->_dvbi = other._dvbi;
  this->_dvbe = other._dvbe;
  this->_thtvdx = other._thtvdx;
  this->_psivdx = other._psivdx;
  this->liftoff = other.liftoff;
  this->ppx = other.ppx;
  this->qqx = other.qqx;
  this->rrx = other.rrx;
  this->control_loss = other.control_loss;
  this->_inclination = other._inclination;
  this->_eccentricity = other._eccentricity;
  this->_semi_major = other._semi_major;
  this->_ha = other._ha;
  this->_hp = other._hp;
  this->_lon_anodex = other._lon_anodex;
  this->_arg_perix = other._arg_perix;
  this->_true_anomx = other._true_anomx;
  this->_ref_alt = other._ref_alt;
  this->reference_point = other.reference_point;
  this->Roll = other.Roll;
  this->Pitch = other.Pitch;
  this->Yaw = other.Yaw;
  this->FAP = other.FAP;
  this->FAPB = other.FAPB;
  this->FMB = other.FMB;
  this->FMAB = other.FMAB;
  this->Q_G = other.Q_G;
  this->Q_Aero = other.Q_Aero;
  this->rhoC_1 = other.rhoC_1;
  this->I1 = other.I1;
  this->ddrP_1 = other.ddrP_1;
  this->ddang_1 = other.ddang_1;
  this->dang_1 = other.dang_1;
  this->ddrhoC_1 = other.ddrhoC_1;
  this->p_b1_ga = other.p_b1_ga;
  this->p_b1_be = other.p_b1_be;
  this->f = other.f;
  this->gamma_b1_q1 = other.gamma_b1_q1;
  this->gamma_b1_q2 = other.gamma_b1_q2;
  this->gamma_b1_q3 = other.gamma_b1_q3;
  this->beta_b1_q4 = other.beta_b1_q4;
  this->beta_b1_q5 = other.beta_b1_q5;
  this->beta_b1_q6 = other.beta_b1_q6;
  return *this;
}

void Rocket_Flight_DM::init() {
  arma::mat33 TEI;  // cad::tei(get_elapsed_time());
  data_exchang->hget("TEI", TEI);
  arma::vec3 XCG;
  data_exchang->hget("XCG_0", XCG);
  build_WEII();

  rhoC_1(0) = -XCG(0) - reference_point;
  rhoC_1(1) = 0.0;
  rhoC_1(2) = 0.0;

  // converting geodetic lonx, latx, alt to SBII
  SBII = cad::in_geo84(lonx * RAD, latx * RAD, alt, TEI);

  // building inertial velocity
  TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
  TGI = cad::tgi84(lonx * RAD, latx * RAD, alt, TEI);
  TDE = cad::tde84(lonx * RAD, latx * RAD, alt);
  TBD = build_psi_tht_phi_TM(psibdx * RAD, thtbdx * RAD, phibdx * RAD);
  psibd = psibdx * RAD;
  thtbd = thtbdx * RAD;
  phibd = phibdx * RAD;
  TBI = TBD * TDI;
  TLI = TBI;

  this->WBIB = this->WBEB + TBI * this->WEII;

  this->TBI_Q = Matrix2Quaternion(
      this->TBI);  // Convert Direct Cosine Matrix to Quaternion

  arma::mat VBEB = this->build_VBEB(alphax, betax, _dvbe);
  // Geodetic velocity
  VBED = trans(TBD) * VBEB;

  VBII = trans(TDI) * VBED + trans(TEI) * (WEII_skew * (TEI * SBII));
  SBIIP = SBII - trans(TBI) * rhoC_1;
  VBIIP = VBII - trans(TBI) * cross(WBIB, rhoC_1);
  arma::vec3 GRAVG;
  data_exchang->hget("GRAVG", GRAVG);
  this->ABII = trans(TEI) * (WEII_skew * WEII_skew * (TEI * SBII));
  FSPB = TBI * (-GRAVG + ABII);  // FSPB: body force include gravity acc
  SBEE = TEI * SBII;             // Calculate position in ECEF
  VBEE = TEI * VBII - cross(WEII, SBEE);  // Calculate velocity in ECEF
  NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * (SBIIP)))));
  Interpolation_Extrapolation_flag = 4;

  if (liftoff == 1) {
    arma::vec3 VAED;
    data_exchang->hget("VAED", VAED);
    VBAB = TBD * (TDE * VBEE - VAED);
    this->alphax = calculate_alphax(VBAB);
    this->betax = calculate_betax(VBAB, norm(VBAB));

    this->alppx = calculate_alppx(VBAB, norm(VBAB));
    this->phipx = calculate_phipx(VBAB);
  }

  Send();
}

void Rocket_Flight_DM::load_angle(double yaw, double roll, double pitch) {
  this->psibdx = yaw;
  this->phibdx = roll;
  this->thtbdx = pitch;
}

void Rocket_Flight_DM::load_angular_velocity(double ppx_in, double qqx_in,
                                             double rrx_in) {
  // body rate wrt Earth frame in body coordinates
  this->WBEB = {ppx_in * RAD, qqx_in * RAD, rrx_in * RAD};
}

void Rocket_Flight_DM::load_location(double lonx_in, double latx_in,
                                     double alt_in) {
  this->lonx = lonx_in;
  this->latx = latx_in;
  this->alt = alt_in;
  arma::mat33 TEI;
  data_exchang->hget("TEI", TEI);
  SBII = cad::in_geo84(lonx * RAD, latx * RAD, alt, TEI);
  TGI = cad::tgi84(lonx * RAD, latx * RAD, alt, TEI);
  data_exchang->hset("SBII", SBII);
  data_exchang->hset("TGI", TGI);
}

void Rocket_Flight_DM::load_geodetic_velocity(double alpha0x, double beta0x,
                                              double dvbe) {
  this->_dvbe = dvbe;
  this->alphax = alpha0x;
  this->betax = beta0x;
  data_exchang->hset("dvbe", _dvbe);
}

void Rocket_Flight_DM::set_liftoff(int in) { liftoff = in; }

void Rocket_Flight_DM::build_WEII() {
  WEII_skew(0, 1) = -WEII3;
  WEII_skew(1, 0) = WEII3;

  WEII(2) = WEII3;
}

arma::vec Rocket_Flight_DM::build_VBEB(double _alpha0x, double _beta0x,
                                       double dvbe) {
  double salp = sin(_alpha0x * RAD);
  double calp = cos(_alpha0x * RAD);
  double sbet = sin(_beta0x * RAD);
  double cbet = cos(_beta0x * RAD);
  double vbeb1 = calp * cbet * dvbe;
  double vbeb2 = sbet * dvbe;
  double vbeb3 = salp * cbet * dvbe;
  arma::vec3 VBEB = {vbeb1, vbeb2, vbeb3};
  return VBEB;
}

void Rocket_Flight_DM::set_reference_point(double rp) { reference_point = rp; }

void Rocket_Flight_DM::algorithm(double int_step) {
  arma::vec3 VAED;
  arma::mat33 TEI;
  data_exchang->hget("TEI", TEI);
  data_exchang->hget("VAED", VAED);

  std::vector<arma::vec> V_State_In;
  std::vector<arma::vec> V_State_Out(4);

  V_State_In.push_back(VBIIP);
  V_State_In.push_back(SBIIP);
  V_State_In.push_back(WBIB);
  V_State_In.push_back(TBI_Q);

  IntegratorRK4(V_State_In, V_State_Out, &Rocket_Flight_DM::RK4F, this,
                int_step);

  VBIIP = V_State_Out[0];
  SBIIP = V_State_Out[1];
  WBIB = V_State_Out[2];
  TBI_Q = V_State_Out[3];
  this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

  arma::vec3 rhoC_IMU;
  arma::vec3 XCG;

  data_exchang->hget("XCG", XCG);

  rhoC_IMU(0) = -XCG(0) - (reference_point);
  rhoC_IMU(1) = 0.0;
  rhoC_IMU(2) = 0.0;

  SBII = SBIIP + trans(TBI) * rhoC_IMU;
  VBII = VBIIP + trans(TBI) * cross(WBIB, rhoC_IMU);
  ABII = NEXT_ACC + trans(TBI) * ddrhoC_1;

  Send();

  this->TBD = calculate_TBD(lonx, latx, alt);
  aux_calulate(TEI, TBI);
  update_diagnostic_attributes(int_step);
  if (liftoff == 1) {
    propagate_aeroloss(int_step);
    propagate_gravityloss(int_step);
    propagate_control_loss(int_step);
    VBAB = TBD * (TDE * VBEE - VAED);
    this->alphax = calculate_alphax(VBAB);
    this->betax = calculate_betax(VBAB, norm(VBAB));

    this->alppx = calculate_alppx(VBAB, norm(VBAB));
    this->phipx = calculate_phipx(VBAB);
  }
  // *incidence angles using wind vector VAED in geodetic coord
  // *diagnostic: calculating the inertial incidence angles
  arma::vec3 VBIB = TBI * VBII;
  this->alphaix = calculate_alphaix(VBIB);
  this->betaix = calculate_betaix(VBIB);

  this->TBDQ = Matrix2Quaternion(this->TBD);
  Quaternion2Euler(TBDQ, Roll, Pitch, Yaw);

  Send();
}

void Rocket_Flight_DM::Send() {
  data_exchang->hset("TBI", TBI);
  data_exchang->hset("WBIB", WBIB);
  data_exchang->hset("FSPB", FSPB);
  data_exchang->hset("NEXT_ACC", NEXT_ACC);
  data_exchang->hset("liftoff", liftoff);
  data_exchang->hset("WBEB", WBEB);
  data_exchang->hset("betax", betax);
  data_exchang->hset("alphax", alphax);
  data_exchang->hset("TDE", TDE);
  data_exchang->hset("VBEE", VBEE);
  data_exchang->hset("phipx", phipx);
  data_exchang->hset("alppx", alppx);
  data_exchang->hset("TBD", TBD);
  data_exchang->hset("TBI", TBI);
  data_exchang->hset("TGI", TGI);
  data_exchang->hset("alt", alt);
  data_exchang->hset("VBED", VBED);
  data_exchang->hset("SBII", SBII);
  data_exchang->hset("dvbe", _dvbe);
  data_exchang->hset("WBIBD", WBIBD);
}

void Rocket_Flight_DM::propagate_TBI(double int_step, arma::vec3 WBIB_in) {
  // *integrating direction cosine matrix
  INTEGRATE_MAT(TBI, trans(skew_sym(WBIB_in)) * this->TBI);

  // orthonormalizing TBI
  arma::mat EE = arma::eye(3, 3) - TBI * trans(TBI);
  this->TBI = TBI + EE * TBI * 0.5;

  // TBI orthogonality check
  double e1 = EE(0, 0);
  double e2 = EE(1, 1);
  double e3 = EE(2, 2);
  this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

void Rocket_Flight_DM::propagate_TBI_Q(double int_step, arma::vec3 WBIB_in) {
  arma::vec TBID_Q_NEW(4);
  /* Prepare for orthonormalization */
  double quat_metric = TBI_Q(0) * TBI_Q(0) + TBI_Q(1) * TBI_Q(1) +
                       TBI_Q(2) * TBI_Q(2) + TBI_Q(3) * TBI_Q(3);
  double erq = 1. - quat_metric;

  /* Calculate Previous states */
  TBID_Q_NEW(0) = 0.5 * (-WBIB_in(0) * TBI_Q(1) - WBIB_in(1) * TBI_Q(2) -
                         WBIB_in(2) * TBI_Q(3)) +
                  50. * erq * TBI_Q(0);
  TBID_Q_NEW(1) = 0.5 * (WBIB_in(0) * TBI_Q(0) + WBIB_in(2) * TBI_Q(2) -
                         WBIB_in(1) * TBI_Q(3)) +
                  50. * erq * TBI_Q(1);
  TBID_Q_NEW(2) = 0.5 * (WBIB_in(1) * TBI_Q(0) - WBIB_in(2) * TBI_Q(1) +
                         WBIB_in(0) * TBI_Q(3)) +
                  50. * erq * TBI_Q(2);
  TBID_Q_NEW(3) = 0.5 * (WBIB_in(2) * TBI_Q(0) + WBIB_in(1) * TBI_Q(1) -
                         WBIB_in(0) * TBI_Q(2)) +
                  50. * erq * TBI_Q(3);

  this->TBI_Q = integrate(TBID_Q_NEW, this->TBID_Q, this->TBI_Q, int_step);

  this->TBID_Q = TBID_Q_NEW;

  this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

  // TBI orthogonality check
  arma::mat TIB = trans(TBI);
  arma::mat UBI = TIB * TBI;
  double e1 = UBI(0, 0) - 1.;
  double e2 = UBI(1, 1) - 1.;
  double e3 = UBI(2, 2) - 1.;
  this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

void Rocket_Flight_DM::propagate_position_speed_acceleration(double int_step) {
  arma::vec3 GRAVG;
  arma::mat33 TEI;  // cad::tei(get_elapsed_time());

  data_exchang->hget("GRAVG", GRAVG);
  data_exchang->hget("TEI", TEI);

  /* Prograte S, V, A status */
  NEXT_ACC = trans(TBI) * FSPB + GRAVG;  // Strapdown Analytics 4.3-11
  /* To check wether the rocket liftoff */
  if (liftoff == 0) {
    if (norm(FSPB) - norm(GRAVG) > 0) {
      liftoff = 1;
    }
    NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * SBII))));
    FSPB = TBI * (-GRAVG + NEXT_ACC);  // Strapdown Analytics 4.3-14
  } else {
    FSPB = FSPB;  //+ TBI * GRAVG;  // FSPB: body force include gravity acc
  }

  arma::mat NEXT_VEL = integrate(NEXT_ACC, ABII, VBII, int_step);
  SBII = integrate(VBII, VBII_old, SBII, int_step);
  JBII = (NEXT_ACC - ABII) / int_step;  // Calculate Jerk in ECI
  ABII = NEXT_ACC;
  VBII_old = VBII;
  VBII = NEXT_VEL;
}
arma::mat Rocket_Flight_DM::calculate_TBD(double lonx_in, double latx_in,
                                          double alt_in) {
  // _Euler_ angles
  arma::mat33 TEI;
  data_exchang->hget("TEI", TEI);
  arma::mat tdi = cad::tdi84(lonx_in * RAD, latx_in * RAD, alt_in, TEI);
  return this->TBI * trans(tdi);
}

double Rocket_Flight_DM::calculate_alphaix(arma::vec3 VBIB) {
  return atan2(VBIB(2), VBIB(0)) * DEG;
}

void Rocket_Flight_DM::propagate_aeroloss(double int_step) {
  double vmass;
  data_exchang->hget("vmass", &vmass);
  // calculate aero loss
  arma::vec3 tmp = FAPB * (1. / vmass);
  _aero_loss = _aero_loss + norm(tmp) * int_step;
}

void Rocket_Flight_DM::propagate_control_loss(double int_step) {
  arma::vec6 Q_TVC;
  double vmass;
  data_exchang->hget("Q_TVC", Q_TVC);
  data_exchang->hget("vmass", &vmass);
  arma::vec3 A_TVC_BODY;
  A_TVC_BODY(0) = Q_TVC(0) / vmass;
  A_TVC_BODY(1) = Q_TVC(1) / vmass;
  A_TVC_BODY(2) = Q_TVC(2) / vmass;

  A_TVC_BODY = TBI * A_TVC_BODY;

  control_loss =
      control_loss + (fabs(A_TVC_BODY(1)) + fabs(A_TVC_BODY(2))) * int_step;
}

void Rocket_Flight_DM::propagate_gravityloss(double int_step) {
  // calculate gravity loss
  double gravg;
  data_exchang->hget("gravg", &gravg);

  gravity_loss = gravity_loss + gravg * sin(get_thtvdx() * RAD) * int_step;
}

double Rocket_Flight_DM::calculate_betaix(arma::vec3 VBIB) {
  double dvbi = norm(VBIB);
  return asin(VBIB(1) / dvbi) * DEG;
}

arma::vec3 Rocket_Flight_DM::calculate_fspb(arma::vec3 FAPB_in, double vmass) {
  /* Stored Value due to coherence with other models */
  return FAPB_in * (1. / vmass);
}

double Rocket_Flight_DM::calculate_alppx(arma::vec3 VBAB_in, double dvba) {
  // incidence angles in load factor plane (aeroballistic)
  double dum = VBAB_in(0) / dvba;

  if (fabs(dum) > 1) dum = 1 * sign(dum);
  double alpp = acos(dum);

  return alpp * DEG;
}

double Rocket_Flight_DM::calculate_phipx(arma::vec3 VBAB_in) {
  double phip = 0;
  // Changed according to comments, not original code, refer commit:b613a992
  if (VBAB_in(1) == 0 && VBAB_in(2) == 0) {
    phip = 0.;
  } else if (fabs(VBAB_in(1)) < arma::datum::eps) {
    // note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
    //      to prevent oscillations
    if (VBAB_in(2) > 0) phip = 0;
    if (VBAB_in(2) < 0) phip = PI;
  } else {
    phip = atan2(VBAB_in(1), VBAB_in(2));
  }

  return phip * DEG;
}

double Rocket_Flight_DM::calculate_alphax(arma::vec3 VBAB_in) {
  double alpha = atan2(VBAB_in(2), VBAB_in(0));
  return alpha * DEG;
}

double Rocket_Flight_DM::calculate_betax(arma::vec3 VBAB_in, double dvba) {
  double beta = asin(VBAB_in(1) / dvba);
  return beta * DEG;
}

void Rocket_Flight_DM::propagate_WBIB(double int_step, arma::vec3 FMB_in,
                                      arma::mat33 IBBB) {
  // integrating the angular velocity acc wrt the inertial frame in body coord
  // Using Armadillo solve for higher accuracy, otherwise will faile the 1ppm
  // test
  INTEGRATE_MAT(WBIB, arma::solve(IBBB, (FMB_in - skew_sym(this->WBIB) * IBBB *
                                                      this->WBIB)));
}

arma::vec3 Rocket_Flight_DM::calculate_WBII(arma::mat33 TBI_in) {
  return trans(TBI_in) * this->WBIB;
}

arma::vec3 Rocket_Flight_DM::calculate_WBEB(arma::mat33 TBI_in) {
  return this->WBIB - TBI_in * this->WEII;
}

double Rocket_Flight_DM::get_psibdx() {
  double cthtbd = 0;

  get_thtbdx_in(cthtbd);

  // yaw angle: 'psibd'
  double cpsi = TBD(0, 0) / cthtbd;
  if (fabs(cpsi) > 1) {
    cpsi = 1 * sign(cpsi);
  }
  psibd = acos(cpsi) * sign(TBD(0, 1));

  return DEG * psibd;
}

double Rocket_Flight_DM::get_thtbdx() {
  double cthtbd = 0;
  return get_thtbdx_in(cthtbd);
}

double Rocket_Flight_DM::get_thtbdx_in(double &cthtbd) {
  // *geodetic _Euler_ angles
  // pitch angle: 'thtbd'
  // note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
  //      forced to be a small positive number to prevent division by zero
  if (fabs(TBD(0, 2)) < 1) {
    thtbd = asin(-TBD(0, 2));
    cthtbd = cos(thtbd);
  } else {
    thtbd = PI / 2 * sign(-TBD(0, 2));
    cthtbd = 1e-10;
  }
  // thtbd = asin(  2 * (TBDQ(0) * TBDQ(2) - TBDQ(1) * TBDQ(3)));

  return DEG * thtbd;
}

double Rocket_Flight_DM::get_phibdx() {
  double cthtbd = 0;

  get_thtbdx_in(cthtbd);

  // roll angle: 'phibdc'
  double cphi = TBD(2, 2) / cthtbd;
  if (fabs(cphi) > 1) {
    cphi = 1 * sign(cphi);
  }
  phibd = acos(cphi) * sign(TBD(1, 2));
  // phibd = atan2( 2. * (TBDQ(2) * TBDQ(3) + TBDQ(0) * TBDQ(1) ), (1. - 2. *
  // (TBDQ(2) * TBDQ(2) + TBDQ(1) * TBDQ(1))));

  return DEG * phibd;
}

void Rocket_Flight_DM::update_diagnostic_attributes(double int_step) {
  this->thtbdx = get_thtbdx();
  this->psibdx = get_psibdx();
  this->phibdx = get_phibdx();

  this->ppx = get_ppx();
  this->qqx = get_qqx();
  this->rrx = get_rrx();

  _dbi = get_dbi();
  _dvbi = get_dvbi();

  _dvbe = get_dvbe();
  _psivdx = get_psivdx();
  _thtvdx = get_thtvdx();

  double vbed1 = get_VBED()[0];
  double vbed2 = get_VBED()[1];
  _grndtrck +=
      sqrt(vbed1 * vbed1 + vbed2 * vbed2) * int_step * REARTH / get_dbi();
  _gndtrkmx = 0.001 * _grndtrck;
  _gndtrnmx = NMILES * _grndtrck;

  _ayx = FSPB(1) / AGRAV;
  _anx = -FSPB(2) / AGRAV;

  if (liftoff == 1) {
    // T.M. of geographic velocity wrt geodetic coordinates
    TVD = build_psivg_thtvg_TM(_psivdx * RAD, _thtvdx * RAD);
    orbital(SBII, VBII, get_dbi());
  }
}

void Rocket_Flight_DM::orbital(arma::vec3 SBII_in, arma::vec3 VBII_in,
                               double dbi_in) {
  // calculate orbital elements
  cadorbin_flag =
      cad::orb_in(_semi_major, _eccentricity, _inclination, _lon_anodex,
                  _arg_perix, _true_anomx, SBII_in, VBII_in);
  _ha = (1. + _eccentricity) * _semi_major - REARTH;
  _hp = (1. - _eccentricity) * _semi_major - REARTH;
  _ref_alt = dbi_in - REARTH;
}

void Rocket_Flight_DM::aux_calulate(arma::mat33 TEI, arma::mat33 TBI_in) {
  double lon, lat, al;
  arma::mat33 TBL;

  // angular velocity wrt inertial frame in inertial coordinates
  this->WBII = calculate_WBII(TBI_in);

  // angular velocity wrt Earth in body coordinates
  this->WBEB = calculate_WBEB(TBI_in);

  SBEE_old = SBEE;
  VBEE_old = VBEE;
  ABEE_old = ABEE;

  ABIB = TBI_in * ABII;
  SBEE = TEI * SBII;                      // Calculate position in ECEF
  VBEE = TEI * VBII - cross(WEII, SBEE);  // Calculate velocity in ECEF
  ABEE = TEI * ABII - cross(WEII, VBEE) - cross(WEII, VBEE) -
         cross(WEII, cross(WEII, SBEE));
  JBEE = TEI * JBII - cross(WEII, cross(WEII, cross(WEII, SBEE))) -
         cross(WEII, cross(WEII, VBEE)) - cross(WEII, ABEE);

  // Calculate lon lat alt
  std::tie(lon, lat, al) = cad::geo84_in(SBII, TEI);
  this->lonx = lon * DEG;
  this->latx = lat * DEG;
  this->alt = al;
  // std::cout<<alt<<std::endl;
  if (liftoff == 1)
    assert(alt >= 0.0 && " *** Stop: Ground impact detected !! *** ");

  TDI = cad::tdi84(lon, lat, al, TEI);
  TDE = cad::tde84(lon, lat, al);
  TGI = cad::tgi84(lon, lat, al, TEI);
  TBL = TBI_in * trans(TLI);
  LT_euler = euler_angle(TBL);

  ABID = TDI * ABII;
  VBED = TDE * VBEE;
}

void Rocket_Flight_DM::RK4F(std::vector<arma::vec> Var_in,
                            std::vector<arma::vec> &Var_out) {
  double vmass;
  double thrust;
  data_exchang->hget("vmass", &vmass);
  data_exchang->hget("thrust", &thrust);
  arma::mat33 TEI;
  arma::vec3 GRAVG;
  data_exchang->hget("GRAVG", GRAVG);
  data_exchang->hget("TEI", TEI);
  arma::vec4 TBID_Q_NEW;
  arma::vec3 rhoC_IMU;

  VBIIP = Var_in[0];
  SBIIP = Var_in[1];
  WBIB = Var_in[2];
  TBI_Q = Var_in[3];

  collect_forces_and_propagate();

  arma::vec3 XCG_0;

  data_exchang->hget("XCG_0", XCG_0);
  WBIBD = ddang_1;

  rhoC_IMU(0) = -XCG_0(0) - (reference_point);
  rhoC_IMU(1) = 0.0;
  rhoC_IMU(2) = 0.0;
  arma::vec3 ddrhoC_IMU =
      cross(WBIBD, rhoC_IMU) + cross(WBIB, cross(WBIB, rhoC_IMU));
  NEXT_ACC = ddrP_1;

  if (liftoff == 0) {
    if ((thrust - norm(vmass * GRAVG)) > 0) {
      liftoff = 1;
    }
    NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * (SBIIP)))));
    FSPB = QuaternionRotation(
        TBI_Q, (-GRAVG + NEXT_ACC +
                QuaternionRotation(QuaternionInverse(TBI_Q),
                                   ddrhoC_IMU)));  // Strapdown Analytics 4.3-14
  } else {
    FSPB = QuaternionRotation(
        TBI_Q,
        (NEXT_ACC - GRAVG +
         QuaternionRotation(QuaternionInverse(TBI_Q),
                            ddrhoC_IMU)));  //+ TBI * GRAVG;  // FSPB: body
                                            // force include gravity acc
  }
  /* Prepare for orthonormalization */
  double quat_metric = TBI_Q(0) * TBI_Q(0) + TBI_Q(1) * TBI_Q(1) +
                       TBI_Q(2) * TBI_Q(2) + TBI_Q(3) * TBI_Q(3);
  double erq = 1. - quat_metric;

  /* Calculate Previous states */  //  Zipfel p.141
  TBID_Q_NEW(0) =
      0.5 * (-WBIB(0) * TBI_Q(1) - WBIB(1) * TBI_Q(2) - WBIB(2) * TBI_Q(3)) +
      50. * erq * TBI_Q(0);
  TBID_Q_NEW(1) =
      0.5 * (WBIB(0) * TBI_Q(0) + WBIB(2) * TBI_Q(2) - WBIB(1) * TBI_Q(3)) +
      50. * erq * TBI_Q(1);
  TBID_Q_NEW(2) =
      0.5 * (WBIB(1) * TBI_Q(0) - WBIB(2) * TBI_Q(1) + WBIB(0) * TBI_Q(3)) +
      50. * erq * TBI_Q(2);
  TBID_Q_NEW(3) =
      0.5 * (WBIB(2) * TBI_Q(0) + WBIB(1) * TBI_Q(1) - WBIB(0) * TBI_Q(2)) +
      50. * erq * TBI_Q(3);

  Var_out[0] = NEXT_ACC;
  Var_out[1] = VBIIP;
  Var_out[2] = WBIBD;
  Var_out[3] = TBID_Q_NEW;

  data_exchang->hset("NEXT_ACC", NEXT_ACC);
  data_exchang->hset("liftoff", liftoff);
  data_exchang->hset("FSPB", FSPB);
}

double Rocket_Flight_DM::get_ppx() { return this->WBEB(0) * DEG; }
double Rocket_Flight_DM::get_qqx() { return this->WBEB(1) * DEG; }
double Rocket_Flight_DM::get_rrx() { return this->WBEB(2) * DEG; }
double Rocket_Flight_DM::get_dbi() { return norm(SBII); }
double Rocket_Flight_DM::get_dvbi() { return norm(VBII); }
double Rocket_Flight_DM::get_dvbe() { return pol_from_cart(get_VBED())(0); }
double Rocket_Flight_DM::get_thtvdx() {
  return DEG * pol_from_cart(get_VBED())(2);
}
double Rocket_Flight_DM::get_psivdx() {
  return DEG * pol_from_cart(get_VBED())(1);
}

arma::vec3 Rocket_Flight_DM::get_VBED() {
  return TDI * (VBII - WEII_skew * SBII);
}

int Rocket_Flight_DM::enqueue_to_simgen_buffer(struct icf_ctrlblk_t *C,
                                               double ext_porlation) {
  struct simgen_motion_data_t motion_info;
  double(*pos)[3];
  double(*vel)[3];
  double(*accel)[3];
  pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
  vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
  accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
  motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
  motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
  motion_info.vehicle_id = 1;
  memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
  memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
  memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
  motion_info.jerk_xyz[0] = 0.0;
  motion_info.jerk_xyz[1] = 0.0;
  motion_info.jerk_xyz[2] = 0.0;
  motion_info.heb[0] = psibd;
  motion_info.heb[1] = thtbd;
  motion_info.heb[2] = phibd;
  memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
  motion_info.angular_acceleration[0] = 0.0;
  motion_info.angular_acceleration[1] = 0.0;
  motion_info.angular_acceleration[2] = 0.0;
  motion_info.angular_jerk[0] = 0.0;
  motion_info.angular_jerk[1] = 0.0;
  motion_info.angular_jerk[2] = 0.0;
  icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info,
                 sizeof(struct simgen_motion_data_t));
  return 0;
}

int Rocket_Flight_DM::stand_still_motion_data(struct icf_ctrlblk_t *C,
                                              double ext_porlation) {
  struct simgen_motion_data_t motion_info;
  double(*pos)[3];
  double(*vel)[3];
  double(*accel)[3];
  pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
  vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
  accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
  motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
  motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
  motion_info.vehicle_id = 1;
  memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
  memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
  memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
  motion_info.jerk_xyz[0] = 0.0;
  motion_info.jerk_xyz[1] = 0.0;
  motion_info.jerk_xyz[2] = 0.0;
  /* Heading, Yaw, Z axis, Down */
  motion_info.heb[0] = 1.5707963268;
  /* Elevation, Pitch, Y axis, East */
  motion_info.heb[1] = 1.5707963268;
  /* Bank, Roll, X axis, North */
  motion_info.heb[2] = 0.0;
  memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
  motion_info.angular_acceleration[0] = 0.0;
  motion_info.angular_acceleration[1] = 0.0;
  motion_info.angular_acceleration[2] = 0.0;
  motion_info.angular_jerk[0] = 0.0;
  motion_info.angular_jerk[1] = 0.0;
  motion_info.angular_jerk[2] = 0.0;
  icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info,
                 sizeof(struct simgen_motion_data_t));
  return 0;
}

void Rocket_Flight_DM::Interpolation_Extrapolation(double T, double int_step,
                                                   double ext_porlation) {
  arma::vec3 A, B, C, D, E, F;
  // double t;
  // Interpolation_Extrapolation_flag++;
  // if (Interpolation_Extrapolation_flag == 5) {
  //     Interpolation_Extrapolation_flag = 0;
  // }
  // t = Interpolation_Extrapolation_flag * int_step;

  A = SBEE_old;
  B = VBEE_old;
  // C = ABEE_old / 2.0;
  // D = (-20.0 * SBEE_old + 20.0 * SBEE - 12.0 * T * VBEE_old - 8.0 * T * VBEE
  // - 3.0 * T * T * ABEE_old + T * T * ABEE) / (2.0 * T * T * T); E = (30.0 *
  // SBEE_old - 30.0 * SBEE + 16.0 * T * VBEE_old + 14.0 * T * VBEE + 3.0 * T *
  // T * ABEE_old - 2.0 * T * T * ABEE) / (2.0 * T * T * T * T); F = (-12.0 *
  // SBEE_old + 12.0 * SBEE - 6.0 * T * VBEE_old - 6.0 * T * VBEE - T * T *
  // ABEE_old + T * T * ABEE) / (2.0 * T * T * T * T * T);

  // SBEE_test = A + B * t + C * t * t + D * t * t * t
  //             + E * t * t * t * t + F * t * t * t * t * t;
  // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t + 4.0 * E * t * t * t
  //             + 5.0 * F * t * t * t * t;
  // ABEE_test = 2.0 * C + 6.0 * D * t + 12.0 * E * t * t + 20.0 * F * t * t *
  // t;

  // C = 3.0 * (SBEE - SBEE_old) / (T * T) - (VBEE + 2.0 * VBEE_old) / T;
  // D = 2.0 * (SBEE_old - SBEE) / (T * T * T) + (VBEE + VBEE_old) / (T * T);

  // SBEE_test = A + B * t + C * t * t + D * t * t * t;
  // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t
  SBEE_test = SBEE + ((SBEE - A) / T) * ext_porlation * 1000.0 * int_step;
  VBEE_test = VBEE + ((VBEE - B) / T) * ext_porlation * 1000.0 * int_step;
}

arma::vec3 Rocket_Flight_DM::euler_angle(arma::mat33 TBD_in) {
  double psibdc(0), thtbdc(0), phibdc(0);
  double cthtbd(0);

  double mroll = 0;

  double tbd13 = TBD_in(0, 2);
  double tbd11 = TBD_in(0, 0);
  double tbd33 = TBD_in(2, 2);
  double tbd12 = TBD_in(0, 1);
  double tbd23 = TBD_in(1, 2);

  arma::vec3 euler_ang;
  // *geodetic Euler angles
  // computed pitch angle: 'thtbdc'
  // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
  // forced to be a small positive number to prevent division by zero
  if (fabs(tbd13) < 1) {
    thtbdc = asin(-tbd13);
    cthtbd = cos(thtbdc);
  } else {
    thtbdc = PI / 2 * sign(-tbd13);
    cthtbd = EPS;
  }
  // computed yaw angle: 'psibdc'
  double cpsi = tbd11 / cthtbd;
  if (fabs(cpsi) > 1) cpsi = 1 * sign(cpsi);
  psibdc = acos(cpsi) * sign(tbd12);

  // computed roll angle: 'phibdc'
  double cphi = tbd33 / cthtbd;
  if (fabs(cphi) > 1) cphi = 1 * sign(cphi);

  // selecting the Euler roll angle of flight mechanics (not for thtbdc=90 or
  // =-90deg)
  if (mroll == 0 || mroll == 1)
    // roll feedback for right side up
    phibdc = acos(cphi) * sign(tbd23);
  else if (mroll == 2)
    // roll feedback for inverted flight
    phibdc = acos(-cphi) * sign(-tbd23);

  euler_ang(0) = phibdc;
  euler_ang(1) = thtbdc;
  euler_ang(2) = psibdc;

  return euler_ang;
}

void Rocket_Flight_DM::set_DOF(int ndof) { DOF = ndof; }
void Rocket_Flight_DM::set_aero_flag(unsigned int in) { Aero_flag = in; }

void Rocket_Flight_DM::collect_forces_and_propagate() {
  double *ff, *x;
  int check(0);
  ff = new double[7];
  x = new double[7];
  /*****************input from another module*******************/

  data_exchang->hget("XCG", rhoC_1);
  dang_1 = WBIB;

  rhoC_1(0) = rhoC_1(0) - reference_point;

  calculate_I1();
  gamma_beta();
  Gravity_Q();
  if (Aero_flag == 1) {
    AeroDynamics_Q();
  }

  for (int i = 0; i < DOF; i++) {
    x[i + 1] = 0.0;
  }

  broydn(x, DOF, &check);

  funcv(DOF, x, ff);
  for (int i = 0; i < 3; i++) {
    ddrP_1(i) = x[i + 1];
    ddang_1(i) = x[i + 4];
  }

  ddrhoC_1 = cross(ddang_1, rhoC_1) +
             cross(dang_1, cross(dang_1, rhoC_1));  // Eq.(5-12)

  delete[] ff;
  delete[] x;
}

void Rocket_Flight_DM::gamma_beta() {
  // 1st generalized coordinate q1
  // Velocity coefficient
  gamma_b1_q1(0) = 1.0;
  gamma_b1_q1(1) = 0.0;
  gamma_b1_q1(2) = 0.0;

  // 2nd generalized coordinate q2
  gamma_b1_q2(0) = 0.0;
  gamma_b1_q2(1) = 1.0;
  gamma_b1_q2(2) = 0.0;

  // 3rd generalized coordinate q3
  gamma_b1_q3(0) = 0.0;
  gamma_b1_q3(1) = 0.0;
  gamma_b1_q3(2) = 1.0;

  // Angular velocity coefficients
  beta_b1_q4(0) = 1.0;
  beta_b1_q4(1) = 0.0;
  beta_b1_q4(2) = 0.0;

  // Angular velocity coefficients
  beta_b1_q5(0) = 0.0;
  beta_b1_q5(1) = 1.0;
  beta_b1_q5(2) = 0.0;

  // Angular velocity coefficients
  beta_b1_q6(0) = 0.0;
  beta_b1_q6(1) = 0.0;
  beta_b1_q6(2) = 1.0;
}

void Rocket_Flight_DM::AeroDynamics_Q() {
  double pdynmc;
  double refa;
  double refd;
  double cy;
  double cll;
  double clm;
  double cln;
  double cx;
  double cz;
  arma::vec3 XCG;

  data_exchang->hget("pdynmc", &pdynmc);
  data_exchang->hget("refa", &refa);
  data_exchang->hget("refd", &refd);
  data_exchang->hget("cy", &cy);
  data_exchang->hget("cll", &cll);
  data_exchang->hget("clm", &clm);
  data_exchang->hget("cln", &cln);
  data_exchang->hget("cx", &cx);
  data_exchang->hget("cz", &cz);
  data_exchang->hget("XCG", XCG);

  arma::vec3 rhoCG;
  rhoCG(0) = XCG(0) - (reference_point);
  rhoCG(1) = 0.0;
  rhoCG(2) = 0.0;

  // total non-gravitational forces
  FAPB = pdynmc * refa * arma::vec({cx, cy, cz});

  // aerodynamic moment
  FMAB = pdynmc * refa * refd * arma::vec({cll, clm, cln});

  Q_Aero(0) =
      dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB), gamma_b1_q1);
  Q_Aero(1) =
      dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB), gamma_b1_q2);
  Q_Aero(2) =
      dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB), gamma_b1_q3);
  Q_Aero(3) = dot(FMAB, beta_b1_q4) +
              dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB),
                  -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                      cross_matrix(rhoCG) * beta_b1_q4));
  Q_Aero(4) = dot(FMAB, beta_b1_q5) +
              dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB),
                  -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                      cross_matrix(rhoCG) * beta_b1_q5));
  Q_Aero(5) = dot(FMAB, beta_b1_q6) +
              dot(QuaternionRotation(QuaternionTranspose(TBI_Q), FAPB),
                  -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                      cross_matrix(rhoCG) * beta_b1_q6));
}

void Rocket_Flight_DM::Gravity_Q() {
  arma::vec3 GRAVG;
  double vmass;

  data_exchang->hget("GRAVG", GRAVG);
  data_exchang->hget("vmass", &vmass);

  arma::vec3 Fg;
  Fg = vmass * GRAVG;

  if (liftoff == 1) {
    Q_G(0) = dot(Fg, gamma_b1_q1);
    Q_G(1) = dot(Fg, gamma_b1_q2);
    Q_G(2) = dot(Fg, gamma_b1_q3);
    Q_G(3) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q4));
    Q_G(4) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q5));
    Q_G(5) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q6));
  } else {
    Q_G(0) = dot(Fg, gamma_b1_q1);
    Q_G(1) = dot(Fg, gamma_b1_q2);
    Q_G(2) = dot(Fg, gamma_b1_q3);
    Q_G(3) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q4));
    Q_G(4) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q5));
    Q_G(5) = dot(Fg, -QuaternionRotation(QuaternionTranspose(TBI_Q),
                                         cross_matrix(rhoC_1) * beta_b1_q6));
  }
}

void Rocket_Flight_DM::calculate_I1() {
  double vmass;
  arma::mat33 IBBB;

  data_exchang->hget("vmass", &vmass);
  data_exchang->hget("IBBB", IBBB);

  I1(0, 0) =
      IBBB(0, 0) + (vmass) * (rhoC_1(1) * rhoC_1(1) + rhoC_1(2) * rhoC_1(2));
  I1(0, 1) = IBBB(0, 1) - (vmass)*rhoC_1(0) * rhoC_1(1);
  I1(0, 2) = IBBB(0, 2) - (vmass)*rhoC_1(0) * rhoC_1(2);
  I1(1, 0) = IBBB(1, 0) - (vmass)*rhoC_1(1) * rhoC_1(0);
  I1(1, 1) =
      IBBB(1, 1) + (vmass) * (rhoC_1(2) * rhoC_1(2) + rhoC_1(0) * rhoC_1(0));
  I1(1, 2) = IBBB(1, 2) - (vmass)*rhoC_1(1) * rhoC_1(2);
  I1(2, 0) = IBBB(2, 0) - (vmass)*rhoC_1(2) * rhoC_1(0);
  I1(2, 1) = IBBB(2, 1) - (vmass)*rhoC_1(2) * rhoC_1(1);
  I1(2, 2) =
      IBBB(2, 2) + (vmass) * (rhoC_1(0) * rhoC_1(0) + rhoC_1(1) * rhoC_1(1));
}

void Rocket_Flight_DM::funcv(int n, double *x, double *ff) {
  double m1;
  arma::vec6 Q_TVC, Q_RCS;
  data_exchang->hget("vmass", &m1);
  data_exchang->hget("Q_TVC", Q_TVC);
  data_exchang->hget("Q_RCS", Q_RCS);

  for (int i = 0; i < 3; i++) {
    ddrP_1(i) = x[i + 1];
    ddang_1(i) = x[i + 4];
  }

  ddrhoC_1 = cross(ddang_1, rhoC_1) +
             cross(dang_1, cross(dang_1, rhoC_1));  // Eq.(5-12)
  //  Eq.(5-19)
  p_b1_ga =
      m1 * (ddrP_1 + QuaternionRotation(QuaternionTranspose(TBI_Q), ddrhoC_1));
  p_b1_be = I1 * ddang_1 + cross_matrix(dang_1) * I1 * dang_1 +
            m1 * cross_matrix(rhoC_1) * QuaternionRotation(TBI_Q, ddrP_1);
  f(0) = dot(p_b1_ga, gamma_b1_q1) - (Q_G(0) + Q_TVC(0) + Q_Aero(0) + Q_RCS(0));
  f(1) = dot(p_b1_ga, gamma_b1_q2) - (Q_G(1) + Q_TVC(1) + Q_Aero(1) + Q_RCS(1));
  f(2) = dot(p_b1_ga, gamma_b1_q3) - (Q_G(2) + Q_TVC(2) + Q_Aero(2) + Q_RCS(2));
  f(3) = dot(p_b1_be, beta_b1_q4) - (Q_G(3) + Q_TVC(3) + Q_Aero(3) + Q_RCS(3));
  f(4) = dot(p_b1_be, beta_b1_q5) - (Q_G(4) + Q_TVC(4) + Q_Aero(4) + Q_RCS(4));
  f(5) = dot(p_b1_be, beta_b1_q6) - (Q_G(5) + Q_TVC(5) + Q_Aero(5) + Q_RCS(5));

  for (int i = 0; i < n; i++) {
    ff[i + 1] = f(i);
  }
}

void Rocket_Flight_DM::broydn(double x[], int n, int *check) {
  int i, j, k, restrt, sing, skip;
  double den, fff, fold, stpmax, sum, temp, test, *c, *d, *fvcold;
  double *g, *p, **qt, **r, *s, *t, *w, *xold;
  c = dvector(1, n);
  d = dvector(1, n);
  fvcold = dvector(1, n);
  g = dvector(1, n);
  p = dvector(1, n);
  qt = dmatrix(1, n, 1, n);
  r = dmatrix(1, n, 1, n);
  s = dvector(1, n);
  t = dvector(1, n);
  w = dvector(1, n);
  xold = dvector(1, n);
  fvec = dvector(1, n);
  nn = n;
  fff = f_min(x);
  test = 0.0;
  for (i = 1; i <= n; i++)
    if (fabs(fvec[i]) > test) test = fabs(fvec[i]);
  if (test < 0.01 * TOLF) {
    *check = 0;
    FREERETURN
  }
  for (sum = 0.0, i = 1; i <= n; i++) sum += DSQR(x[i]);
  stpmax = STPMX * DMAX(sqrt(sum), (double)n);
  restrt = 1;
  for (its = 1; its <= MAXITS; its++) {
    if (restrt) {
      fdjac(n, x, fvec, r);
      qrdcmp(r, n, c, d, &sing);
      if (sing) nrerror("singular Jacobian in broydn : qrdcmp");
      for (i = 1; i <= n; i++) {
        for (j = 1; j <= n; j++) qt[i][j] = 0.0;
        qt[i][i] = 1.0;
      }
      for (k = 1; k < n; k++) {
        if (c[k]) {
          for (j = 1; j <= n; j++) {
            sum = 0.0;
            for (i = k; i <= n; i++) sum += r[i][k] * qt[i][j];
            sum /= c[k];
            for (i = k; i <= n; i++) qt[i][j] -= sum * r[i][k];
          }
        }
      }
      for (i = 1; i <= n; i++) {
        r[i][i] = d[i];
        for (j = 1; j < i; j++) r[i][j] = 0.0;
      }
    } else {
      for (i = 1; i <= n; i++) s[i] = x[i] - xold[i];
      for (i = 1; i <= n; i++) {
        for (sum = 0.0, j = i; j <= n; j++) sum += r[i][j] * s[j];
        t[i] = sum;
      }
      skip = 1;
      for (i = 1; i <= n; i++) {
        for (sum = 0.0, j = 1; j <= n; j++) sum += qt[j][i] * t[j];
        w[i] = fvec[i] - fvcold[i] - sum;
        if (fabs(w[i]) >= EPS * (fabs(fvec[i]) + fabs(fvcold[i]))) {
          skip = 0;
        } else {
          w[i] = 0.0;
        }
      }
      if (!skip) {
        for (i = 1; i <= n; i++) {
          for (sum = 0.0, j = 1; j <= n; j++) sum += qt[i][j] * w[j];
          t[i] = sum;
        }
        for (den = 0.0, i = 1; i <= n; i++) den += DSQR(s[i]);
        for (i = 1; i <= n; i++) s[i] /= den;
        qrupdt(r, qt, n, t, s);
        for (i = 1; i <= n; i++) {
          if (r[i][i] == 0.0) nrerror("r singular in broydn : qrupdt");
          d[i] = r[i][i];
        }
      }
    }
    for (i = 1; i <= n; i++) {
      for (sum = 0.0, j = 1; j <= n; j++) sum += qt[i][j] * fvec[j];
      p[i] = -sum;
    }
    for (i = n; i >= 1; i--) {
      for (sum = 0.0, j = 1; j <= i; j++) sum -= r[j][i] * p[j];
      g[i] = sum;
    }
    for (i = 1; i <= n; i++) {
      xold[i] = x[i];
      fvcold[i] = fvec[i];
    }
    fold = fff;
    rsolv(r, n, d, p);
    lnsrch(n, xold, fold, g, p, x, &fff, stpmax, check);
    test = 0.0;
    for (i = 1; i <= n; i++)
      if (fabs(fvec[i]) > test) test = fabs(fvec[i]);
    if (test < TOLF) {
      *check = 0;
      FREERETURN
    }
    if (*check) {
      if (restrt) {
        FREERETURN
      } else {
        test = 0.0;
        den = DMAX(fff, 0.5 * n);
        for (i = 1; i <= n; i++) {
          temp = fabs(g[i]) * DMAX(fabs(x[i]), 1.0) / den;
          if (temp > test) {
            test = temp;
          }
        }
        if (test < TOLMIN) {
          FREERETURN
        } else {
          restrt = 1;
        }
      }
    } else {
      restrt = 0;
      test = 0.0;
      for (i = 1; i <= n; i++) {
        temp = (fabs(x[i] - xold[i])) / DMAX(fabs(x[i]), 1.0);
        if (temp > test) test = temp;
      }
      if (test < EPS) FREERETURN
    }
  }
  nrerror("MAXITS exceeded in broydn");
  FREERETURN
}

void Rocket_Flight_DM::rsolv(double **a, int n, double d[], double b[]) {
  int i, j;
  double sum;

  b[n] /= d[n];
  for (i = n - 1; i >= 1; i--) {
    for (sum = 0.0, j = i + 1; j <= n; j++) sum += a[i][j] * b[j];
    b[i] = (b[i] - sum) / d[i];
  }
}

void Rocket_Flight_DM::fdjac(int n, double x[], double fvec_in[], double **df) {
  int i, j;
  double h, temp, *ff;

  ff = dvector(1, n);
  for (j = 1; j <= n; j++) {
    temp = x[j];
    h = EPS * fabs(temp);
    if (h == 0.0) h = EPS;
    x[j] = temp + h;
    h = x[j] - temp;
    funcv(n, x, ff);
    x[j] = temp;
    for (i = 1; i <= n; i++) {
      df[i][j] = (ff[i] - fvec_in[i]) / h;
    }
  }
  free_dvector(ff, 1, n);
}

double Rocket_Flight_DM::f_min(double x[]) {
  int i;
  double sum;

  funcv(nn, x, fvec);
  for (sum = 0.0, i = 1; i <= nn; i++) sum += DSQR(fvec[i]);
  return 0.5 * sum;
}

void Rocket_Flight_DM::lnsrch(int n, double xold[], double fold, double g[],
                              double p[], double x[], double *f_in,
                              double stpmax, int *check) {
  int i;
  double a, alam, alam2, alamin, b, disc, f2, rhs1, rhs2, slope, sum, temp,
      test, tmplam;

  *check = 0;
  for (sum = 0.0, i = 1; i <= n; i++) sum += p[i] * p[i];
  sum = sqrt(sum);
  if (sum > stpmax) {
    for (i = 1; i <= n; i++) p[i] *= stpmax / sum;
  }
  for (slope = 0.0, i = 1; i <= n; i++) slope += g[i] * p[i];
  if (slope >= 0.0) {
    nrerror("Roundoff problem in lnsrch.");
  }
  test = 0.0;
  for (i = 1; i <= n; i++) {
    temp = fabs(p[i]) / DMAX(fabs(xold[i]), 1.0);
    if (temp > test) {
      test = temp;
    }
  }
  alamin = TOLX / test;
  alam = 1.0;
  for (;;) {
    for (i = 1; i <= n; i++) x[i] = xold[i] + alam * p[i];
    *f_in = f_min(x);
    if (alam < alamin) {
      for (i = 1; i <= n; i++) x[i] = xold[i];
      *check = 1;
      return;
    } else if (*f_in <= fold + ALF * alam * slope) {
      return;
    } else {
      if (alam == 1.0) {
        tmplam = -slope / (2.0 * (*f_in - fold - slope));
      } else {
        rhs1 = *f_in - fold - alam * slope;
        rhs2 = f2 - fold - alam2 * slope;
        a = (rhs1 / (alam * alam) - rhs2 / (alam2 * alam2)) / (alam - alam2);
        b = (-alam2 * rhs1 / (alam * alam) + alam * rhs2 / (alam2 * alam2)) /
            (alam - alam2);
        if (a == 0.0) {
          tmplam = -slope / (2.0 * b);
        } else {
          disc = b * b - 3.0 * a * slope;
          if (disc < 0.0) {
            tmplam = 0.5 * alam;
          } else if (b <= 0.0) {
            tmplam = (-b + sqrt(disc)) / (3.0 * a);
          } else {
            tmplam = -slope / (b + sqrt(disc));
          }
        }
        if (tmplam > 0.5 * alam) {
          tmplam = 0.5 * alam;
        }
      }
    }
    alam2 = alam;
    f2 = *f_in;
    alam = DMAX(tmplam, 0.1 * alam);
  }
}

void Rocket_Flight_DM::qrdcmp(double **a, int n, double *c, double *d,
                              int *sing) {
  int i, j, k;
  double scale, sigma, sum, tau;

  *sing = 0;
  for (k = 1; k < n; k++) {
    scale = 0.0;
    for (i = k; i <= n; i++) scale = DMAX(scale, fabs(a[i][k]));
    if (scale == 0.0) {
      *sing = 1;
      c[k] = d[k] = 0.0;
    } else {
      for (i = k; i <= n; i++) a[i][k] /= scale;
      for (sum = 0.0, i = k; i <= n; i++) sum += DSQR(a[i][k]);
      sigma = SIGN(sqrt(sum), a[k][k]);
      a[k][k] += sigma;
      c[k] = sigma * a[k][k];
      d[k] = -scale * sigma;
      for (j = k + 1; j <= n; j++) {
        for (sum = 0.0, i = k; i <= n; i++) sum += a[i][k] * a[i][j];
        tau = sum / c[k];
        for (i = k; i <= n; i++) a[i][j] -= tau * a[i][k];
      }
    }
  }
  d[n] = a[n][n];
  if (d[n] == 0.0) *sing = 1;
}

void Rocket_Flight_DM::qrupdt(double **r, double **qt, int n, double u[],
                              double v[]) {
  int i, j, k;

  for (k = n; k >= 1; k--) {
    if (u[k]) break;
  }
  if (k < 1) k = 1;
  for (i = k - 1; i >= 1; i--) {
    rotate(r, qt, n, i, u[i], -u[i + 1]);
    if (u[i] == 0.0) {
      u[i] = fabs(u[i + 1]);
    } else if (fabs(u[i]) > fabs(u[i + 1])) {
      u[i] = fabs(u[i]) * sqrt(1.0 + DSQR(u[i + 1] / u[i]));
    } else {
      u[i] = fabs(u[i + 1]) * sqrt(1.0 + DSQR(u[i] / u[i + 1]));
    }
  }
  for (j = 1; j <= n; j++) r[1][j] += u[1] * v[j];
  for (i = 1; i < k; i++) rotate(r, qt, n, i, r[i][i], -r[i + 1][i]);
}

void Rocket_Flight_DM::rotate(double **r, double **qt, int n, int i, double a,
                              double b) {
  int j;
  double c, fact, s, w, y;

  if (a == 0.0) {
    c = 0.0;
    s = (b >= 0.0 ? 1.0 : -1.0);
  } else if (fabs(a) > fabs(b)) {
    fact = b / a;
    c = SIGN(1.0 / sqrt(1.0 + (fact * fact)), a);
    s = fact * c;
  } else {
    fact = a / b;
    s = SIGN(1.0 / sqrt(1.0 + (fact * fact)), b);
    c = fact * s;
  }
  for (j = i; j <= n; j++) {
    y = r[i][j];
    w = r[i + 1][j];
    r[i][j] = c * y - s * w;
    r[i + 1][j] = s * y + c * w;
  }
  for (j = 1; j <= n; j++) {
    y = qt[i][j];
    w = qt[i + 1][j];
    qt[i][j] = c * y - s * w;
    qt[i + 1][j] = s * y + c * w;
  }
}

#undef MAXITS
#undef EPS
#undef TOLF
#undef TOLMIN
#undef TOLX
#undef STPMX
#undef FREERETURN
#undef NRANSI
