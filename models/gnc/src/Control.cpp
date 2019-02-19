#include "Control.hh"

#include "global_constants.hh"
#include "integrate.hh"
#include "math_utility.hh"

Control::Control()
    : VECTOR_INIT(GAINFP, 3),
      VECTOR_INIT(GAINFY, 3),
      MATRIX_INIT(GAINGAM, 3, 1),
      VECTOR_INIT(IBBB0, 3),
      VECTOR_INIT(IBBB1, 3),
      VECTOR_INIT(IBBB2, 3),
      VECTOR_INIT(CONTROLCMD, 3),
      VECTOR_INIT(CMDQ, 4),
      VECTOR_INIT(TCMDQ, 4),
      VECTOR_INIT(delta_euler, 3),
      VECTOR_INIT(euler, 3),
      VECTOR_INIT(WBICBT, 3),
      VECTOR_INIT(CMDG, 3) {
  this->default_data();
  this->perrori = 0.0;
  this->rerrori = 0.0;
  this->yerrori = 0.0;
  this->perror_old = 0.0;
  this->rerror_old = 0.0;
  this->yerror_old = 0.0;
  this->aoaerrori = 0.0;
  this->pitchcmd_new = 0.0;
  this->pitchcmd_old = 0.0;
  this->pitchcmd_out_old = 0.0;
  this->pitchiout_old = 0.0;
  this->pdout_old = 0.0;
  this->rolliout_old = 0.0;
  this->rdout_old = 0.0;
  this->yawiout_old = 0.0;
  this->ydout_old = 0.0;
  thterrort = 0.0;
  perrorit = 0.0;
  perrorpt = 0.0;
  perror_oldt = 0.0;
  pitchiout_oldt = 0.0;
  pdout_oldt = 0.0;
  this->aoaerror_old = 0.0;
  this->aoaiout_old = 0.0;
  this->aoadout_old = 0.0;
}

Control::Control(const Control& other)
    : VECTOR_INIT(GAINFP, 3),
      VECTOR_INIT(GAINFY, 3),
      MATRIX_INIT(GAINGAM, 3, 1),
      VECTOR_INIT(IBBB0, 3),
      VECTOR_INIT(IBBB1, 3),
      VECTOR_INIT(IBBB2, 3),
      VECTOR_INIT(CONTROLCMD, 3),
      VECTOR_INIT(CMDQ, 4),
      VECTOR_INIT(TCMDQ, 4) {
  this->default_data();
}

Control& Control::operator=(const Control& other) {
  if (&other == this) return *this;

  return *this;
}

void Control::default_data() {}

double Control::get_delecx() { return delecx; }
double Control::get_delrcx() { return delrcx; }

void Control::initialize() {
  fmasse = -mdot * 0.05;
  theta_a_cmd = 0.0;
  theta_b_cmd = 0.0;
  theta_c_cmd = 0.0;
  theta_d_cmd = 0.0;
}

///////////////////////////////////////////////////////////////////////////////
// 'control' module
// Member function of class 'Control'
//
// maut = |mauty|mautp|
//
//         mauty = 0 no control, fixed control surfaces
//               = 5 yaw acceleration control for yaw-to-turn
//
//               mautp = 0 no control, fixed control surfaces
//                     = 3 pitch acceleration control
//
// 030520 Created by Peter H Zipfel
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Control::control(double int_step) {
  calculate_xcg_thrust(int_step);

  switch (maut) {
    case NO_CONTROL:
      return;
      break;

    case ACC_CONTROL_ON:
      // calculate_xcg_thrust(int_step);
      aerodynamics_der();
      delecx = control_normal_accel(ancomx, int_step);
      delrcx = control_yaw_accel(alcomx, int_step);
      theta_a_cmd = delecx;
      theta_b_cmd = delrcx;
      break;
    default:
      break;
  }
}

void Control::load_aerotable(const char* filename) {
  aerotable = Datadeck(filename);
}

void Control::atmosphere_use_nasa() {
  atmosphere = new cad::Atmosphere_nasa2002();
}

void Control::Quaternion_cmd(double int_step) {
  // arma::vec4 TBDQ = grab_TBDQ();
  arma::vec3 ANGLECMD;
  // arma::mat33 TBD = grab_TBD();
  arma::vec4 TDBQ;

  arma::mat33 TLI = grab_TBICI();
  arma::mat33 TBIC = grab_TBIC();
  arma::mat33 TBL = TBIC * trans(TLI);

  // double tmp = int_step * (-pitchcmd_old + pitchcmd * RAD);
  // pitchcmd_new = pitchcmd_old +tmp;
  pitchcmd_new =
      0.02439 * pitchcmd + 0.02439 * pitchcmd_old + 0.9512 * pitchcmd_out_old;
  pitchcmd_old = pitchcmd;
  pitchcmd_out_old = pitchcmd_new;

  ANGLECMD(0) = rollcmd * RAD;
  ANGLECMD(1) = pitchcmd_new * RAD;
  ANGLECMD(2) = yawcmd * RAD;

  arma::mat33 TCMD = build_321_rotation_matrix(ANGLECMD);

  TCMDQ = Matrix2Quaternion(TCMD);
  arma::vec4 TBLQ = Matrix2Quaternion(TBL);
  // TDBQ = Quaternion_conjugate(TBDQ);
  TDBQ = Quaternion_conjugate(TBLQ);

  CMDQ = Quaternion_cross(TDBQ, TCMDQ);

  CMDQ = sign(CMDQ(0)) * CMDQ;
}

void Control::set_close_loop_pole(double in1, double in2) {
  zaclp = in1;
  zacly = in2;
}

void Control::set_factor(double in1, double in2) {
  factwaclp = in1;
  factwacly = in2;
}

void Control::set_thtvdcomx(double in) { this->thtvdcomx = in; }
// void Control::set_maut(double in) { this->maut = in; }
void Control::set_delimx(double in) { this->delimx = in; }
void Control::set_drlimx(double in) { this->drlimx = in; }
void Control::set_pgam(double in) { this->pgam = in; }
void Control::set_wgam(double in) { this->wgam = in; }
void Control::set_zgam(double in) { this->zgam = in; }
void Control::set_ierror_zero() {
  this->perrori = 0.0;
  this->rerrori = 0.0;
  this->yerrori = 0.0;
  this->perror_old = 0.0;
  this->pitchiout_old = 0.0;
  this->pdout_old = 0.0;
  this->rerror_old = 0.0;
  this->rolliout_old = 0.0;
  this->rdout_old = 0.0;
  this->yerror_old = 0.0;
  this->yawiout_old = 0.0;
  this->ydout_old = 0.0;
  this->aoaerrori = 0.0;
  this->aoaiout_old = 0.0;
  this->aoadout_old = 0.0;
}
void Control::set_reference_point(double in) { reference_point = in; }
void Control::set_engine_d(double in) { d = in; }

void Control::pitch_down(double pitchcmd, double int_step) {
  double thtbdcx = grab_thtbdcx();
  arma::vec3 WBICB = grab_computed_WBIB();

  thterror = 2.0 * pitchcmd;
  double p1out = thterror * kpp;
  perrori = 0.5 * (kpi * int_step * perror_old + 2.0 * pitchiout_old +
                   kpi * int_step * thterror);
  double dout = (2.0 * kpd * pN * thterror - 2.0 * kpd * pN * perror_old -
                 (int_step * pN - 2.0) * pdout_old) /
                (2.0 + int_step * pN);
  double iout = perrori;
  perrorp = p1out + iout + dout - WBICB(1);
  double p2out = perrorp * kppp;

  CONTROLCMD(1) = p2out;

  perror_old = thterror;
  pitchiout_old = iout;
  pdout_old = dout;
}

void Control::pitch_down_test(double pitchcmd, double int_step) {
  // double thtbdcx = grab_thtbdcx();
  WBICBT = grab_computed_WBIB();

  thterrort = thterror;
  double p1out = thterrort * kpp;
  perrorit = 0.5 * (kpi * int_step * perror_oldt + 2.0 * pitchiout_oldt +
                    kpi * int_step * thterrort);
  double dout = (2.0 * kpd * pN * thterrort - 2.0 * kpd * pN * perror_oldt -
                 (int_step * pN - 2.0) * pdout_oldt) /
                (2.0 + int_step * pN);
  double iout = perrorit;
  perrorpt = (p1out + iout + dout - WBICBT(1)) * kppp;

  perror_oldt = thterrort;
  pitchiout_oldt = iout;
  pdout_oldt = dout;
}

void Control::roll_control(double rollcmd, double int_step) {
  double phibdcx = grab_phibdcx();
  arma::vec3 WBICB = grab_computed_WBIB();

  rollerror = 2.0 * rollcmd;
  double p1out = rollerror * krp;
  rerrori = 0.5 * (kri * int_step * rerror_old + 2.0 * rolliout_old +
                   kri * int_step * rollerror);
  double dout = (2.0 * krd * rN * rollerror - 2.0 * krd * rN * rerror_old -
                 (int_step * rN - 2.0) * rdout_old) /
                (2.0 + int_step * rN);
  double iout = rerrori;
  rerrorp = p1out + iout + dout - WBICB(0);
  double p2out = rerrorp * krpp;

  CONTROLCMD(0) = p2out;

  rerror_old = rollerror;
  rolliout_old = iout;
  rdout_old = dout;
}

void Control::yaw_control(double yawcmd, double int_step) {
  double psibdcx = grab_psibdcx();
  arma::vec3 WBICB = grab_computed_WBIB();

  yawerror = 2.0 * yawcmd;
  double p1out = yawerror * kyp;
  yerrori = 0.5 * (kyi * int_step * yerror_old + 2.0 * yawiout_old +
                   kyi * int_step * yawerror);
  double dout = (2.0 * kyd * yN * yawerror - 2.0 * kyd * yN * yerror_old -
                 (int_step * yN - 2.0) * ydout_old) /
                (2.0 + int_step * yN);
  double iout = yerrori;
  yerrorp = p1out + iout + dout - WBICB(2);
  double p2out = yerrorp * kypp;

  CONTROLCMD(2) = p2out;

  yerror_old = yawerror;
  yawiout_old = iout;
  ydout_old = dout;
}

void Control::AOA_control(double aoacmd, double int_step) {
  double alphacx = grab_alphacx();
  arma::vec3 WBICB = grab_computed_WBIB();

  aoaerror = aoacmd - 0.5 * alphacx * RAD;

  // aoaerror_new = 0.02439 * aoaerror + 0.02439 * aoaerror_smooth_old + 0.9512
  // * aoaerror_out_old; aoaerror_smooth_old = aoaerror; aoaerror_out_old =
  // aoaerror_new;
  aoaerror_new = aoaerror;

  double p1out = aoaerror_new * kaoap;
  aoaerrori = 0.5 * (kaoai * int_step * aoaerror_old + 2.0 * aoaiout_old +
                     kaoai * int_step * aoaerror_new);
  // double dout = (2.0 * kaoad * aoaN * aoaerror_new - 2.0 * kaoad * aoaN *
  // aoaerror_old - (int_step * aoaN - 2.0) * aoadout_old) / (2.0 + int_step *
  // aoaN);
  double dout = kaoad / int_step * (aoaerror_new - aoaerror_old);
  double iout = aoaerrori;
  aoaerrorp = (p1out + iout + dout) - WBICB(1);
  double p2out = aoaerrorp * kaoapp;

  CONTROLCMD(1) = p2out;

  aoaerror_old = aoaerror_new;
  aoaiout_old = iout;
  aoadout_old = dout;
}

void Control::S2_B_pseudo_G(arma::vec3 cmd, double int_step) {
  arma::vec4 anglecmd;
  anglecmd.zeros();
  arma::mat B_pseudo(3, 4);
  arma::mat B_pseudo1(4, 3);
  arma::mat33 G;

  lx = -xcg - (reference_point);
  // B_pseudo(0, 0) = 0.5;
  // B_pseudo(0, 1) = 0.0;
  // B_pseudo(0, 2) = -0.5;
  // B_pseudo(1, 0) = -0.5;
  // B_pseudo(1, 1) = 0.0;
  // B_pseudo(1, 2) = -0.5;
  // B_pseudo(2, 0) = -0.5;
  // B_pseudo(2, 1) = -0.5;
  // B_pseudo(2, 2) = 0.0;
  // B_pseudo(3, 0) = 0.5;
  // B_pseudo(3, 1) = -0.5;
  // B_pseudo(3, 2) = 0.0;
  B_pseudo(0, 0) = 0.5 * thrust * d;
  B_pseudo(0, 1) = -0.5 * thrust * d;
  B_pseudo(0, 2) = -0.5 * thrust * d;
  B_pseudo(0, 3) = 0.5 * thrust * d;
  B_pseudo(1, 0) = 0.0;
  B_pseudo(1, 1) = 0.0;
  B_pseudo(1, 2) = -1.0 * thrust * lx;
  B_pseudo(1, 3) = -1.0 * thrust * lx;
  B_pseudo(2, 0) = -1.0 * thrust * lx;
  B_pseudo(2, 1) = -1.0 * thrust * lx;
  B_pseudo(2, 2) = 0.0;
  B_pseudo(2, 3) = 0.0;

  G(0, 0) = IBBB2(0);
  G(1, 1) = IBBB2(1);
  G(2, 2) = IBBB2(2);

  CMDG = G * cmd;
  B_pseudo1 = pinv(B_pseudo);
  anglecmd = B_pseudo1 * CMDG;

  theta_a_cmd = anglecmd(0);
  theta_c_cmd = anglecmd(1);
  theta_b_cmd = anglecmd(2);
  theta_d_cmd = anglecmd(3);
}

void Control::S3_B_pseudo_G(arma::vec3 cmd, double int_step) {
  arma::vec2 anglecmd;
  anglecmd.zeros();
  arma::mat B_pseudo(2, 3);
  arma::mat33 G;

  B_pseudo(0, 0) = 0.0;
  B_pseudo(0, 1) = 0.0;
  B_pseudo(0, 2) = -1.0;
  B_pseudo(1, 0) = 0.0;
  B_pseudo(1, 1) = -1.0;
  B_pseudo(1, 2) = 0.0;

  lx = -xcg - (reference_point);

  G(0, 0) = 0.0;
  G(1, 1) = IBBB2(1) / (thrust * 4.0 * lx);
  G(2, 2) = IBBB2(2) / (thrust * 4.0 * lx);

  anglecmd = B_pseudo * G * cmd;

  theta_a_cmd = anglecmd(0);  // yaw
  theta_b_cmd = anglecmd(1);  // pitch
                              // theta_b_cmd = anglecmd(2);
                              // theta_c_cmd = anglecmd(3);
}

void Control::set_controller_var(double in1, double in2, double in3, double in4,
                                 double in5, double in6, double in7) {
  vmass0 = in1;
  mdot = in2;
  fmass0 = in3;
  xcg_1 = in4;
  xcg_0 = in5;
  isp = in6;
  fmasse = in7;
}

void Control::set_IBBB0(double in1, double in2, double in3) {
  IBBB0(0) = in1;
  IBBB0(1) = in2;
  IBBB0(2) = in3;
}

void Control::set_IBBB1(double in1, double in2, double in3) {
  IBBB1(0) = in1;
  IBBB1(1) = in2;
  IBBB1(2) = in3;
}

void Control::set_S2_PITCH_DOWN_I() { maut = S2_PITCH_DOWN_I; }
void Control::set_S2_PITCH_DOWN_II() { maut = S2_PITCH_DOWN_II; }
void Control::set_S2_AOA() { maut = S2_AOA; }
void Control::set_S3_AOA() { maut = S3_AOA; }
void Control::set_S3_PITCH_DOWN() { maut = S3_PITCH_DOWN; }
void Control::set_S2_ROLL_CONTROL() { maut = S2_ROLL_CONTROL; }
void Control::set_NO_CONTROL() { maut = NO_CONTROL; }
void Control::set_acc_control() { maut = ACC_CONTROL_ON; }
void Control::set_aero_coffe(double in1, double in2, double in3) {
  refd = in1;
  refa = in2;
  xcp = in3;
}
void Control::set_feedforward_gain(double in1) { gainp = in1; }
double Control::get_theta_a_cmd() { return theta_a_cmd; }
double Control::get_theta_b_cmd() { return theta_b_cmd; }
double Control::get_theta_c_cmd() { return theta_c_cmd; }
double Control::get_theta_d_cmd() { return theta_d_cmd; }
void Control::get_control_gain(double in1, double in2, double in3, double in4,
                               double in5, double in6, double in7, double in8,
                               double in9, double in10, double in11,
                               double in12, double in13, double in14,
                               double in15, double in16, double in17,
                               double in18, double in19, double in20) {
  kpp = in1;
  kpi = in2;
  kpd = in3;
  kppp = in4;
  pN = in5;

  krp = in6;
  kri = in7;
  krd = in8;
  krpp = in9;
  rN = in10;

  kyp = in11;
  kyi = in12;
  kyd = in13;
  kypp = in14;
  yN = in15;

  kaoap = in16;
  kaoai = in17;
  kaoad = in18;
  kaoapp = in19;
  aoaN = in20;
}
void Control::set_kpp(double in) { this->kpp = in; }
void Control::set_kpi(double in) { this->kpi = in; }
void Control::set_kppp(double in) { this->kppp = in; }
void Control::set_krp(double in) { this->krp = in; }
void Control::set_kri(double in) { this->kri = in; }
void Control::set_krpp(double in) { this->krpp = in; }
void Control::set_kyp(double in) { this->kyp = in; }
void Control::set_kyi(double in) { this->kyi = in; }
void Control::set_kypp(double in) { this->kypp = in; }
void Control::set_kaoap(double in) { this->kaoap = in; }
void Control::set_kaoai(double in) { this->kaoai = in; }
void Control::set_kaoad(double in) { this->kaoad = in; }
void Control::set_kaoapp(double in) { this->kaoapp = in; }
void Control::set_attcmd(double in1, double in2, double in3) {
  this->rollcmd = in1;
  this->pitchcmd = in2;
  this->yawcmd = in3;
}
void Control::set_engnum(double in) { eng_num = in; }

void Control::set_aoacmd(double in) { this->aoacmd = in; }

arma::mat33 Control::build_321_rotation_matrix(arma::vec3 angle) {
  arma::mat33 TM;
  TM(0, 0) = cos(angle(2)) * cos(angle(1));
  TM(0, 1) = sin(angle(2)) * cos(angle(1));
  TM(0, 2) = -sin(angle(1));
  TM(1, 0) = (cos(angle(2)) * sin(angle(1)) * sin(angle(0))) -
             (sin(angle(2)) * cos(angle(0)));
  TM(1, 1) = (sin(angle(2)) * sin(angle(1)) * sin(angle(0))) +
             (cos(angle(2)) * cos(angle(0)));
  TM(1, 2) = cos(angle(1)) * sin(angle(0));
  TM(2, 0) = (cos(angle(2)) * sin(angle(1)) * cos(angle(0))) +
             (sin(angle(2)) * sin(angle(0)));
  TM(2, 1) = (sin(angle(2)) * sin(angle(1)) * cos(angle(0))) -
             (cos(angle(2)) * sin(angle(0)));
  TM(2, 2) = cos(angle(1)) * cos(angle(0));

  return TM;
}

arma::vec3 Control::euler_angle(arma::mat33 TBD) {
  double psibdc(0), thtbdc(0), phibdc(0);
  double cthtbd(0);

  double mroll = 0;

  double tbd13 = TBD(0, 2);
  double tbd11 = TBD(0, 0);
  double tbd33 = TBD(2, 2);
  double tbd12 = TBD(0, 1);
  double tbd23 = TBD(1, 2);

  arma::vec3 euler_ang;
  // *geodetic Euler angles
  // computed pitch angle: 'thtbdc'
  // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
  // forced to be a small positive number to prevent division by zero
  if (fabs(tbd13) < 1) {
    thtbdc = asin(-tbd13);
    cthtbd = cos(thtbdc);
  } else {
    thtbdc = PI / 2. * sign(-tbd13);
    cthtbd = EPS;
  }
  // computed yaw angle: 'psibdc'
  double cpsi = tbd11 / cthtbd;
  if (fabs(cpsi) > 1) cpsi = 1. * sign(cpsi);
  psibdc = acos(cpsi) * sign(tbd12);

  // computed roll angle: 'phibdc'
  double cphi = tbd33 / cthtbd;
  if (fabs(cphi) > 1) cphi = 1. * sign(cphi);

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

void Control::calculate_xcg_thrust(double int_step) {
  fmasse += mdot * int_step;
  mass_ratio = fmasse / fmass0;
  xcg = xcg_0 + (xcg_1 - xcg_0) * mass_ratio;
  thrust = isp * mdot * AGRAV / eng_num;
  IBBB2 = IBBB0 + (IBBB1 - IBBB0) * mass_ratio;
  vmass = vmass0 - fmasse;
}

void Control::aerodynamics_der() {
  // from aerodynamics module
  double cla(0);
  double clde(0);
  double cyb(0);
  double cydr(0);
  double cllda(0);
  double cllp(0);
  double cma(0);
  double cmde(0);
  double cmq(0);
  double cnb(0);
  double cndr(0);
  double cnr(0);
  double cn0(0);

  // from other modules
  double vmach(0);
  double dvbec = grab_dvbec();
  double gtvc(1.0);
  double altc = grab_altc();
  double phipcx = grab_phipcx();
  double alppcx = grab_alppcx();
  // arma::vec3 WBECB = grab_WBECB();

  //-------------------------------------------------------------------------
  // MOI components
  double ibbb11 = IBBB2(0);
  double ibbb22 = IBBB2(1);
  double ibbb33 = IBBB2(2);

  atmosphere->set_altitude(altc);
  // Dynamics pressure
  pdynmc = 0.5 * atmosphere->get_density() * dvbec * dvbec;
  // mach number
  vmach = fabs(dvbec / atmosphere->get_speed_of_sound());

  // //  transforming body rates from body -> aeroballistic coord.
  // double phip = phipcx * RAD;
  // double cphip = cos(phip);
  // double sphip = sin(phip);
  // double qqax = WBECB(1) * cphip - WBECB(2) * sphip;
  // double rrax = WBECB(1) * sphip + WBECB(2) * cphip;

  // // looking up axial force coefficients
  // ca0 = aerotable.look_up("ca0_vs_mach", vmach, 1);
  // caa = aerotable.look_up("caa_vs_mach", vmach, 1);
  // ca0b = aerotable.look_up("ca0b_vs_mach", vmach, 1);

  // // axial force coefficient
  // if (thrust_state == 1) thrust_on = 1;
  // ca = ca0 + caa * alppcx + thrust_on * ca0b;

  // // looking up normal force coefficients in aeroballistic coord
  // cn0 = aerotable.look_up("cn0_vs_mach_alpha", vmach, alppcx, 1);
  // // normal force coefficient
  // cna = cn0;

  // // looking up pitching moment coefficients in aeroballistic coord
  // clm0 = aerotable.look_up("clm0_vs_mach_alpha", vmach, alppcx, 1);
  double clmq = aerotable.look_up("clmq_vs_mach", vmach, 0);
  // pitching moment coefficient
  // double clmaref = clm0 + clmq * qqax * refd / (2. * dvba);
  // clma = clmaref - cna * (XCP(0) - XCG(0)) / refd;

  // Non-dimensional derivatives
  // look up coeff at +- 3 deg, but not beyond tables
  double alplx = alppcx + 3.0;
  double alpmx = alppcx - 3.0;
  if (alpmx < 0.) alpmx = 0.0;

  // calculating normal force dim derivative wrt alpha 'cla'

  double cn0p = aerotable.look_up("cn0_vs_mach_alpha", vmach, alplx, 0);
  double cn0m = aerotable.look_up("cn0_vs_mach_alpha", vmach, alpmx, 0);

  // replacing value from previous cycle, only if within max alpha limit
  // if(alplx<alplimx)
  cla = (cn0p - cn0m) / (alplx - alpmx);

  // calculating pitch moment dim derivative wrt alpha 'cma'

  double clm0p = aerotable.look_up("clm0_vs_mach_alpha", vmach, alplx, 0);
  double clm0m = aerotable.look_up("clm0_vs_mach_alpha", vmach, alpmx, 0);

  // replacing value from previous cycle, only if within max alpha limit
  // if(alppcx<alplimx)
  cma = (clm0p - clm0m) / (alplx - alpmx) - cla * (xcp - xcg) / refd;

  // converting output to be compatible with 'aerodynamics_der()'
  // force
  clde = 0.0;
  cyb = -cla;
  cydr = 0.0;
  // roll
  cllda = 0.0;
  cllp = 0.0;
  // pitch
  cmde = 0.0;
  cmq = clmq;
  // yaw
  cnb = -cma;
  cndr = 0.0;
  cnr = clmq;

  // Dimensional derivatives for pitch plane (converted to 1/rad where required)
  double duml = (pdynmc * refa / vmass) / RAD;
  dla = duml * cla;
  dlde = duml * clde;
  dnd = duml * cn0;
  double dumm = pdynmc * refa * refd / ibbb22;
  dma = dumm * cma / RAD;
  dmq = dumm * (refd / (2. * dvbec)) * cmq;
  dmde = dumm * cmde / RAD;

  // Dimensional derivatives in plane (converted to 1/rad where required)
  double dumy = pdynmc * refa / vmass;
  dyb = dumy * cyb / RAD;
  dydr = dumy * cydr / RAD;
  double dumn = pdynmc * refa * refd / ibbb33;
  dnb = dumn * cnb / RAD;
  dnr = dumn * (refd / (2. * dvbec)) * cnr;
  dndr = dumn * cndr / RAD;

  // Dimensional derivatives in roll (converted to 1/rad where required)
  double dumll = pdynmc * refa * refd / ibbb11;
  dllp = dumll * (refd / (2. * dvbec)) * cllp;
  dllda = dumll * cllda / RAD;

  // TVC control derivatives
  // pitch plane
  dlde = gtvc * thrust / vmass;
  dmde = -(reference_point - xcg) * gtvc * thrust / ibbb33;
  // yaw plane
  dydr = dlde;
  dndr = dmde;
  // static margin in pitch (per chord length 'refd')
  if (cla) stmarg_pitch = -cma / cla;

  // static margin in yaw (per span length 'refd')
  if (cyb) stmarg_yaw = -cnb / cyb;

  // diagnostics: pitch plane roots
  double a11 = dmq;
  double a12(0);
  if (dla) a12 = dma / dla;
  double a21 = dla;
  double a22 = -dla / dvbec;

  double arg = pow((a11 + a22), 2.) - 4. * (a11 * a22 - a12 * a21);
  if (arg >= 0.) {
    wnp = 0.;
    zetp = 0.;
    double dum = a11 + a22;
    realp1 = (dum + sqrt(arg)) / 2.;
    realp2 = (dum - sqrt(arg)) / 2.;
    rpreal = (realp1 + realp2) / 2.;
  } else {
    realp1 = 0.;
    realp2 = 0.;
    wnp = sqrt(a11 * a22 - a12 * a21);
    zetp = -(a11 + a22) / (2. * wnp);
    rpreal = -zetp * wnp;
  }
  // diagnostics: yaw plane roots
  a11 = dnr;
  if (dyb)
    a12 = dnb / dyb;
  else
    a12 = 0.;
  a21 = -dyb;
  a22 = dyb / dvbec;
  arg = pow((a11 + a22), 2.) - 4. * (a11 * a22 - a12 * a21);
  if (arg >= 0.) {
    wny = 0.;
    zety = 0.;
    double dum = a11 + a22;
    realy1 = (dum + sqrt(arg)) / 2.;
    realy2 = (dum - sqrt(arg)) / 2.;
    ryreal = (realy1 + realy2) / 2.;
  } else {
    realy1 = 0.;
    realy2 = 0.;
    wny = sqrt(a11 * a22 - a12 * a21);
    zety = -(a11 + a22) / (2. * wny);
    ryreal = -zety * wny;
  }
}

void Control::set_ancomx(double in) { ancomx = in; }
void Control::set_alcomx(double in) { alcomx = in; }
///////////////////////////////////////////////////////////////////////////////
// Acceleration controller in normal (pitch) plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch control deflection
//
// Return output
//        delecx = pitch control command - deg
// Parameter input
//        ancomx = normal loadfactor command - g
//        int_step = integration step size - s
//
// 021015 Created by Peter H Zipfel
// 060120 Added variable bandwidth, PZi
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Control::control_normal_accel(double ancomx, double int_step) {
  double gainfb1(0);
  double gainfb2(0);
  double gainfb3(0);

  // input from other modules
  double dvbec = grab_dvbec();
  arma::vec3 WBICB = grab_computed_WBIB();
  arma::vec3 FSPCB = grab_FSPCB();
  //-------------------------------------------------------------------------
  // calculating online close loop poles
  waclp = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1. + factwaclp);
  paclp = 0.7 + 1e-5 * (pdynmc - 20e3) * (1. + factwaclp);

  // calculating three feedback gains

  gainfb3 = waclp * waclp * paclp / (dla * dmde);
  gainfb2 = (2. * zaclp * waclp + paclp + dmq - dla / dvbec) / dmde;
  gainfb1 = (waclp * waclp + 2. * zaclp * waclp * paclp + dma +
             dmq * dla / dvbec - gainfb2 * dmde * dla / dvbec) /
                (dla * dmde) -
            gainp;

  // gainfb3=zaclp*zaclp*waclp*waclp*paclp/(dmde*dla);
  // gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
  // gainfb1=(zaclp*zaclp*waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbec-gainfb2*dmde*dla/dvbec)-gainp;

  // pitch loop acceleration control, pitch control command
  double fspb3 = FSPCB(2);
  double zzd_new = AGRAV * ancomx + fspb3;
  zz = integrate(zzd_new, zzd, zz, int_step);
  zzd = zzd_new;
  double dqc =
      -gainfb1 * (-fspb3) - gainfb2 * WBICB(1) + gainfb3 * zz + gainp * zzd;
  double delecx = dqc * DEG;

  // diagnostic output
  GAINFP = arma::vec3({gainfb1, gainfb2, gainfb3});
  //--------------------------------------------------------------------------
  // loading module-variables

  return dqc;
}

///////////////////////////////////////////////////////////////////////////////
// Acceleration controller in lateral (yaw) plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded yaw control deflection
//
// Return output
//        drcx = yaw control command - deg
// Parameter input
//        alcomx = lateral loadfactor command = g
//        int_step = integration step size - s
//
// 050104 Adopted from DRM sim, PZi
// 060120 Added variable bandwidth, PZi
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Control::control_yaw_accel(double alcomx, double int_step) {
  // input from other modules
  double dvbec = grab_dvbec();
  arma::vec3 WBICB = grab_computed_WBIB();
  arma::vec3 FSPCB = grab_FSPCB();

  //-------------------------------------------------------------------------
  // calculating close loop poles
  wacly = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1. + factwacly);
  pacly = 0.7 + 1e-5 * (pdynmc - 20e3) * (1. + factwacly);

  // calculating three feedback gains
  double gainfb3 = -wacly * wacly * pacly / (dyb * dndr);
  double gainfb2 = (2. * zacly * wacly + pacly + dnr + dyb / dvbec) / dndr;
  double gainfb1 = (-wacly * wacly - 2. * zacly * wacly * pacly + dnb +
                    dnr * dyb / dvbec - gainfb2 * dndr * dnb / dvbec) /
                       (dyb * dndr) -
                   gainy;

  // yaw loop acceleration controller, yaw control command
  double fspb2 = FSPCB(1);
  double yyd_new = AGRAV * alcomx - fspb2;
  yy = integrate(yyd_new, yyd, yy, int_step);
  yyd = yyd_new;
  double drc =
      -gainfb1 * fspb2 - gainfb2 * WBICB(2) + gainfb3 * yy + gainy * yyd;
  // double drcx = drc * DEG;

  // diagnostic output
  GAINFY = arma::vec3({gainfb1, gainfb2, gainfb3});
  //--------------------------------------------------------------------------

  return drc;
}
//--------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// Pitch rate controller in pitch plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)s
//
//
// 20160427 Modified for ROCKET6, by Lai
///////////////////////////////////////////////////////////////////////////////
// double Control::control_pitch_rate(double qqdx) {
//     double zrate(0);
//     double dqcx(0);

//     // input from aerodynamic
//     double dla = grab_dla();
//     double dma = grab_dma();
//     double dmq = grab_dmq();
//     double dmde = grab_dmde();
//     double dvbec = grab_dvbec();
//     double dnd = grab_dnd();
//     double qqcx = grab_qqcx();

//     //-------------------------------------------------------------------------
//     // parameters of open loop angular rate transfer function
//     zrate = dla / dvbec - dma * dnd / (dvbec * dmde);
//     double aa = dla / dvbec - dmq;
//     double bb = -dma - dmq * dla / dvbec;

//     // feecback gain of rate loop, given desired close loop 'zetlager'
//     double dum1 = (aa - 2. * zaclp * zaclp * zrate);
//     double dum2 = aa * aa - 4. * zaclp * zaclp * bb;
//     double radix = dum1 * dum1 - dum2;

//     if (radix < 0.)
//         radix = 0.;
//     grate = (-dum1 + sqrt(radix)) / (dmde);

//     // natural frequency of closed rate loop

//     dqcx = qqdx - DEG * grate * qqcx;
//     double delecx = dqcx;

//     return delecx;
// }

// double Control::control_gamma(double thtvdcomx) {
//     // local variables
//     arma::mat33 AA;
//     arma::vec3 BB;
//     arma::mat33 DP;
//     arma::vec3 DD;
//     arma::vec3 HH;

//     // local module-variables
//     // arma::mat GAINGAM(3,3);
//     // GAINGAM.zeros();
//     double gainff = 0;

//     // localizing module-variables
//     // input data
//     double pgam = this->pgam;
//     double wgam = this->wgam;
//     double zgam = this->zgam;
//     // input from other modules
//     double dvbe = grab_dvbe();
//     double dla = grab_dla();
//     double dlde = grab_dlde();
//     double dma = grab_dma();
//     double dmq = grab_dmq();
//     double dmde = grab_dmde();
//     double qqcx = grab_qqcx();
//     double dvbec = grab_dvbec();
//     double thtvdcx = grab_thtvdcx();
//     double thtbdcx = grab_thtbdcx();
//     //--------------------------------------------------------------------------

//     // prevent division by zero
//     if (dvbec == 0) dvbec = dvbe;

//     // building fundamental matrices (body rate, acceleration, fin
//     deflection) AA(0, 0) = dmq; AA(0, 1) = dma; AA(0, 2) = -dma; AA(1, 0)
//     = 1.; AA(1, 1) = 0.0; AA(1, 2) = 0.0; AA(2, 0) = 0.0; AA(2, 1) =
//     dla/dvbec; AA(2, 2) = -dla/dvbec;
//     // AA.build_mat33(dmq,dma,-dma,1.,0.,0.,0.,dla/dvbec,-dla/dvbec);
//     BB(0) = dmde;
//     BB(1) = 0.0;
//     BB(2) = dlde/dvbec;
//     // BB.build_vec3(dmde,0.,dlde/dvbec);

//     // feedback gains from closed-loop pole placement
//     double am = 2.*zgam*wgam+pgam;
//     double bm = wgam*wgam+2.*zgam*wgam*pgam;
//     double cm = wgam*wgam*pgam;
//     // double v11=dmde;
//     // double v12=0.;
//     // double v13=dlde/dvbec;
//     // double v21=dmde*dla/dvbec-dlde*dma/dvbec;
//     // double v22=dmde;
//     // double v23=-dmq*dlde/dvbec;
//     // double v31=0.;
//     // double v32=v21;
//     // double v33=v21;

//     DP(0, 0) = dmde;
//     DP(0, 1) = 0.0;
//     DP(0, 2) = dlde/dvbec;
//     DP(1, 0) = dmde*dla/dvbec-dlde*dma/dvbec;
//     DP(1, 1) = dmde;
//     DP(1, 2) = -dmq*dlde/dvbec;
//     DP(2, 0) = 0.0;
//     DP(2, 1) = dmde*dla/dvbec-dlde*dma/dvbec;
//     DP(2, 2) = dmde*dla/dvbec-dlde*dma/dvbec;
//     // DP.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);

//     DD(0) = am+dmq-dla/dvbec;
//     DD(1) = bm+dma+dmq*dla/dvbec;
//     DD(2) = cm;
//     // DD.build_vec3(am+dmq-dla/dvbec,bm+dma+dmq*dla/dvbec,cm);
//     arma::mat33 DPI = inv(DP);
//     GAINGAM = DPI*DD;

//     // steady-state feed-forward gain to achieve unit gamma response
//     arma::mat33 DUM33 = AA-BB*trans(GAINGAM);
//     arma::mat33 IDUM33 = inv(DUM33);
//     arma::vec DUM3 = IDUM33*BB;

//     HH(0) = 0.0;
//     HH(1) = 0.0;
//     HH(2) = 1.;
//     // HH.build_vec3(0.,0.,1.);
//     double denom = dot(HH, DUM3);
//     gainff = -1./denom;

//     // pitch control command
//     double thtc = gainff*thtvdcomx*RAD;
//     double qqf = GAINGAM(0, 0)*qqcx*RAD;
//     double thtbgf = GAINGAM(1, 0)*thtbdcx*RAD;
//     double thtugf = GAINGAM(2, 0)*thtvdcx*RAD;
//     double delec = thtc-(qqf+thtbgf+thtugf);
//     double delecx = delec*DEG;

//     //--------------------------------------------------------------------------
//     // loading module-variables
//     // diagnostics
//     // hyper[566].gets_vec(GAINGAM);
//     // hyper[567].gets(gainff);

//     return delecx;
// }
