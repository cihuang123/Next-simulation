#include <cassert>
#include <iostream>
#include "aux.hh"
#include "global_constants.hh"
#include "integrate.hh"
#include "math_utility.hh"
#include "matrix/utility.hh"
#include "sdt/SDT_NONIDEAL.hh"


SDT_NONIDEAL::SDT_NONIDEAL(Data_exchang& input)
    : VECTOR_INIT(WBISB, 3),
      VECTOR_INIT(WBISB_old, 3),
      VECTOR_INIT(DELTA_ALPHA, 3),
      VECTOR_INIT(DELTA_ALPHA_old, 3),
      VECTOR_INIT(ALPHA, 3),
      VECTOR_INIT(FSPSB, 3),
      VECTOR_INIT(FSPSB_old, 3),
      VECTOR_INIT(cross2_old, 3),
      VECTOR_INIT(VEL, 3) {
  k = 0;
  data_exchang = &input;
}

SDT_NONIDEAL::SDT_NONIDEAL(const SDT_NONIDEAL& other) {
  this->data_exchang = other.data_exchang;
  this->WBISB = other.WBISB;
  this->WBISB_old = other.WBISB_old;
  this->DELTA_ALPHA = other.DELTA_ALPHA;
  this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
  this->ALPHA = other.ALPHA;
  this->FSPSB = other.FSPSB;
  this->PHI = other.PHI;
}

SDT_NONIDEAL& SDT_NONIDEAL::operator=(const SDT_NONIDEAL& other) {
  if (&other == this) return *this;
  this->data_exchang = other.data_exchang;
  this->WBISB = other.WBISB;
  this->WBISB_old = other.WBISB_old;
  this->DELTA_ALPHA = other.DELTA_ALPHA;
  this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
  this->ALPHA = other.ALPHA;
  this->FSPSB = other.FSPSB;
  this->PHI = other.PHI;
  return *this;
}

void SDT_NONIDEAL::algorithm(double int_step) {
  data_exchang->hget("WBICB", WBISB);
  data_exchang->hget("FSPCB", FSPSB);

  if (k == 11) {
    k = 1;
    PHI.zeros();
    ALPHA.zeros();
    DELTA_VEL.zeros();
    VEL.zeros();
  }

  arma::vec3 zo(arma::fill::zeros);
  arma::vec3 CONING = grab_CONING();

  arma::vec3 tmp = WBISB + CONING;

  DELTA_ALPHA = integrate(tmp, WBISB_old, zo, int_step);

  ALPHA += DELTA_ALPHA;

  DELTA_ALPHA_old = DELTA_ALPHA;

  WBISB_old = tmp;

  PHI += DELTA_ALPHA;  // + 0.5 * DELTA_BETA;

  arma::vec3 cross2 = cross(ALPHA, FSPSB);

  DELTA_VEL += integrate(FSPSB, FSPSB_old, zo, int_step) +
               integrate(cross2, cross2_old, zo, int_step);
  FSPSB_old = FSPSB;
  cross2_old = cross2;

  k++;
}

arma::mat33 SDT_NONIDEAL::build_321_rotation_matrix(arma::vec3 angle) {
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

int SDT_NONIDEAL::write_to_(const char* bus_name) {
  int rc = 0;
  rc |= nxbus_mset(NXBUS_DOUBLE, "SDT:_PHI", 3, _PHI);
  rc |= nxbus_mset(NXBUS_DOUBLE, "SDT:_DELTA_VEL", 3, _DELTA_VEL);
  return rc;
}
