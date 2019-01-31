#include <cassert>
#include "aux.hh"

#include <iostream>
#include "global_constants.hh"
#include "integrate.hh"
#include "math_utility.hh"
#include "matrix/utility.hh"
#include "sdt/SDT_IDEAL.hh"

SDT_ideal::SDT_ideal(Data_exchang &input, unsigned int kl_in)
    : VECTOR_INIT(WBISB, 3),
      VECTOR_INIT(WBISB_old, 3),
      VECTOR_INIT(DELTA_ALPHA, 3),
      VECTOR_INIT(ALPHA, 3),
      VECTOR_INIT(FSPSB, 3),
      VECTOR_INIT(FSPSB_old, 3) {
  data_exchang = &input;
  k_limit = kl_in;
}

SDT_ideal::SDT_ideal(const SDT_ideal &other) {
  this->data_exchang = other.data_exchang;
  this->WBISB = other.WBISB;
  this->WBISB_old = other.WBISB_old;
  this->DELTA_ALPHA = other.DELTA_ALPHA;
  this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
  this->ALPHA = other.ALPHA;
  this->FSPSB = other.FSPSB;
  this->FSPSB_old = other.FSPSB_old;
  this->k = other.k;
}

SDT_ideal &SDT_ideal::operator=(const SDT_ideal &other) {
  if (&other == this) return *this;
  this->data_exchang = other.data_exchang;
  this->WBISB = other.WBISB;
  this->WBISB_old = other.WBISB_old;
  this->DELTA_ALPHA = other.DELTA_ALPHA;
  this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
  this->ALPHA = other.ALPHA;
  this->FSPSB = other.FSPSB;
  this->FSPSB_old = other.FSPSB_old;
  this->k = other.k;
  return *this;
}

void SDT_ideal::init() {
  k = 0;
  PHI.zeros();
  WBISB_old.zeros();
  DELTA_ALPHA.zeros();
}
void SDT_ideal::algorithm(double int_step) {
  data_exchang->hget("WBICB", WBISB);
  data_exchang->hget("FSPCB", FSPSB);
  if (k == k_limit || k == 1) {
    k = 1;
    PHI.zeros();
    ALPHA.zeros();
    DELTA_VEL.zeros();
  }

  // Coning compensation algorithm

  arma::vec3 zo(arma::fill::zeros);

  arma::vec3 tmp = WBISB;

  DELTA_ALPHA = WBISB * int_step;  // Strapdown Analytics 7.1.1.1.1-5

  ALPHA += DELTA_ALPHA;  // Strapdown Analytics 7.1.1.1.1-5

  DELTA_ALPHA_old = DELTA_ALPHA;

  WBISB_old = tmp;

  PHI += DELTA_ALPHA;  // + 0.5 * DELTA_BETA;

  FSPSB = build_321_rotation_matrix(DELTA_ALPHA) * FSPSB;  // Strapdown 7.2.2-3

  DELTA_VEL +=
      FSPSB * int_step;  // + integrate(cross2, cross2_old, zo, int_step);
  FSPSB_old = FSPSB;
  k++;
}

arma::mat33 SDT_ideal::build_321_rotation_matrix(arma::vec3 angle) {
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

int SDT_ideal::write_to_(const char *bus_name) {
  int rc = 0;
  rc |= nxbus_mset(NXBUS_DOUBLE, "SDT:_PHI", 3, _PHI);
  rc |= nxbus_mset(NXBUS_DOUBLE, "SDT:_DELTA_VEL", 3, _DELTA_VEL);
  return rc;
}
