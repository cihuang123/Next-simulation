#include <armadillo>
#include <cstring>

#include "matrix/utility.hh"

#include "gyro/gyro_rocket6g.hh"

#include "stochastic.hh"

#include <ctime>

GyroRocket6G::GyroRocket6G(Data_exchang &input)
    : VECTOR_INIT(EUG, 3),
      VECTOR_INIT(EWG, 3),
      VECTOR_INIT(EWALKG, 3),
      VECTOR_INIT(EUNBG, 3),
      VECTOR_INIT(EMISG, 3),
      VECTOR_INIT(ESCALG, 3),
      VECTOR_INIT(EBIASG, 3),
      VECTOR_INIT(ITA1, 3),
      VECTOR_INIT(ITA2, 3),
      VECTOR_INIT(BETA, 3) {
  snprintf(name, sizeof(name), "Rocket6G Gyro Sensor Model");
  srand(static_cast<unsigned int>(time(NULL)));
  data_exchang = &input;
}

GyroRocket6G::GyroRocket6G(const GyroRocket6G &other) {
  this->WBICB = other.WBICB;
  this->EWBIB = other.EWBIB;
  this->EUG = other.EUG;
  this->EWG = other.EWG;
  this->EWALKG = other.EWALKG;
  this->EUNBG = other.EUNBG;
  this->EMISG = other.EMISG;
  this->ESCALG = other.ESCALG;
  this->EBIASG = other.EBIASG;
  this->ITA1 = other.ITA1;
  this->ITA2 = other.ITA2;
  this->BETA = other.BETA;
  this->data_exchang = other.data_exchang;
}

GyroRocket6G &GyroRocket6G::operator=(const GyroRocket6G &other) {
  if (&other == this) return *this;

  this->WBICB = other.WBICB;
  this->EWBIB = other.EWBIB;
  this->EUG = other.EUG;
  this->EWG = other.EWG;
  this->EWALKG = other.EWALKG;
  this->EUNBG = other.EUNBG;
  this->EMISG = other.EMISG;
  this->ESCALG = other.ESCALG;
  this->EBIASG = other.EBIASG;
  this->ITA1 = other.ITA1;
  this->ITA2 = other.ITA2;
  this->BETA = other.BETA;
  this->data_exchang = other.data_exchang;

  return *this;
}

void GyroRocket6G::algorithm(double int_step) {
  arma::vec3 WBIB = grab_WBIB();
  arma::vec3 FSPB = grab_FSPB();

  //-------------------------------------------------------------------------
  // ARW RRW
  double sig(1.0);
  double RRW(0.0130848811);  // 0.4422689813  7.6072577e-3
  double ARW(0.2828427125);  // 0.07071067812  7.90569415e-3
  double Freq(200.0);

  for (int i = 0; i < 3; i++) {
    ITA2(i) =
        gauss(0, 1.0) * RRW * RAD;  // distribution(generator) * RRW * RAD;
    BETA(i) = 0.9999 * BETA(i) + ITA2(i) * int_step;
    ITA1(i) = gauss(0, 1.0) * (ARW * sqrt(Freq) / 60 * (1 / sig)) *
              RAD;  // distribution(generator) * (ARW * sqrt(Freq) / 60 * (1 /
                    // sig)) * RAD;
  }

  // combining all uncertainties
  this->EWBIB = ITA1 + BETA;  // EMSBG + EUG + EWG;

  this->WBICB = WBIB + EWBIB;

  data_exchang->hset("WBICB", WBICB);
  return;
}

int GyroRocket6G::write_to_(const char *bus_name) {
  int rc = 0;
  rc |= nxbus_mset(NXBUS_DOUBLE, "Gyro:_WBICB", 3, _WBICB);
  rc |= nxbus_mset(NXBUS_DOUBLE, "Gyro:_EWBIB", 3, _EWBIB);
  return rc;
}
