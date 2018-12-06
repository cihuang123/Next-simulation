#include "accel/accelerometer_rocket6g.hh"
#include "matrix/utility.hh"
#include "stochastic.hh"

AccelerometerRocket6G::AccelerometerRocket6G(Data_exchang &input)
    : VECTOR_INIT(EWALKA, 3),
      VECTOR_INIT(EMISA, 3),
      VECTOR_INIT(ESCALA, 3),
      VECTOR_INIT(EBIASA, 3),
      VECTOR_INIT(ITA1, 3),
      VECTOR_INIT(ITA2, 3),
      VECTOR_INIT(BETA, 3) {
  snprintf(name, sizeof(name), "Rocket6G Non-Ideal Accelerometer Sensor");
  srand(static_cast<unsigned int>(time(NULL)));
  EWALKA.zeros();
  EMISA.zeros();
  ESCALA.zeros();
  EBIASA.zeros();
  this->data_exchang = &input;
}

AccelerometerRocket6G::AccelerometerRocket6G(const AccelerometerRocket6G &other)
    : VECTOR_INIT(EWALKA, 3),
      VECTOR_INIT(EMISA, 3),
      VECTOR_INIT(ESCALA, 3),
      VECTOR_INIT(EBIASA, 3),
      VECTOR_INIT(ITA1, 3),
      VECTOR_INIT(ITA2, 3),
      VECTOR_INIT(BETA, 3) {
  memcpy(this->name, other.name, sizeof(other.name));
  srand(static_cast<unsigned int>(time(NULL)));
  this->FSPCB = other.FSPCB;
  this->EFSPB = other.EFSPB;
  this->EWALKA = other.EWALKA;
  this->EMISA = other.EMISA;
  this->ESCALA = other.ESCALA;
  this->EBIASA = other.EBIASA;
  this->ITA1 = other.ITA1;
  this->ITA2 = other.ITA2;
  this->BETA = other.BETA;
}

AccelerometerRocket6G &AccelerometerRocket6G::operator=(
    const AccelerometerRocket6G &other) {
  if (&other == this) return *this;

  memcpy(this->name, other.name, sizeof(other.name));
  srand(static_cast<unsigned int>(time(NULL)));
  this->FSPCB = other.FSPCB;
  this->EFSPB = other.EFSPB;
  this->EWALKA = other.EWALKA;
  this->EMISA = other.EMISA;
  this->ESCALA = other.ESCALA;
  this->EBIASA = other.EBIASA;
  this->ITA1 = other.ITA1;
  this->ITA2 = other.ITA2;
  this->BETA = other.BETA;

  return *this;
}

void AccelerometerRocket6G::algorithm(double int_step) {
  arma::vec3 FSPB = grab_FSPB();

  // accelerometer error (bias,scale factor,misalignment)
  // acceleration measurement with random walk effect
  //-------------------------------------------------------------------------
  // computing accelerometer erros without random walk (done in 'ins()')
  // arma::mat33 EAB = diagmat(ESCALA) + skew_sym(EMISA);
  // this->EFSPB = EWALKA + EBIASA + EAB * FSPB;
  // //-------------------------------------------------------------------------
  double sig(1.0);

  double RRW(0.01647856578);  // 0.4422689813  7.6072577e-3
  double ARW(0.01025304833);  // 0.07071067812  7.90569415e-3
  double Freq(200.0);

  for (int i = 0; i < 3; i++) {
    ITA2(i) = gauss(0, 1.0) * RRW;
    BETA(i) = 0.9999 * BETA(i) + ITA2(i) * int_step;
    ITA1(i) = gauss(0, 1.0) * (ARW * sqrt(Freq) / 60 * (1 / sig));
  }

  // combining all uncertainties
  this->EFSPB = ITA1 + BETA;  // EMSBG + EUG + EWG;

  arma::vec3 tmp = floor((FSPB + EFSPB) / AMSB) * AMSB;
  arma::vec3 tmp2 = floor(((FSPB + EFSPB) - tmp) / ALSB) * ALSB;
  this->FSPCB = tmp + tmp2;
  data_exchang->hset("FSPCB", FSPCB);
}

int AccelerometerRocket6G::write_to_(const char *bus_name) {
  int rc = 0;
  rc |= nxbus_mset(NXBUS_DOUBLE, "Accelerometer:_FSPCB", 3, _FSPCB);
  rc |= nxbus_mset(NXBUS_DOUBLE, "Accelerometer:_EFSPB", 3, _EFSPB);
  return rc;
}