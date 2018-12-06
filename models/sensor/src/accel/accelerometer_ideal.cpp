#include "accel/accelerometer_ideal.hh"
AccelerometerIdeal::AccelerometerIdeal(Data_exchang &input) {
  snprintf(name, sizeof(name), "Ideal Accelerometer Sensor");
  this->data_exchang = &input;
}

AccelerometerIdeal::AccelerometerIdeal(const AccelerometerIdeal &other) {
  memcpy(this->name, other.name, sizeof(other.name));
  this->FSPCB = other.FSPCB;
  this->EFSPB = other.EFSPB;
}

AccelerometerIdeal &AccelerometerIdeal::operator=(
    const AccelerometerIdeal &other) {
  if (&other == this) return *this;
  memcpy(this->name, other.name, sizeof(other.name));
  this->FSPCB = other.FSPCB;
  this->EFSPB = other.EFSPB;
  return *this;
}

void AccelerometerIdeal::init() {
  data_exchang->hget("FSPB", FSPCB);
  this->EFSPB.zeros();
  data_exchang->hset("FSPCB", FSPCB);
}


void AccelerometerIdeal::algorithm(double int_step) {
  data_exchang->hget("FSPB", FSPCB);
  data_exchang->hset("FSPCB", FSPCB);
}

int AccelerometerIdeal::write_to_(const char *bus_name) {
  int rc = 0;
  rc |= nxbus_mset(NXBUS_DOUBLE, "Accelerometer:_FSPCB", 3, _FSPCB);
  rc |= nxbus_mset(NXBUS_DOUBLE, "Accelerometer:_EFSPB", 3, _EFSPB);
  return rc;
}
