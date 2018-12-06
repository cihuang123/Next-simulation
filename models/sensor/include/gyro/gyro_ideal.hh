#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Gyro Implementation)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_ideal.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "gyro/gyro.hh"

class GyroIdeal : public Gyro {
  TRICK_INTERFACE(GyroIdeal);

 public:
  GyroIdeal(Data_exchang &input);
  GyroIdeal(const GyroIdeal &other);
  GyroIdeal &operator=(const GyroIdeal &other);

  virtual ~GyroIdeal() {}
  virtual void init();
  virtual void algorithm(double int_step);
  virtual int write_to_(const char *bus_name);
};

#endif  // __GYRO_IDEAL__
