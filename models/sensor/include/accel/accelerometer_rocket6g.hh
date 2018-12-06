#ifndef __ACCEL_6G_HH__
#define __ACCEL_6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Accelerometer Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/accel/accelerometer_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "accel/accelerometer.hh"

class AccelerometerRocket6G : public Accelerometer {
  TRICK_INTERFACE(AccelerometerRocket6G);

 public:
  AccelerometerRocket6G(Data_exchang &input);
  AccelerometerRocket6G(const AccelerometerRocket6G &other);
  AccelerometerRocket6G &operator=(const AccelerometerRocket6G &other);
  virtual ~AccelerometerRocket6G() {}

  virtual void algorithm(double int_step);
  virtual int write_to_(const char *bus_name);

 private:
  VECTOR(EWALKA, 3); /* *o   (m/s2)  Acceleration random noise */

  VECTOR(EMISA, 3); /* *o   (r)     Acceleration misalignment */

  VECTOR(ESCALA, 3); /* *o   (--)    Acceleration scale factor */

  VECTOR(EBIASA, 3); /* *o   (m/s2)  Acceleration bias */

  VECTOR(ITA1, 3); /* *o   (--)  Random noise calculation variable */

  VECTOR(ITA2, 3); /* *o   (--)  Random noise calculation variable */

  VECTOR(BETA, 3); /* *o   (--)  Random noise calculation variable */
};

#endif  // __ACCEL_6G__
