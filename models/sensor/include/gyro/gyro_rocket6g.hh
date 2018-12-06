#ifndef __GYRO_ROCKET6G_HH__
#define __GYRO_ROCKET6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Gyro Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "gyro/gyro.hh"

class GyroRocket6G : public Gyro {
  TRICK_INTERFACE(GyroRocket6G);

 public:
  GyroRocket6G(Data_exchang &input);
  GyroRocket6G(const GyroRocket6G &other);
  GyroRocket6G &operator=(const GyroRocket6G &other);

  virtual ~GyroRocket6G() {}
  virtual void init(){};
  virtual void algorithm(double int_step);
  virtual int write_to_(const char *bus_name);
  std::default_random_engine generator;

 private:
  /* Routing components */
  VECTOR(EUG, 3); /* *o   (r/s)    Gyro spin axis accel sensitivity */

  VECTOR(EWG, 3); /* *o   (r/s)    Gyro random walk errors */

  VECTOR(EWALKG, 3); /* *o   (r/s0.5) Random walk */

  VECTOR(EUNBG, 3); /* *o   (r)      Gyro cluster misalignment */

  VECTOR(EMISG, 3); /* *o   (r)      Gyro misalignmt */

  VECTOR(ESCALG, 3); /* *o   (--)     Gyro scale factor */

  VECTOR(EBIASG, 3); /* *o   (r/s)    Gyro bias */

  VECTOR(ITA1, 3); /* *o   (--)  Coefficient of gyro noise calculation */

  VECTOR(ITA2, 3); /* *o   (--)  Coefficient of gyro noise calculation */

  VECTOR(BETA, 3); /* *o   (--)  Coefficient of gyro noise calculation */
};

#endif  // __GYRO_ROCKET6G__
