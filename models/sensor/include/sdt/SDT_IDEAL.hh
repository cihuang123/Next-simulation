#ifndef __SDT_IDEAL_HH
#define __SDT_IDEAL_HH
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Sensor Data Transport module)
LIBRARY DEPENDENCY:
      ((../../src/SDT_ideal.cpp))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "sdt/SDT.hh"

class SDT_ideal : public SDT {
  TRICK_INTERFACE(SDT_ideal);

 public:
  SDT_ideal(Data_exchang &input);

  SDT_ideal(const SDT_ideal &other);

  SDT_ideal &operator=(const SDT_ideal &other);

  virtual void init();

  virtual void algorithm(double int_step);
  virtual int write_to_(const char *bus_name);

 private:
  arma::mat33 build_321_rotation_matrix(arma::vec3 angle);

  VECTOR(WBISB, 3); /* *o  (r/s)    Angular rate of body frame relative inertial
                       frame as described in body frame sensed by gyro */

  VECTOR(WBISB_old,
         3); /* *o  (r/s)    Angular rate of body frame relative inertial frame
                as described in body frame (previous time step) */

  VECTOR(DELTA_ALPHA, 3); /* *o  (r)      Delta theta */

  VECTOR(DELTA_ALPHA_old, 3); /* *o (r)   Delta theta (previous time step) */

  VECTOR(ALPHA, 3); /* *o (r)       Alpha */

  VECTOR(FSPSB, 3); /* *o (m/s2)    Specific force of body frame sensed by
                       accelerometer */

  VECTOR(FSPSB_old, 3); /* *o  (m/s2)    Previous Specific force of body frame
                           sensed by accelerometer */

  unsigned int k; /* *o (--)       calculate index */
};

#endif
