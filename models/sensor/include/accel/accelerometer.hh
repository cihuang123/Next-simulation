#ifndef __ACCEL_HH__
#define __ACCEL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Accelerometer model interface definition)
*******************************************************************************/
#include <armadillo>
#include <aux.hh>
#include "Module.hh"
#include "global_constants.hh"
#include "stochastic.hh"
#include "nxbus_engine.h"
class Accelerometer : public Sensor {
  TRICK_INTERFACE(Accelerometer);

 public:
  char name[256];

  Accelerometer() : VECTOR_INIT(FSPCB, 3), VECTOR_INIT(EFSPB, 3){};

  virtual ~Accelerometer() {}
  virtual void init() = 0;
  virtual void algorithm(double int_step) = 0;

  std::function<arma::vec3()> grab_FSPB;

  virtual arma::vec3 get_computed_FSPB() { return FSPCB; }
  virtual arma::vec3 get_error_of_computed_FSPB() { return EFSPB; }
  virtual int write_to_(const char *bus_name) { return 0; }

 protected:
  VECTOR(FSPCB, 3); /* *o  (N/kg)  Computed specific force on body */

  VECTOR(EFSPB, 3); /* *o   (N/kg)  Error in specific force on body in body
                      coordinate */
};

#endif  // __ACCEL__
