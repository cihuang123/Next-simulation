#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Gyro model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux.hh>
#include "Module.hh"
#include "global_constants.hh"
#include "stochastic.hh"
#include "nxbus_engine.h"

class Gyro : public Sensor {
  TRICK_INTERFACE(Gyro);

 public:
  char name[256];

  Gyro() : VECTOR_INIT(WBICB, 3), VECTOR_INIT(EWBIB, 3){};

  virtual ~Gyro() {}
  virtual void init() = 0;
  virtual void algorithm(double int_step) = 0;

  virtual arma::vec3 get_computed_WBIB() { return WBICB; }
  virtual arma::vec3 get_error_of_computed_WBIB() { return EWBIB; }
  virtual int write_to_(const char *bus_name) { return 0; }

  std::function<arma::vec3()> grab_WBIB;
  std::function<arma::vec3()> grab_FSPB;

 protected:
  VECTOR(WBICB,
         3); /* *o  (r/s)   Computed inertial body rate in body coordinate */

  VECTOR(EWBIB, 3); /* *o  (r/s)   Error in angular vel of body wrt earth */

  double qqcx; /* *o  (d/s)   INS computed pitch rate */
  double rrcx; /* *o  (d/s)   INS computed yaw rate */
  double ppcx; /* *o  (d/s)   INS computed roll rate */
};

#endif  // __GYRO__
