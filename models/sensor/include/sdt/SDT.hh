#ifndef __SDT_HH
#define __SDT_HH
#include <armadillo>
#include <functional>
#include "Module.hh"

class SDT : public Sensor {
  TRICK_INTERFACE(SDT);

 public:
  SDT()
      : VECTOR_INIT(PHI, 3),
        VECTOR_INIT(DELTA_VEL, 3) {};
  virtual ~SDT(){};

  virtual void init() = 0;
  virtual void algorithm(double int_step) = 0;

  std::function<arma::vec3()> grab_WBICB;
  std::function<arma::vec3()> grab_FSPCB;
  std::function<arma::vec3()> grab_CONING;
  std::function<arma::vec3()> grab_GHIGH;
  std::function<arma::vec3()> grab_GLOW;
  std::function<arma::vec3()> grab_AHIGH;
  std::function<arma::vec3()> grab_ALOW;

  virtual arma::vec3 get_PHI() { return PHI; }
  virtual arma::vec3 get_DELTA_VEL() { return DELTA_VEL; }

 protected:
  virtual arma::mat33 build_321_rotation_matrix(arma::vec3 angle) {}

  VECTOR(PHI, 3); /* *o (r)       PHI = DELTA_ALPHA + 0.5 * DELTA_BETA */

  VECTOR(DELTA_VEL, 3); /* *o (m/s)     Delta V */
};

#endif
