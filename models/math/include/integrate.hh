#ifndef __INTEGRATE_UTIL_HH__
#define __INTEGRATE_UTIL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Integration wapper functions)
LIBRARY DEPENDENCY:
      ((../src/integrate.cpp))
*******************************************************************************/

#include <armadillo>
#include <cassert>
#include <vector>
#include "Rocket_Flight_DM.hh"

#define INTEGRATE(in, diff)                         \
  do {                                              \
    double in##d_new = diff;                        \
    in = integrate(in##d_new, in##d, in, int_step); \
    in##d = in##d_new;                              \
  } while (0)

#define INTEGRATE_MAT(in, diff)                     \
  do {                                              \
    arma::mat in##D_NEW = diff;                     \
    in = integrate(in##D_NEW, in##D, in, int_step); \
    in##D = in##D_NEW;                              \
  } while (0)

/**
 * \brief Integration of scalar state variable.
 * Modified Euler Midpoint method
 * Example first order lag:
 *   phid_new=(phic-phi)/tphi;
 *   phi=integrate(phid_new,phid,phi,int_step);
 *   phid=phid_new;
 */
double integrate(const double &dydx_new, const double &dydx, const double &y,
                 const double &int_step);

arma::mat integrate(arma::mat &DYDX_NEW, arma::mat &DYDX, arma::mat &Y,
                    const double int_step);
template <typename T>
void IntegratorRK4(std::vector<arma::vec> V_in, std::vector<arma::vec> &V_out,
                   void (T::*fp)(std::vector<arma::vec> Var_in,
                                 std::vector<arma::vec> &Var_out),
                   T *ClassPointer, double int_step) {
  {
    std::vector<std::vector<arma::vec>> KMAT;
    for (unsigned int i = 0; i < V_in.size(); i++) {
      std::vector<arma::vec> KROW(4);
      KMAT.push_back(KROW);
    }

    V_out = V_in;

    ((ClassPointer)->*fp)(V_out, KMAT[0]);

    for (unsigned int i = 0; i < V_in.size(); i++) {
      V_out[i] = V_in[i] + KMAT[0][i] * 0.5 * int_step;
    }

    ((ClassPointer)->*fp)(V_out, KMAT[1]);

    for (unsigned int i = 0; i < V_in.size(); i++) {
      V_out[i] = V_in[i] + KMAT[1][i] * 0.5 * int_step;
    }

    ((ClassPointer)->*fp)(V_out, KMAT[2]);

    for (unsigned int i = 0; i < V_in.size(); i++) {
      V_out[i] = V_in[i] + KMAT[2][i] * int_step;
    }

    ((ClassPointer)->*fp)(V_out, KMAT[3]);

    for (unsigned int i = 0; i < V_in.size(); i++) {
      V_out[i] = V_in[i] + (int_step / 6.0) * (KMAT[0][i] + 2.0 * KMAT[1][i] +
                                               2.0 * KMAT[2][i] + KMAT[3][i]);
    }
  }
}

template <typename T>
void IntegratorEuler(std::vector<arma::vec> V_in, std::vector<arma::vec> &V_out,
                     void (T::*fp)(std::vector<arma::vec> Var_in,
                                   std::vector<arma::vec> &Var_out),
                     T *ClassPointer, double int_step) {
  std::vector<arma::vec> K_TEMP;

  for (unsigned int i = 0; i < V_in.size(); i++) {
    arma::vec temp;
    K_TEMP.push_back(temp);
  }

  ((ClassPointer)->*fp)(V_in, K_TEMP);

  for (unsigned int i = 0; i < V_in.size(); i++) {
    V_out[i] = V_in[i] + (1. / 2.) * K_TEMP[i] * int_step;
  }
}
#endif  // __INTEGRATE_UTIL_HH__
