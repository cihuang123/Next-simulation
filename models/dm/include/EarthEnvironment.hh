#ifndef __earthenvironment_HH__
#define __earthenvironment_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the EarthEnvironment Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/EarthEnvironment.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "Module.hh"
#include "aux.hh"
#include "global_constants.hh"

#include "env/atmosphere.hh"
#include "env/atmosphere76.hh"
#include "env/atmosphere_nasa2002.hh"
#include "env/atmosphere_weatherdeck.hh"

#include "env/wind.hh"
#include "env/wind_constant.hh"
#include "env/wind_no.hh"
#include "env/wind_tabular.hh"

#include "Time_management.hh"
#include "dm_delta_ut.hh"

class EarthEnvironment : public Dynamics {
  TRICK_INTERFACE(EarthEnvironment);

 public:
  EarthEnvironment(Data_exchang& input);

  EarthEnvironment(const EarthEnvironment& other);
  ~EarthEnvironment();

  EarthEnvironment& operator=(const EarthEnvironment& other);

  void atmosphere_use_public();
  void atmosphere_use_nasa();
  void atmosphere_use_weather_deck(char* filename);

  void set_no_wind();
  void set_constant_wind(double dvae, double dir, double twind,
                         double vertical_wind);
  void set_tabular_wind(char* filename, double twind, double vertical_wind);

  void set_no_wind_turbulunce();
  void set_wind_turbulunce(double turb_length, double turb_sigma, double taux1,
                           double taux1d, double taux2, double taux2d,
                           double tau, double gauss_value);

  virtual void init();
  virtual void algorithm(double int_step);

  void set_RNP();

  // double get_rho();
  // double get_vmach();
  // double get_pdynmc();
  // double get_tempk();
  // double get_dvba();
  // double get_grav();
  // double get_press();

  // arma::vec3 get_GRAVG();
  // arma::vec3 get_VAED();
  // arma::mat33 get_TEI();

 private:
  time_management* time;

  /* Constants */
  cad::Atmosphere* atmosphere;
  cad::Wind* wind;

  /* Variable declaration */
  VECTOR(GRAVG, 3);  /* *o (m/s2) Earth gravity acceleration in inertia frame */
  VECTOR(VBAB, 3);   /* *o (m/s)  Vehicle speed wrt air speed in body frame */
  MATRIX(TEI, 3, 3); /* *o (--)  T.M. from ECI to ECEF */

  double vmach;            /* *o (--)  vehicle mach number */
  double pdynmc;           /* *o (pa)  Dynamic pressure */
  double dvba;             /* *o (m/s) Vehicle air speed magnitude */
  double gravg;            /* *o (m/s2)  Earth gravity acceleration magnitude */
  double tempc;            /* *o (c)  Atmosphere temperature in Celsius */
  double DM_sidereal_time; /* *o  (--)  Sidereal time */
  double DM_Julian_century; /* *o  (--)  Julian_century */

  /* Function declaration */
  void RNP();
  arma::vec AccelHarmonic(arma::vec3 SBII, int n_max, int m_max);
};

#endif  // __earthenvironment_HH__
