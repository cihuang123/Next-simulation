#ifndef __aerodynamics_HH__
#define __aerodynamics_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the AERODYNAMICS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Aerodynamics.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "Module.hh"
#include "Propulsion.hh"
#include "aux.hh"
#include "datadeck.hh"
#include "global_constants.hh"

class Propulsion;

class AeroDynamics : public Dynamics {
  TRICK_INTERFACE(AeroDynamics);

 public:
  explicit AeroDynamics(Propulsion& prop, Data_exchang& input);
  AeroDynamics(const AeroDynamics& other);

  AeroDynamics& operator=(const AeroDynamics& other);

  void load_aerotable(const char* filename);

  virtual void init();
  virtual void algorithm(double int_step);

  void set_refa(double);
  void set_refd(double);

 private:
  Propulsion* propulsion;
  Datadeck aerotable; /* ** (--) Aero Deck */

  double refa;  /* *o (m2)     Reference area for aero coefficients - m^2*/
  double refd;  /* *o (m)      Reference length for aero coefficients - m*/
  double cy;    /* *o (--)     Side force coefficient - ND*/
  double cll;   /* *o (--)     Rolling moment coefficient - ND*/
  double clm;   /* *o (--)     Pitching moment coefficient - ND*/
  double cln;   /* *o (--)     Yawing moment coefficient - ND*/
  double cx;    /* *o (--)     X-force coefficient - ND*/
  double cz;    /* *o (--)     Z-force coefficient - ND*/
  double cn;    /* *o (--)     Normal force coefficient */
  double ca;    /* *o (--)     Axial force coefficient */
  double ca_on; /* *o (--)     Axial force coefficient when engine is running */
  double cnq;   /* *o (--)     Normal damping coefficient */
  double cmq;   /* *o (--)     Pitch moment derivative coefficient */
  double ca0;
  double caa;
  double ca0b;
  double cn0;
  double clm0;
  double clmq;
  double cna;
  double clma;

  VECTOR(XCP,
         3); /* *o (m)      Non-dimensional center of pressure (from nose)*/
};

#endif  // __aerodynamics_HH__
