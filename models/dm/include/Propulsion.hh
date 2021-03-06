#ifndef __propulsion_HH__
#define __propulsion_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the propulsion Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Propulsion.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "Module.hh"
#include "aux.hh"
#include "datadeck.hh"
#include "global_constants.hh"
#include "sim_services/include/simtime.h"

class Propulsion : public Actuator {
  TRICK_INTERFACE(Propulsion);

 public:
  Propulsion(Data_exchang& input);
  Propulsion(const Propulsion& other);

  Propulsion& operator=(const Propulsion& other);

  virtual void init();
  virtual void algorithm(double int_step);
  void load_proptable(const char* filename);

  enum THRUST_TYPE {
    NO_THRUST = 0,
    INPUT_THRUST = 3,
    LTG_THRUST = 4,
    HOT_STAGE = 5
  };

  enum STAGE { STAGE_1 = 0, STAGE_2 = 1, STAGE_3 = 2, FARING_SEP = 3 };

  void Allocate_stage(unsigned int stage_num);
  void set_stage_var(double isp, double fmass_init, double vmass_init,
                     double aexit_in, double fuel_flow_rate_in, double xcg0,
                     double xcg1, double moi_roll0, double moi_roll1,
                     double moi_pitch0, double moi_pitch1, double moi_yaw0,
                     double moi_yaw1, unsigned int num_stage);
  void set_stage_1();
  void set_stage_2();
  void set_stage_3();
  void set_faring_sep();
  void engine_ignition();
  void set_no_thrust();
  void set_ltg_thrust();
  // void get_input_file_var(double XCG0, double XCG1, double moi_roll0,
  //                         double moi_roll1, double moi_pitch0,
  //                         double moi_pitch1, double moi_yaw0, double
  //                         moi_yaw1, double spi, double fuel_flow_rate);
  void set_faring_mass(double in);
  void set_HOT_STAGE();
  // XXX: get_thrust_state
  enum THRUST_TYPE get_thrust_state();
  std::function<int()> grab_beco_flag;
  /* Input File */
  // void set_vmass0(double);
  // void set_fmass0(double);

  // void set_aexit(double);
  void set_payload_mass(double);

 private:
  /* Rocket Engine state variables */
  struct ENG {
    double ENG_mass;
    double ENG_mass0;
    double ENG_mass1;
    double ENG_isp;

    MATRIX(ENG_I, 3, 3);
    MATRIX(ENG_I_0, 3, 3);
    MATRIX(ENG_I_1, 3, 3);

    VECTOR(ENG_XCG, 3);
    VECTOR(ENG_XCG_0, 3);
    VECTOR(ENG_XCG_1, 3);
  };

  /* Stage variable */
  struct STAGE_VAR {
    double isp;            /* *o (--) Rocket engine's specific impluse */
    double fmass0;         /* ** (kg) Init fuel mass */
    double StageMass0;     /* ** (kg) Stage init mass */
    arma::mat33 IBBB0;     /* *o (--) Initial vehicle's MOI */
    arma::mat33 IBBB1;     /* *o (kg*m2) Final vehicle's MOI */
    arma::vec3 XCG_0;      /* *o (m) Initial vehicle's cg */
    arma::vec3 XCG_1;      /* *o (m) Final vehicle's cg */
    double fmasse;         /* *o (kg) Expended fuel mass */
    double fmassd;         /* ** (kg/s) Fuel mass expended derivative */
    double fuel_flow_rate; /* *o (kg/s) Fuel flow rate of engine */
    double aexit;          /* *o (m2) Engine nozzle area */
    double timer;          /* ** (s) Stage timer */
  };

  Datadeck proptable;
  /* Internal Initializers */
  void default_data();

  /* Internal Propagator / Calculators */
  void propagate_delta_v(double int_step, double spi, double fuel_flow_rate,
                         double vmass);
  void fuel_expend_integrator(double int_step, enum STAGE flag);

  /* Internal Calculators */
  // double calculate_thrust(double press);
  double calculate_fmassr(enum STAGE stage);

  arma::vec3 calculate_XCG(enum STAGE stage);
  arma::mat33 calculate_IBBB(enum STAGE stage);

  /* Variable declaration */
  std::vector<struct ENG*> Eng_list;
  std::vector<struct STAGE_VAR*> Stage_var_list;
  VECTOR(XCG, 3);      /* *o (m) Current vehicle's cg */
  MATRIX(IBBB, 3, 3);  /* *o (kg*m2) Current cheicle's MOI */
  double vmass;        /* *o (kg) Current vehicle mass */
  double fmassr;       /* *o (kg) Remaining fuel mass */
  double mass_ratio;   /* *o (--) fmasse / fmass0 */
  double payload_mass; /* *o (kg) Payload mass */
  double faring_mass;  /* *o (kg) Vehicle faring mass */
  double thrust;       /* *o (N) Thrust generated by vehicle's engine */
  double delta_v;      /* *o (m/s)  Vehicle delta-V generated by engine */

  /* State */
  enum THRUST_TYPE
      thrust_state; /* *o (--)     Propulsion mode, See THRUST TYPE*/
  enum STAGE stage;
};

#endif  // __propulsion_HH__
