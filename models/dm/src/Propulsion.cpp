#include "integrate.hh"

#include "Propulsion.hh"
#include "sim_services/include/simtime.h"

Propulsion::Propulsion(Data_exchang& input)
    : MATRIX_INIT(IBBB, 3, 3), VECTOR_INIT(XCG, 3) {
  this->default_data();
  data_exchang = &input;
}

Propulsion::Propulsion(const Propulsion& other)
    : MATRIX_INIT(IBBB, 3, 3), VECTOR_INIT(XCG, 3) {
  this->default_data();

  /* Constants */
  this->data_exchang = other.data_exchang;
  this->XCG = other.XCG;
  this->IBBB = other.IBBB;
  this->vmass = other.vmass;
  this->fmassr = other.fmassr;
  this->mass_ratio = other.mass_ratio;
  this->payload_mass = other.payload_mass;
  this->faring_mass = other.faring_mass;
  this->thrust = other.thrust;
  this->delta_v = other.delta_v;
  this->thrust_state = other.thrust_state;
  this->stage = other.stage;
  this->Eng_list.assign(other.Eng_list.begin(), other.Eng_list.end());
  this->Stage_var_list.assign(other.Stage_var_list.begin(),
                              other.Stage_var_list.end());
}

Propulsion& Propulsion::operator=(const Propulsion& other) {
  if (&other == this) return *this;

  /* Constants */
  this->XCG = other.XCG;
  this->IBBB = other.IBBB;
  this->vmass = other.vmass;
  this->fmassr = other.fmassr;
  this->mass_ratio = other.mass_ratio;
  this->payload_mass = other.payload_mass;
  this->faring_mass = other.faring_mass;
  this->thrust = other.thrust;
  this->delta_v = other.delta_v;
  this->thrust_state = other.thrust_state;
  this->stage = other.stage;
  this->Eng_list.assign(other.Eng_list.begin(), other.Eng_list.end());
  this->Stage_var_list.assign(other.Stage_var_list.begin(),
                              other.Stage_var_list.end());
  this->data_exchang = other.data_exchang;
  return *this;
}

void Propulsion::init() {
  this->IBBB = calculate_IBBB(STAGE_1);
  this->XCG = calculate_XCG(STAGE_1);
  vmass = Stage_var_list[STAGE_1]->StageMass0;

  data_exchang->hset("vmass", vmass);
  data_exchang->hset("IBBB", IBBB);
  data_exchang->hset("XCG", XCG);
  data_exchang->hset("XCG_0", Stage_var_list[STAGE_1]->XCG_0);
}

void Propulsion::default_data() {
  this->thrust = 0;
  this->payload_mass = 0;

  for (int i = 0; i < Eng_list.size(); i++) {
    Eng_list[i] = NULL;
  }

  for (int i = 0; i < Stage_var_list.size(); i++) {
    Stage_var_list[i] = NULL;
  }
}

void Propulsion::set_no_thrust() { this->thrust_state = NO_THRUST; }

void Propulsion::Allocate_stage(unsigned int stage_num) {
  struct STAGE_VAR** stage_var = new STAGE_VAR*[stage_num];
  for (int i = 0; i < stage_num; i++) {
    stage_var[i] = new STAGE_VAR;
    Stage_var_list.push_back(stage_var[i]);
  }
}

void Propulsion::set_stage_var(double isp, double fmass_init, double vmass_init,
                               double aexit_in, double fuel_flow_rate_in,
                               double xcg0, double xcg1, double moi_roll0,
                               double moi_roll1, double moi_pitch0,
                               double moi_pitch1, double moi_yaw0,
                               double moi_yaw1, unsigned int num_stage) {
  Stage_var_list[num_stage]->IBBB0.zeros();
  Stage_var_list[num_stage]->IBBB1.zeros();
  Stage_var_list[num_stage]->XCG_0.zeros();
  Stage_var_list[num_stage]->XCG_1.zeros();

  Stage_var_list[num_stage]->isp = isp;
  Stage_var_list[num_stage]->fmass0 = fmass_init;
  Stage_var_list[num_stage]->StageMass0 = vmass_init;
  Stage_var_list[num_stage]->fuel_flow_rate = fuel_flow_rate_in;
  Stage_var_list[num_stage]->IBBB0(0, 0) = moi_roll0;
  Stage_var_list[num_stage]->IBBB1(0, 0) = moi_roll1;
  Stage_var_list[num_stage]->IBBB0(1, 1) = moi_pitch0;
  Stage_var_list[num_stage]->IBBB1(1, 1) = moi_pitch1;
  Stage_var_list[num_stage]->IBBB0(2, 2) = moi_yaw0;
  Stage_var_list[num_stage]->IBBB1(2, 2) = moi_yaw1;
  Stage_var_list[num_stage]->XCG_0(0) = xcg0;
  Stage_var_list[num_stage]->XCG_1(0) = xcg1;
  Stage_var_list[num_stage]->fmasse = 0.0;
  Stage_var_list[num_stage]->fmassd = 0.0;
  Stage_var_list[num_stage]->aexit = aexit_in;
}

// void Propulsion::get_input_file_var(double XCG0, double XCG1, double
// moi_roll0,
//                                     double moi_roll1, double moi_pitch0,
//                                     double moi_pitch1, double moi_yaw0,
//                                     double moi_yaw1, double isp,
//                                     double fuel_flow_rate) {
//   this->XCG_0(0) = XCG0;
//   this->XCG_1(0) = XCG1;
//   this->IBBB0(0, 0) = moi_roll0;
//   this->IBBB1(0, 0) = moi_roll1;
//   this->IBBB0(1, 1) = moi_pitch0;
//   this->IBBB1(1, 1) = moi_pitch1;
//   this->IBBB0(2, 2) = moi_yaw0;
//   this->IBBB1(2, 2) = moi_yaw1;
//   this->fuel_flow_rate = fuel_flow_rate;
//   this->isp = isp;
//   this->fmasse = 0;
// }

void Propulsion::engine_ignition() { this->thrust_state = INPUT_THRUST; }

void Propulsion::set_ltg_thrust() { this->thrust_state = LTG_THRUST; }
void Propulsion::set_HOT_STAGE() { this->thrust_state = HOT_STAGE; }
void Propulsion::set_stage_2() { this->stage = STAGE_2; }
void Propulsion::set_stage_3() { this->stage = STAGE_3; }
void Propulsion::set_faring_sep() { this->stage = FARING_SEP; }
void Propulsion::set_faring_mass(double in) { faring_mass = in; }
void Propulsion::set_stage_1() { this->stage = STAGE_1; }

void Propulsion::algorithm(double int_step) {
  double psl(101300);  // chamber pressure - Pa

  double press;
  data_exchang->hget("press", &press);
  // no thrusting
  switch (this->thrust_state) {
    case NO_THRUST:
      thrust = 0;
      break;
    case INPUT_THRUST:
      thrust = Stage_var_list[stage]->isp *
                   Stage_var_list[stage]->fuel_flow_rate * AGRAV +
               (psl - press) * Stage_var_list[stage]->aexit;
      double fmassd_next = thrust / (Stage_var_list[stage]->isp * AGRAV);
      Stage_var_list[stage]->fmasse =
          integrate(fmassd_next, Stage_var_list[stage]->fmassd,
                    Stage_var_list[stage]->fmasse, int_step);
      Stage_var_list[stage]->fmassd = fmassd_next;
      mass_ratio =
          Stage_var_list[stage]->fmasse / Stage_var_list[stage]->fmass0;
      vmass = Stage_var_list[stage]->StageMass0 - Stage_var_list[stage]->fmasse;
      fmassr = Stage_var_list[stage]->fmass0 - Stage_var_list[stage]->fmasse;
      IBBB = calculate_IBBB(stage);
      XCG = calculate_XCG(stage);
      propagate_delta_v(int_step, Stage_var_list[stage]->isp,
                        Stage_var_list[stage]->fuel_flow_rate, vmass);
      if (fmassr <= 0.0) thrust_state = NO_THRUST;
      break;
  }

  data_exchang->hset("XCG", XCG);
  data_exchang->hset("thrust", thrust);
  data_exchang->hset("vmass", vmass);
  data_exchang->hset("IBBB", IBBB);
  data_exchang->hset("XCG_0", Stage_var_list[stage]->XCG_0);
}

void Propulsion::fuel_expend_integrator(double int_step, enum STAGE flag) {
  double K1, K2, K3, K4;
  switch (flag) {
    case STAGE_2:
      K1 = proptable.look_up("S2_time_vs_thrust",
                             Stage_var_list[STAGE_2]->timer, 0) /
           Stage_var_list[STAGE_2]->isp;
      K2 = proptable.look_up("S2_time_vs_thrust",
                             Stage_var_list[STAGE_2]->timer + 0.5 * int_step,
                             0) /
           Stage_var_list[STAGE_2]->isp;
      K3 = proptable.look_up("S2_time_vs_thrust",
                             Stage_var_list[STAGE_2]->timer + 0.5 * int_step,
                             0) /
           Stage_var_list[STAGE_2]->isp;
      K4 = proptable.look_up("S2_time_vs_thrust",
                             Stage_var_list[STAGE_2]->timer + int_step, 0) /
           Stage_var_list[STAGE_2]->isp;

      Stage_var_list[STAGE_2]->fmasse =
          Stage_var_list[STAGE_2]->fmasse +
          (int_step / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
      break;

    case STAGE_3:
      K1 = proptable.look_up("S3_time_vs_thrust",
                             Stage_var_list[STAGE_3]->timer, 0) /
           Stage_var_list[STAGE_3]->isp;
      K2 = proptable.look_up("S3_time_vs_thrust",
                             Stage_var_list[STAGE_3]->timer + 0.5 * int_step,
                             0) /
           Stage_var_list[STAGE_3]->isp;
      K3 = proptable.look_up("S3_time_vs_thrust",
                             Stage_var_list[STAGE_3]->timer + 0.5 * int_step,
                             0) /
           Stage_var_list[STAGE_3]->isp;
      K4 = proptable.look_up("S3_time_vs_thrust",
                             Stage_var_list[STAGE_3]->timer + int_step, 0) /
           Stage_var_list[STAGE_3]->isp;
      Stage_var_list[STAGE_3]->fmasse =
          Stage_var_list[STAGE_3]->fmasse +
          (int_step / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
      break;
  }
}

void Propulsion::propagate_delta_v(double int_step, double spi,
                                   double fuel_flow_rate, double vmass) {
  this->delta_v += spi * AGRAV * (fuel_flow_rate / vmass) * int_step;
}

// double Propulsion::calculate_thrust(double press) {
//   return proptable.look_up("time_vs_thrust", get_rettime(), 0) * AGRAV +
//          (-press) * this->aexit;
// }

double Propulsion::calculate_fmassr(enum STAGE stage) {
  return Stage_var_list[stage]->fmass0 - Stage_var_list[stage]->fmasse;
}

arma::vec3 Propulsion::calculate_XCG(enum STAGE stage) {
  return -(Stage_var_list[stage]->XCG_0 +
           (Stage_var_list[stage]->XCG_1 - Stage_var_list[stage]->XCG_0) *
               mass_ratio);
}

arma::mat33 Propulsion::calculate_IBBB(enum STAGE stage) {
  return Stage_var_list[stage]->IBBB0 +
         (Stage_var_list[stage]->IBBB1 - Stage_var_list[stage]->IBBB0) *
             mass_ratio;
}

void Propulsion::load_proptable(const char* filename) {
  proptable = Datadeck(filename);
}

// void Propulsion::set_StageMass0(double in) { StageMass0 = in; }
// void Propulsion::set_fmass0(double in) { fmass0 = in; }
// void Propulsion::set_aexit(double in) { aexit = in; }
void Propulsion::set_payload_mass(double in) { payload_mass = in; }
enum Propulsion::THRUST_TYPE Propulsion::get_thrust_state() {
  return this->thrust_state;
}
