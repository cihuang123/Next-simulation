#include "RCS.hh"

RCS::RCS(Data_exchang &input) : VECTOR_INIT(Q_RCS, 6) { data_exchang = &input; }

RCS::RCS(const RCS &other) : VECTOR_INIT(Q_RCS, 6) {
  data_exchang = other.data_exchang;
  Q_RCS = other.Q_RCS;
}

void RCS::init(){};

void RCS::algorithm(double int_step) {
  arma::mat33 TBI;
  data_exchang->hget("TBI", TBI);
  double e_roll = grab_e_roll();
  double e_pitch = grab_e_pitch();
  double e_yaw = grab_e_yaw();

  Q_RCS.zeros();

  Thruster_list[0]->calculate_Torque_Q(e_roll, TBI, Thruster_list[0]->mode);
  Thruster_list[1]->calculate_Torque_Q(e_pitch, TBI, Thruster_list[1]->mode);
  Thruster_list[2]->calculate_Torque_Q(e_yaw, TBI, Thruster_list[2]->mode);

  for (unsigned int i = 0; i < Thruster_list.size(); i++) {
    Q_RCS += Thruster_list[i]->Q;
  }

  data_exchang->hset("Q_RCS", Q_RCS);
}

void RCS::Allocate_RCS(int num, std::vector<RCS_Thruster *> &RT_list) {
  RCS_Thruster **RT_LIST = new RCS_Thruster *[num];
  for (int i = 0; i < num; i++) {
    RT_LIST[i] = new RCS_Thruster;
    RT_list.push_back(RT_LIST[i]);
  }
}