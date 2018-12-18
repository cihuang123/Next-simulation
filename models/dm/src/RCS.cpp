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

  Q_RCS.zeros();

  Thruster_list[0]->calculate_Q(e_roll, TBI, Thruster_list[0]->mode);
  Thruster_list[1]->calculate_Q(-e_roll, TBI, Thruster_list[1]->mode);
  Thruster_list[2]->calculate_Q(0.0, TBI, Thruster_list[2]->mode);
  Thruster_list[3]->calculate_Q(0.0, TBI, Thruster_list[3]->mode);
  Thruster_list[4]->calculate_Q(0.0, TBI, Thruster_list[4]->mode);
  Thruster_list[5]->calculate_Q(0.0, TBI, Thruster_list[5]->mode);

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