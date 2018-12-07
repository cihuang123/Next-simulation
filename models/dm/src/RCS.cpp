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

  Q_RCS.zeros();

  Thruster_list[0]->calculate_Q(0.0, TBI, X);
  Thruster_list[1]->calculate_Q(0.0, TBI, X);
  Thruster_list[2]->calculate_Q(0.0, TBI, X);
  Thruster_list[3]->calculate_Q(0.0, TBI, X);

  for (unsigned int i = 0; i < Thruster_list.size(); i++) {
    Q_RCS += Thruster_list[i]->Q;
  }

  data_exchang->hset("Q_RCS", Q_RCS);
}