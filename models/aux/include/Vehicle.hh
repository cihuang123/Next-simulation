#ifndef Vehicle_list__HH
#define Vehicle_list__HH
#include <armadillo>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
#include "aux.hh"
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Simulation module abstraction class and vehicle class)
LIBRARY DEPENDENCY:
      ()
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
class Data_exchang {
 public:
  Data_exchang(){};
  ~Data_exchang(){};

  void hset(std::string input, int val) { int_table[input] = val; }
  void hset(std::string input, double val) { double_table[input] = val; }
  void hset(std::string input, arma::vec val) { vec_table[input] = val; }
  void hset(std::string input, arma::mat val) { mat_table[input] = val; }

  void hget(std::string input, int *val) {
    if (int_table.find(input) == int_table.end()) {
      std::cerr << "Can not find " << input << " in int_table." << std::endl;
      std::abort();
    }
    *val = int_table.find(input)->second;
  }
  void hget(std::string input, double *val) {
    if (double_table.find(input) == double_table.end()) {
      std::cerr << "Can not find " << input << " in double_table." << std::endl;
      std::abort();
    }
    *val = double_table.find(input)->second;
  }
  void hget(std::string input, arma::vec &val) {
    if (vec_table.find(input) == vec_table.end()) {
      std::cerr << "Can not find " << input << " in vec_table." << std::endl;
      std::abort();
    }
    val = vec_table.find(input)->second;
  }
  void hget(std::string input, arma::mat &val) {
    if (mat_table.find(input) == mat_table.end()) {
      std::cerr << "Can not find " << input << " in mat_table." << std::endl;
      std::abort();
    }
    val = mat_table.find(input)->second;
  }

 private:
  std::map<std::string, int> int_table;
  std::map<std::string, double> double_table;
  std::map<std::string, arma::vec> vec_table;
  std::map<std::string, arma::mat> mat_table;
};

class FH_module {
 public:
  FH_module(){};
  ~FH_module(){};

  virtual void algorithm(double int_step) = 0;
  virtual void init() = 0;

 protected:
  Data_exchang *data_exchang;
};

// class Vehicle {
//  private:
//   char Name[32];
//   int module_num;
//   FH_module **module_ptr;

//  public:
//   Vehicle(int num);
//   ~Vehicle();

//   char *get_name();
// };

// class Vehicle_list {
//  private:
//   int howmany;
//   Vehicle **vehicle_ptr;

//  public:
//   Vehicle_list();
//   ~Vehicle_list();
//   void Add_vehicle(Vehicle *ptr);
// };
#endif  // Vehicle_list__HH