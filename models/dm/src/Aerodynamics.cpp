#include <iostream>
#include "Aerodynamics.hh"

AeroDynamics::AeroDynamics(Propulsion& prop, Data_exchang& input)
    : propulsion(&prop), VECTOR_INIT(XCP, 3) {
  data_exchang = &input;
}

AeroDynamics::AeroDynamics(const AeroDynamics& other)
    : propulsion(other.propulsion), VECTOR_INIT(XCP, 3) {
  this->aerotable = other.aerotable;
  this->refa = other.refa;
  this->refd = other.refd;
  this->cy = other.cy;
  this->cll = other.cll;
  this->clm = other.clm;
  this->cln = other.cln;
  this->cx = other.cx;
  this->cz = other.cz;
  this->cn = other.cn;
  this->ca = other.ca;
  this->ca_on = other.ca_on;
  this->cnq = other.cnq;
  this->cmq = other.cmq;
  this->XCP = other.XCP;
  this->data_exchang = other.data_exchang;
}

AeroDynamics& AeroDynamics::operator=(const AeroDynamics& other) {
  if (&other == this) return *this;

  this->propulsion = other.propulsion;
  this->data_exchang = other.data_exchang;
  this->aerotable = other.aerotable;

  this->refa = other.refa;
  this->refd = other.refd;
  this->cy = other.cy;
  this->cll = other.cll;
  this->clm = other.clm;
  this->cln = other.cln;
  this->cx = other.cx;
  this->cz = other.cz;
  this->cn = other.cn;
  this->ca = other.ca;
  this->ca_on = other.ca_on;
  this->cnq = other.cnq;
  this->cmq = other.cmq;
  this->XCP = other.XCP;

  return *this;
}

void AeroDynamics::init() {}

void AeroDynamics::load_aerotable(const char* filename) {
  aerotable = Datadeck(filename);
}

void AeroDynamics::set_refa(double in) { refa = in; }
void AeroDynamics::set_refd(double in) { refd = in; }
void AeroDynamics::set_XCP(double in) { XCP(0) = in; }
void AeroDynamics::algorithm(double int_step) {
  /* only calculate when rocket liftoff */
  int liftoff;
  int thrust_on = 0;
  data_exchang->hget("liftoff", &liftoff);

  if (liftoff == 1) {
    double vmach, dvba, alppx, phipx, alt;
    data_exchang->hget("alppx", &alppx);
    data_exchang->hget("phipx", &phipx);
    data_exchang->hget("vmach", &vmach);
    data_exchang->hget("dvba", &dvba);
    data_exchang->hget("alt", &alt);

    arma::vec3 WBIB;
    arma::vec3 WBEB;
    arma::vec3 XCG;
    data_exchang->hget("XCG", XCG);
    data_exchang->hget("WBEB", WBEB);
    data_exchang->hget("WBIB", WBIB);

    enum Propulsion::THRUST_TYPE thrust_state = propulsion->get_thrust_state();
    // double vmass  = grab_vmass();

    //  transforming body rates from body -> aeroballistic coord.
    double phip = phipx * RAD;
    double cphip = cos(phip);
    double sphip = sin(phip);
    double qqax = WBEB(1) * cphip - WBEB(2) * sphip;
    double rrax = WBEB(1) * sphip + WBEB(2) * cphip;

    // looking up axial force coefficients
    ca0 = aerotable.look_up("ca0_vs_mach", vmach, 0);
    caa = aerotable.look_up("caa_vs_mach", vmach, 0);
    ca0b = aerotable.look_up("ca0b_vs_mach", vmach, 0);

    // axial force coefficient
    if (thrust_state = Propulsion::NO_THRUST) thrust_on = 1;
    ca = ca0 + caa * alppx + thrust_on * ca0b;

    // looking up normal force coefficients in aeroballistic coord
    cn0 = aerotable.look_up("cn0_vs_mach_alpha", vmach, alppx, 0);
    // normal force coefficient
    cna = cn0;

    // looking up pitching moment coefficients in aeroballistic coord
    clm0 = aerotable.look_up("clm0_vs_mach_alpha", vmach, alppx, 0);
    clmq = aerotable.look_up("clmq_vs_mach", vmach, 0);
    // pitching moment coefficient
    double clmaref = clm0 + clmq * qqax * refd / (2. * dvba);
    clma = clmaref - cna * (XCP(0) + XCG(0)) / refd;

    // converting force and moment coeff to body axes
    // force coefficients in body axes
    cx = -ca;
    cy = -cna * sphip;
    cz = -cna * cphip;
    // moment coefficient in body axes
    cll = 0;
    clm = clma * cphip;
    cln = -clma * sphip;
  }

  data_exchang->hset("refa", refa);
  data_exchang->hset("refd", refd);
  data_exchang->hset("cx", cx);
  data_exchang->hset("cy", cy);
  data_exchang->hset("cz", cz);
  data_exchang->hset("cll", cll);
  data_exchang->hset("clm", clm);
  data_exchang->hset("cln", cln);
  data_exchang->hset("XCP", XCP);
}
