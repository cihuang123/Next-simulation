#ifndef EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
#define EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
#include "sirius_utility.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern Rocket_SimObject rkt;
const double LONX           = -120.49;  //  Vehicle longitude - deg  module newton
const double LATX           = 34.68;   //  Vehicle latitude  - deg  module newton
const double ALT            = 100.0;         //  Vehicle altitude  - m  module newton
const double PHIBDX         = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX         = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX         = -83.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X        = 0;    // Initial angle-of-attack   - deg  module newton
const double BETA0X         = 0;    // Initial sideslip angle    - deg  module newton
const double DVBE           = 1.0;    // Vehicle geographic speed  - m/s  module newton
const double RCS_DEADZONE   = 0.1;   //  Dead zone of Schmitt trigger
const double RCS_HYSTERESIS = 0.4;   //  Hysteresis of Schmitt trigger
const double RCS_THRUST     = 5.0;   //  RCS thrust
const double RCS_D          = 1.016;   
const unsigned int CYCLE    = 2;
/* S1 */
const double S1_XCG_0          = 10.53;    //  vehicle initial xcg
const double S1_XCG_1          = 6.76;     //  vehicle final xcg
const double S1_MOI_ROLL_0     = 21.94e3;    //  vehicle initial moi in roll direction
const double S1_MOI_ROLL_1     = 6.95e3;     //  vehicle final moi in roll direction
const double S1_MOI_PITCH_0    = 671.62e3;  //  vehicle initial transverse moi
const double S1_MOI_PITCH_1    = 158.83e3;   //  vehicle final transverse moi
const double S1_MOI_YAW_0      = 671.62e3;  //  vehicle initial transverse moi
const double S1_MOI_YAW_1      = 158.83e3;   //  vehicle final transverse moi
const double S1_SPI            = 279.2;     //  Specific impusle
const double S1_FUEL_FLOW_RATE = 514.1;     //  fuel flow rate
const double S1_XCP            = 8.6435;    //  Xcp location
const double S1_refa           = 3.243;      //  Aerodynamics reference area
const double S1_refd           = 2.032;     //  Aerodynamics reference length
const double S1_vmass0         = 48984.0;   //  Vehicle init mass
const double S1_fmass0         = 31175.0;   //  Vehicle init fuel mass
const double S1_RP             = -16.84;      //  reference point
/* S2 */
const double S2_XCG_0          = 6.1736;    //  vehicle initial xcg
const double S2_XCG_1          = 5.914;     //  vehicle final xcg
const double S2_MOI_ROLL_0     = 860.06;    //  vehicle initial moi in roll direction
const double S2_MOI_ROLL_1     = 357.61;     //  vehicle final moi in roll direction
const double S2_MOI_PITCH_0    = 18932.79;  //  vehicle initial transverse moi
const double S2_MOI_PITCH_1    = 16238.01;   //  vehicle final transverse moi
const double S2_MOI_YAW_0      = 18932.79;  //  vehicle initial transverse moi
const double S2_MOI_YAW_1      = 16238.01;   //  vehicle final transverse moi
const double S2_SPI            = 280.0;     //  Specific impusle
const double S2_FUEL_FLOW_RATE = 26.92;     //  fuel flow rate
const double S2_XCP            = 5.0384;    //  Xcp location
const double S2_refa           = 1.45;      //  Aerodynamics reference area
const double S2_refd           = 1.36;     //  Aerodynamics reference length
const double S2_vmass0         = 3844.1;   //  Vehicle init mass
const double S2_fmass0         = 2880.44;   //  Vehicle init fuel mass
const double S2_RP             = -9.749;     //  Reference point
/* S3 */
const double S3_XCG_0          = 2.6651;    //  vehicle initial xcg
const double S3_XCG_1          = 2.66507;     //  vehicle final xcg
const double S3_MOI_ROLL_0     = 74.35;    //  vehicle initial moi in roll direction
const double S3_MOI_ROLL_1     = 37.18;     //  vehicle final moi in roll direction
const double S3_MOI_PITCH_0    = 329.16;  //  vehicle initial transverse moi
const double S3_MOI_PITCH_1    = 164.18;   //  vehicle final transverse moi
const double S3_MOI_YAW_0      = 329.16;  //  vehicle initial transverse moi
const double S3_MOI_YAW_1      = 164.18;   //  vehicle final transverse moi
const double S3_SPI            = 290.0;     //  Specific impusle
const double S3_FUEL_FLOW_RATE = 2.07;     //  fuel flow rate
const double S3_XCP            = 3.2489;    //  Xcp location
const double S3_refa           = 1.45;      //  Aerodynamics reference area
const double S3_refd           = 1.36;     //  Aerodynamics reference length
const double S3_vmass0         = 346.3;   //  Vehicle init mass
const double S3_fmass0         = 248.4;   //  Vehicle init fuel mass
const double S3_RP             = -3.86;    //  Reference point

extern "C" int event_start() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_LIFTOFF, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);
    rkt.propulsion.engine_ignition();
    rkt.tvc.set_S1_TVC();
    return 0;
}


extern "C" int event_separation_1() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_S1_SEPERATION, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S1_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);

    rkt.aerodynamics.set_refa(S2_refa);
    rkt.aerodynamics.set_refd(S2_refd);
    rkt.aerodynamics.load_aerotable("../../../tables/Aero_Insertion_S2.txt");
    rkt.propulsion.set_stage_2();
    // rkt.forces.set_reference_point(-3.917);  // set reference point
    rkt.dynamics.set_reference_point(S2_RP);
    rkt.tvc.set_S2_TVC();
    rkt.propulsion.engine_ignition();
    return 0;
}

extern "C" int event_separation_2() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_S2_SEPERATION, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S2_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);

    rkt.aerodynamics.set_refa(S3_refa);
    rkt.aerodynamics.set_refd(S3_refd);
    rkt.aerodynamics.load_aerotable("../../../tables/Aero_Insertion_S3.txt");
    rkt.propulsion.set_stage_3();
    // rkt.forces.set_reference_point(-3.917);  // set reference point
    rkt.dynamics.set_reference_point(S3_RP);
    rkt.tvc.set_S3_TVC();
    // rkt.propulsion.engine_ignition();
    return 0;
}

extern "C" int event_S3_ignition() {
    rkt.propulsion.engine_ignition();
    return 0;
}


extern "C" void master_startup(Rocket_SimObject *rkt) {
    rkt->egse_flight_event_handler_bitmap &= ~(0x1U << 0);
}

extern "C" int master_model_configuration(Rocket_SimObject *rkt) {
    // rkt->forces.set_Slosh_flag(0);
    rkt->dynamics.set_DOF(6);
    // rkt->forces.set_damping_ratio(0.005);
    // rkt->forces.set_TWD_flag(0);
    rkt->dynamics.set_aero_flag(1);
    rkt->dynamics.set_liftoff(0);  // 1 only for test
}

extern "C" void master_init_time(Rocket_SimObject *rkt) {
    /********************************Set simulation start time*****************************************************/
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    rkt->time->load_start_time(Year, DOY, Hour, Min, Sec);
}

extern "C" void master_init_environment(Rocket_SimObject *rkt) {
    /***************************************environment*************************************************************/
    rkt->env.set_RNP();
    rkt->env.atmosphere_use_nasa();
    rkt->env.set_no_wind();
    rkt->env.set_no_wind_turbulunce();
    rkt->gps_con.readfile("../../../tables/brdc0810.17n");
}

extern "C" void master_init_slv(Rocket_SimObject *rkt) {
    /****************************************SLV************************************************************************/
    double lonx       = LONX;  //  Vehicle longitude - deg  module newton
    double latx       = LATX;   //  Vehicle latitude  - deg  module newton
    double alt        = ALT;         //  Vehicle altitude  - m  module newton
    rkt->dynamics.load_location(lonx, latx, alt);

    double phibdx = PHIBDX;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = THTBDX;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = PSIBDX;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    rkt->dynamics.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x    = ALPHA0X;   // Initial angle-of-attack   - deg  module newton
    double beta0x     = BETA0X;   // Initial sideslip angle    - deg  module newton
    double dvbe       = DVBE;   // Vehicle geographic speed  - m/s  module newton
    rkt->dynamics.load_geodetic_velocity(alpha0x, beta0x, dvbe);
    rkt->dynamics.load_angular_velocity(0, 0, 0);
}

extern "C" void master_init_aerodynamics(Rocket_SimObject *rkt) {
    /************************************aerodynamics*******************************************************/
    rkt->aerodynamics.load_aerotable("../../../tables/aero_table_slv3.txt");
    rkt->aerodynamics.set_refa(S1_refa);       // Reference area for aero coefficients - m^2
    rkt->aerodynamics.set_refd(S1_refd);     // Reference length for aero coefficients - m
    rkt->aerodynamics.set_XCP(S1_XCP);
    /********************************************************************************************************/
}

extern "C" void master_init_propulsion(Rocket_SimObject *rkt) {
    /******************************propulsion & mass property***************************************************************************/
    rkt->propulsion.Allocate_stage(3);
    rkt->propulsion.set_stage_var(S1_SPI, S1_fmass0, S1_vmass0, 0.0, S1_FUEL_FLOW_RATE, S1_XCG_0, S1_XCG_1,
                                     S1_MOI_ROLL_0, S1_MOI_ROLL_1, S1_MOI_PITCH_0, S1_MOI_PITCH_1, S1_MOI_YAW_0, S1_MOI_YAW_1, 
                                     0);
    // rkt->dynamics.set_reference_point(S1_RP);
    rkt->dynamics.set_reference_point_eq_xcg();
    rkt->propulsion.set_stage_1();
    rkt->propulsion.set_no_thrust();
}

extern "C" void master_init_sensors(Rocket_SimObject *rkt) {
    /**************************************************Sensor****************************************************************/
    // Accelerometer
    double EMISA[3];      // gauss(0, 1.1e-4)
    double ESCALA[3];      // gauss(0, 2.e-5)
    double EBIASA[3];      // gauss(0, 1.e-6)

    // Create a Ideal Accelerometer
    rkt->accelerometer = new AccelerometerIdeal(rkt->data_exchang);

    // gyro
    double EMISG[3];      // gauss(0, 1.1e-4)
    double ESCALG[3];      // gauss(0, 2.e-5)
    double EBIASG[3];      // gauss(0, 1.e-6)

    // Create a Ideal Gyro
    rkt->gyro = new GyroIdeal(rkt->data_exchang);

    // rkt->sdt = new SDT_NONIDEAL();
    rkt->sdt = new SDT_ideal(rkt->data_exchang, CYCLE);
}

extern "C" void master_init_tvc(Rocket_SimObject *rkt) {
    /****************************************************TVC*************************************************************************/
    rkt->tvc.Allocate_ENG(1, rkt->tvc.S1_Eng_list);
    rkt->tvc.Allocate_ENG(1, rkt->tvc.S2_Eng_list);
    rkt->tvc.Allocate_ENG(1, rkt->tvc.S3_Eng_list);
    
    // Allocate S1 Engine position
    rkt->tvc.S1_Eng_list[0]->set_ENG_HINGE_POS(S1_RP, 0.0, 0.0);

    // Allocate S1 Engine gimbal direction
    rkt->tvc.S1_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S2 Engine position
    rkt->tvc.S2_Eng_list[0]->set_ENG_HINGE_POS(S2_RP, 0.0, 0.0);

    // Allocate S2 Engine gimbal direction
    rkt->tvc.S2_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S3 Engine position
    rkt->tvc.S3_Eng_list[0]->set_ENG_HINGE_POS(S3_RP, 0.0, 0.0);

    // Allocate S3 Engine gimbal direction
    rkt->tvc.S3_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S1 Actuator
    for (int i = 0; i < rkt->tvc.S1_Eng_list.size(); i++) rkt->tvc.S1_Eng_list[i]->Allocate_Actuator(2, NO_DYN);

    // Allocate S1 Actuator variables
    for (int i = 0; i < rkt->tvc.S1_Eng_list.size(); i++) {
        for (int j = 0; j < rkt->tvc.S1_Eng_list[i]->Act_list.size(); j++) {
            rkt->tvc.S1_Eng_list[i]->Act_list[j]->set_1st_act_var(7.0 * RAD, 160000.0 * RAD, 360000.0 * RAD, 20.0);
        }
    }
    
    // Allocate S2 Actuator
    for (int i = 0; i < rkt->tvc.S2_Eng_list.size(); i++) rkt->tvc.S2_Eng_list[i]->Allocate_Actuator(2, NO_DYN);

    // Allocate S2 Actuator variables
    // for (int i = 0; i < rkt->tvc.S2_Eng_list.size(); i++) {
    //     for (int j = 0; j < rkt->tvc.S2_Eng_list[i]->Act_list.size(); j++) {
    //         rkt->tvc.S2_Eng_list[i]->Act_list[j]->set_1st_act_var(7.0 * RAD, 16.0 * RAD, 360.0 * RAD, 20.0);
    //     }
    // }

    // Allocate S3 Actuator
    for (int i = 0; i < rkt->tvc.S3_Eng_list.size(); i++) rkt->tvc.S3_Eng_list[i]->Allocate_Actuator(2, NO_DYN);

    // Allocate S3 Actuator variables
    // for (int i = 0; i < rkt->tvc.S3_Eng_list.size(); i++) {
    //     for (int j = 0; j < rkt->tvc.S3_Eng_list[i]->Act_list.size(); j++) {
    //         rkt->tvc.S3_Eng_list[i]->Act_list[j]->set_1st_act_var(7.0 * RAD, 16.0 * RAD, 360.0 * RAD, 20.0);
    //     }
    // }
}

extern "C" void master_init_rcs(Rocket_SimObject *rkt) {
    rkt->rcs.Allocate_RCS(3, rkt->rcs.Thruster_list);

    // Thruster 1
    rkt->rcs.Thruster_list[0]->set_thruster_var(RCS_DEADZONE, RCS_HYSTERESIS, RCS_THRUST, 0);
    rkt->rcs.Thruster_list[0]->set_mom_max(100.0, 0.0, 0.0);

    // Thruster 2
    rkt->rcs.Thruster_list[1]->set_thruster_var(RCS_DEADZONE, RCS_HYSTERESIS, RCS_THRUST, 1);
    rkt->rcs.Thruster_list[1]->set_mom_max(0.0, 200000.0, 0.0);

    // Thruster 3
    rkt->rcs.Thruster_list[2]->set_thruster_var(RCS_DEADZONE, RCS_HYSTERESIS, RCS_THRUST, 2);
    rkt->rcs.Thruster_list[2]->set_mom_max(0.0, 0.0, 200000.0);
}

extern "C" void flight_events_handler_configuration(Rocket_SimObject *rkt) {
    /* events */
    jit_add_event("event_start", "LIFTOFF", 0.005);
    // jit_add_event("event_separation_1", "S2", 0.005);
    // jit_add_event("event_separation_2", "S3", 0.005);
    // jit_add_event("event_S3_ignition", "S3IG", 0.005);
    // jit_add_read(102.051 + rkt->stand_still_time, "event_S3_ignition");
    // jit_add_read(107.001, "event_fairing_separation");
    // jit_add_event("event_fairing_separation", "FAIRING_JETTSION", 0.005);
    // jit_add_event("event_hot_staging", "HOT_STAGING", 0.005);
    exec_set_terminate_time(12.001  + rkt->stand_still_time);
}

#endif  //  EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
