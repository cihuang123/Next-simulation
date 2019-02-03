#ifndef EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
#define EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
#include "sirius_utility.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern FlightComputer_SimObject fc;

const double LONX           = -120.49;  //  Vehicle longitude - deg  module newton
const double LATX           = 34.68;   //  Vehicle latitude  - deg  module newton
const double ALT            = 100.0;         //  Vehicle altitude  - m  module newton
const double PHIBDX         = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX         = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX         = -83.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X        = 0;    // Initial angle-of-attack   - deg  module newton
const double BETA0X         = 0;    // Initial sideslip angle    - deg  module newton
const double DVBE           = 1.0;    // Vehicle geographic speed  - m/s  module newton
const double RCS_TAU        = 1.0;  // Slope of the switching function - sec RCS
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
const double S1_VMASS0         = 48984.0;   //  Vehicle init mass
const double S1_FMASS0         = 31175.0;   //  Vehicle init fuel mass
const double S1_RP             = -16.84;      //  reference point
const double S1_ROLL_CMD       = 0.0;       //  Roll command - deg
const double S1_PITCH_CMD      = 80.0;
const double S1_YAW_CMD        = -83.0;
const double S1_ANCOMX         = -0.15;
const double S1_ALCOMX         = 0.0;
const double S1_GAINP          = 0.0;
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
const double S2_VMASS0         = 3844.1;   //  Vehicle init mass
const double S2_FMASS0         = 2880.44;   //  Vehicle init fuel mass
const double S2_RP             = 9.749;
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
const double S3_VMASS0         = 346.3;   //  Vehicle init mass
const double S3_FMASS0         = 248.4;   //  Vehicle init fuel mass
const double S3_RP             = 3.86;
/* Controller setting */
const double ZACLP             = 1.0;   // Damping of accel close loop complex pole - ND
const double ZACLY             = 1.0;   // Damping of accel close loop complex pole - ND
const double FACTWACLP         = 0.5;   // Factor to mod 'waclp': waclp*(1+factwacl) - ND
const double FACTWACLY         = 0.5;   // Factor to mod 'wacly': wacly*(1+factwacl) - ND
const double ALPHACOMX         = 0.0;   // AOA command - deg
const double BETACOMX          = 0.0;   // Side slip angle command - deg
const double ROLLCOMX          = 0.0;   // Roll angle command - deg

extern "C" int event_liftoff(void) {
    fc.ins.set_liftoff(1);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_LIFTOFF;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_LIFTOFF", fc.ctl_tvc_db.flight_event_code);
    fc.rcs_fc.enable_rcs();
    fc.rcs_fc.set_mode(1);
    return 0;
}

extern "C" int event_acc_on(void) {
    fc.control.set_acc_control();
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S1_CONTROL_ON;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S1_CONTROL_ON);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S1_CONTROL_ON", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

// extern "C" int event_s2_control_on(void) {
//     fc.control.set_S2_ROLL_CONTROL();
//     // fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S2_ROLL_CONTROL;
//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S2_ROLL_CONTROL);
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S2_ROLL_CONTROL", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

// extern "C" int event_pitch_down_phase_1(void) {
//     if (fc.ins.get_altc() >= 500.0) {
//         if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_PITCH_DOWN_PHASE_I, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_PITCH_DOWN_PHASE_I))
//         return 0;
//     } else {
//         return 0;
//     }

//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_PITCH_DOWN_PHASE_I);
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_PITCH_DOWN_PHASE_I", FLIGHT_EVENT_PITCH_DOWN_PHASE_I);
//     fc.control.set_S2_PITCH_DOWN_I();
//     return 0;
// }

// extern "C" int event_pitch_down_phase_2(void) {
//     if (fc.ins.get_altc() >= 2000.0) {
//         if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_PITCH_DOWN_PHASE_II, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_PITCH_DOWN_PHASE_II))
//         return 0;
//     } else {
//         return 0;
//     }

//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_PITCH_DOWN_PHASE_II);
//     double S2_rollcmd = S2_ROLLCMD;
//     double S2_yawcmd = S2_YAWCMD;
//     fc.control.set_attcmd(S2_rollcmd, -5.5, S2_yawcmd);
//     fc.control.set_S2_PITCH_DOWN_II();
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_PITCH_DOWN_PHASE_II", FLIGHT_EVENT_PITCH_DOWN_PHASE_II);
//     return 0;
// }

// extern "C" int event_aoac_on(void) {
//     if (fc.ins.get_altc() >= 20000.0 && fc.ins.get_alphacx() <= 1.0) {
//         if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_AOA_CONTROL, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_AOA_CONTROL))
//         return 0;
//     } else {
//         return 0;
//     }

//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_AOA_CONTROL);

//     fc.control.set_S2_AOA();
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_AOA_CONTROL", FLIGHT_EVENT_AOA_CONTROL);
//     return 0;
// }

// extern "C" int event_control_off(void) {
//     fc.control.set_NO_CONTROL();
//     fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_CONTROL_OFF;
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_CONTROL_OFF", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

// extern "C" int event_fairing_jettison(void) {
//     if (fc.ins.get_altc() >= 95000.0) {
//         if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_FAIRING_JETTSION, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_FAIRING_JETTSION))
//         return 0;
//     } else {
//         return 0;
//     }

//     fc.control.set_IBBB0(FARING_MOI_ROLL_0, FARING_MOI_PITCH_0, FARING_MOI_YAW_0);
//     fc.control.set_IBBB1(FARING_MOI_ROLL_1, FARING_MOI_PITCH_1, FARING_MOI_YAW_1);
//     fc.control.set_controller_var(S3_MDOT, S3_FMASS0, FARING_XCG_1, FARING_XCG_0, S3_ISP, S3_MDOT * HS_time);

//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_FAIRING_JETTSION);

//     fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_FAIRING_JETTSION;
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_FAIRING_JETTSION", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

// extern "C" int event_hot_staging(void) {
//     fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_HOT_STAGING;
//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_HOT_STAGING);
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_HOT_STAGING", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

// extern "C" int event_s3_control_on(void) {
//     fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S3_CONTROL_ON;
//     fc.control.set_S3_AOA();
//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S3_CONTROL_ON);
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S3_CONTROL_ON", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

// extern "C" int event_s1_seperation(void) {
//     fc.control.set_controller_var(S3_MDOT, S3_FMASS0, S3_XCG_1, S3_XCG_0, S3_ISP, S3_MDOT * HS_time);
//     fc.control.set_IBBB0(S3_MOI_ROLL_0, S3_MOI_PITCH_0, S3_MOI_YAW_0);
//     fc.control.set_IBBB1(S3_MOI_ROLL_1, S3_MOI_PITCH_1, S3_MOI_YAW_1);
//     fc.control.get_control_gain(S3_KPP, S3_KPI, S3_KPD, S3_KPPP, S3_PN, S3_KRP, S3_KRI, S3_KRD, S3_KRPP, S3_RN, S3_KYP, S3_KYI,
//                                 S3_KYD, S3_KYPP, S3_YN, S3_KAOAP, S3_KAOAI, S3_KAOAD, S3_KAOAPP, S3_AOAN);
//     fc.control.set_attcmd(S3_ROLLCMD, S3_PITCHCMD, S3_YAWCMD);
//     fc.control.set_aoacmd(S3_AOACMD);
//     fc.control.set_ierror_zero();
//     fc.control.set_reference_point(S3_REFERENCE_P);
//     fc.control.set_engine_d(0.0);
//     // fc.control.set_S3_AOA();
//     fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S3_SEPERATION;
//     fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S3_SEPERATION);
//     PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S3_SEPERATION", fc.ctl_tvc_db.flight_event_code);
//     return 0;
// }

extern "C" int slave_init_stage1_control(FlightComputer_SimObject *fc) {
    /* Control variable Stage2 */
    fc->control.set_controller_var(S1_FUEL_FLOW_RATE, S1_FMASS0, S1_XCG_1, S1_XCG_0, S1_SPI, 0.0);
    fc->control.set_IBBB0(S1_MOI_ROLL_0, S1_MOI_PITCH_0, S1_MOI_YAW_0);
    fc->control.set_IBBB1(S1_MOI_ROLL_1, S1_MOI_PITCH_1, S1_MOI_YAW_1);
    fc->rcs_fc.set_rcs_tau(RCS_TAU);
    fc->rcs_fc.set_phibdcomx(S1_ROLL_CMD);
    fc->rcs_fc.set_thtbdcomx(S1_PITCH_CMD);
    fc->rcs_fc.set_psibdcomx(S1_YAW_CMD);

    fc->control.set_ancomx(S1_ANCOMX);
    fc->control.set_alcomx(S1_ALCOMX);
    fc->control.load_aerotable("../../../tables/Aero_Insertion_S1.txt");
    fc->control.atmosphere_use_nasa();
    fc->control.set_close_loop_pole(ZACLP, ZACLY);
    fc->control.set_aero_coffe(S1_refd, S1_refa, S1_XCP);
    fc->control.set_feedforward_gain(S1_GAINP);
    fc->control.set_reference_point(S1_RP);
}

extern "C" int slave_init_ins_variable(FlightComputer_SimObject *fc) {
    fc->ins.load_location(LONX, LATX, ALT);
    fc->ins.load_angle(PSIBDX, PHIBDX, THTBDX);
    fc->ins.load_geodetic_velocity(ALPHA0X, BETA0X, DVBE);
    fc->ins.set_ideal();
    //  fc->ins.set_non_ideal();
    uint32_t gpsupdate  = 0;
    fc->ins.set_gps_correction(gpsupdate);
    return 0;
}

extern "C" int slave_init_gps_fc_variable(FlightComputer_SimObject *fc) {
    /* GPS */
    double pr_bias_default[4] = {0, 0, 0, 0};
    double pr_noise_default[4] = {0.25, 0.25, 0.25, 0.25};
    double dr_noise_default[4] = {0.03, 0.03, 0.03, 0.03};
    //  User clock frequency error - m/s MARKOV  module gps
    fc->gps.ucfreq_noise      = 0.1;
    // User clock bias error - m GAUSS  module gps
    fc->gps.ucbias_error      = 0;
    // Pseudo-range bias - m GAUSS  module gps
    memcpy(fc->gps.PR_BIAS, pr_bias_default, sizeof(pr_bias_default));
    //  Pseudo-range noise - m MARKOV  module gps
    memcpy(fc->gps.PR_NOISE, pr_noise_default, sizeof(pr_noise_default));
    //  Delta-range noise - m/s MARKOV  module gps
    memcpy(fc->gps.DR_NOISE, dr_noise_default, sizeof(dr_noise_default));
    //  Factor to modifiy initial P-matrix P(1+factp)=module gps
    double gpsr_factp       = 0;
    //  Init 1sig clock bias error of state cov matrix - m=module gps
    double gpsr_pclockb     = 3;
    //  Init 1sig clock freq error of state cov matrix - m/s=module gps
    double gpsr_pclockf     = 1;
    fc->gps.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf);

    //  Factor to modifiy the Q-matrix Q(1+factq)=module gps
    double gpsr_factq       = 0;
    //  1sig clock bias error of process cov matrix - m=module gps
    double gpsr_qclockb     = 0.5;
    //  1sig clock freq error of process cov matrix - m/s=module gps
    double gpsr_qclockf     = 0.1;
    fc->gps.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf);

    //  User clock correlation time constant - s=module gps
    double gpsr_uctime_cor = 100;
    fc->gps.setup_fundamental_dynamic_matrix(gpsr_uctime_cor);

    //  Init 1sig pos values of state cov matrix - m=module gps
    fc->gps.ppos        = 5;
    //  Init 1sig vel values of state cov matrix - m/s=module gps
    fc->gps.pvel        = 0.2;
    //  1sig pos values of process cov matrix - m=module gps
    fc->gps.qpos        = 0.1;
    //  1sig vel values of process cov matrix - m/s=module gps
    fc->gps.qvel        = 0.01;
    //  1sig pos value of meas cov matrix - m=module gps
    fc->gps.rpos        = 1;
    //  1sig vel value of meas cov matrix - m/s=module gps
    fc->gps.rvel        = 0.1;
    //  Factor to modifiy the R-matrix R(1+factr)=module gps
    fc->gps.factr       = 0;
    return 0;
}

extern "C" int slave_init_time(FlightComputer_SimObject *fc) {
    unsigned int Year = 2017;
    unsigned int DOY = 81;
    unsigned int Hour = 2;
    unsigned int Min = 0;
    unsigned int Sec = 0;
    fc->time->load_start_time(Year, DOY, Hour, Min, Sec);
    return 0;
}

extern "C" void flight_events_trigger_configuration(FlightComputer_SimObject *fc) {
    /* events */
    jit_add_read(0.001 + fc->stand_still_time, "event_liftoff");
    // jit_add_read(10.001 + fc->stand_still_time, "event_acc_on");
    // jit_add_read(0.001 + fc->stand_still_time, "event_s2_control_on");

    exec_set_terminate_time(10.001 + fc->stand_still_time);
}
#endif  //  EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
