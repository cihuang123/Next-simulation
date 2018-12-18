#ifndef EXE_XIL_COMMON_MODIFIED_DATA_GPS_FC_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_GPS_FC_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_gps_slave() {
    Trick::DRAscii *drg = new Trick::DRAscii("ngps_slave");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.05);
    drg->set_single_prec_only(false);
    // drg->add_variable("fc.time->gpstime.SOW");
    //  drg->add_variable("fc.gps.state_pos");
    //  drg->add_variable("fc.gps.state_vel");
    //  drg->add_variable("fc.ins.ins_pos_err");
    //  drg->add_variable("fc.gps.state_pos");
    //  drg->add_variable("fc.ins.ins_vel_err");
    //  drg->add_variable("fc.gps.state_pos");
    //  drg->add_variable("fc.ins.ins_pose_err");
    //  drg->add_variable("fc.ins.ins_vele_err");
    //  drg->add_variable("fc.ins.ins_phi_err");
    //  drg->add_variable("fc.ins.ins_tht_err");
    //  drg->add_variable("fc.ins.thtvdcx");
    //  drg->add_variable("fc.ins.psivdcx");
    //  drg->add_variable("fc.ins.psibdcx");
    //  drg->add_variable("fc.ins.ins_psi_err");
    //  drg->add_variable("fc.ins._WBICI[0]");
    //  drg->add_variable("fc.ins._WBICI[1]");
    //  drg->add_variable("fc.ins._WBICI[2]");
    //  drg->add_variable("fc.ins._POS_ERR[0]");
    //  drg->add_variable("fc.ins._POS_ERR[1]");
    //  drg->add_variable("fc.ins._POS_ERR[2]");
    drg->add_variable("fc.ins._VBIIC[0]");
    drg->add_variable("fc.ins._VBIIC[1]");
    drg->add_variable("fc.ins._VBIIC[2]");
    drg->add_variable("fc.ins._SBIIC[0]");
    drg->add_variable("fc.ins._SBIIC[1]");
    drg->add_variable("fc.ins._SBIIC[2]");
    drg->add_variable("fc.ins.phibdcx");
    drg->add_variable("fc.ins.thtbdcx");
    drg->add_variable("fc.ins.psibdcx");
    // drg->add_variable("fc.ins._VBEEC[0]");
    // drg->add_variable("fc.ins._VBEEC[1]");
    // drg->add_variable("fc.ins._VBEEC[2]");
    // drg->add_variable("fc.ins._SBEEC[0]");
    // drg->add_variable("fc.ins._SBEEC[1]");
    // drg->add_variable("fc.ins._SBEEC[2]");
    // drg->add_variable("fc.ins.phibdcx");
    // drg->add_variable("fc.ins.thtbdcx");
    // drg->add_variable("fc.ins.psibdcx");
    //  drg->add_variable("fc.gps._ZZ[0]");
    //  drg->add_variable("fc.gps._ZZ[1]");
    //  drg->add_variable("fc.gps._ZZ[2]");
    //  drg->add_variable("fc.gps._ZZ[3]");
    //  drg->add_variable("fc.gps._ZZ[4]");
    //  drg->add_variable("fc.gps._ZZ[5]");
    //  drg->add_variable("fc.gps._ZZ[6]");
    //  drg->add_variable("fc.gps._ZZ[7]");
    //  drg->add_variable("fc.ins._INS_I_ATT_ERR[0]");
    //  drg->add_variable("fc.ins._INS_I_ATT_ERR[1]");
    //  drg->add_variable("fc.ins._INS_I_ATT_ERR[2]");
    //  drg->add_variable("fc.ins._TESTV[0]");
    //  drg->add_variable("fc.ins._TESTV[1]");
    //  drg->add_variable("fc.ins._TESTV[2]");
    //  drg->add_variable("fc.ins.liftoff");
     drg->add_variable("fc.control.theta_a_cmd");
     drg->add_variable("fc.control.theta_c_cmd");
     drg->add_variable("fc.control.theta_b_cmd");
     drg->add_variable("fc.control.theta_d_cmd");
     drg->add_variable("fc.rcs_fc.e_roll");
    //  drg->add_variable("fc.control.fmasse");
    //  drg->add_variable("fc.control._WBICBT[0]");
    //  drg->add_variable("fc.control._WBICBT[1]");
    //  drg->add_variable("fc.control._WBICBT[2]");
    //  drg->add_variable("fc.control.mass_ratio");
     // drg->add_variable("fc.control._CONTROLCMD[0]");
     // drg->add_variable("fc.control._CONTROLCMD[1]");
     // drg->add_variable("fc.control._CONTROLCMD[2]");
    //  drg->add_variable("fc.control._CMDG[0]");
    //  drg->add_variable("fc.control._CMDG[1]");
    //  drg->add_variable("fc.control._CMDG[2]");
     // drg->add_variable("fc.control._euler[0]");
     // drg->add_variable("fc.control._euler[1]");
     // drg->add_variable("fc.control._euler[2]");
     // drg->add_variable("fc.control.pitchcmd_new");
    //  drg->add_variable("fc.control._delta_euler[0]");
    //  drg->add_variable("fc.control._delta_euler[1]");
    //  drg->add_variable("fc.control._delta_euler[2]");
    //  drg->add_variable("fc.control.pitchcmd_new");
    //  drg->add_variable("fc.control.perrorpt");
    //  drg->add_variable("fc.control.perrorit");
    //  drg->add_variable("fc.control.thterrort");
    //  drg->add_variable("fc.control.pitchiout_oldt");
    //  drg->add_variable("fc.control.pdout_oldt");
    //  drg->add_variable("fc.control.kpp");
    //  drg->add_variable("fc.control.kpi");
    //  drg->add_variable("fc.control.kpd");
    //  drg->add_variable("fc.control.kppp");
    //  drg->add_variable("fc.control.pN");
    //  drg->add_variable("fc.control.xcg");
    //  drg->add_variable("fc.control.thrust");
    //  drg->add_variable("fc.control.mass_ratio");
    //  drg->add_variable("fc.control._IBBB2[0]");
    //  drg->add_variable("fc.control._IBBB2[1]");
    //  drg->add_variable("fc.control._IBBB2[2]");
     // drg->add_variable("fc.control._CMDQ[0]");
     // drg->add_variable("fc.control._CMDQ[1]");
     // drg->add_variable("fc.control._CMDQ[2]");
     // drg->add_variable("fc.control._CMDQ[3]");
    //  drg->add_variable("fc.ins._TBDQ[0]");
    //  drg->add_variable("fc.ins._TBDQ[1]");
    //  drg->add_variable("fc.ins._TBDQ[2]");
    //  drg->add_variable("fc.ins._TBDQ[3]");
     drg->add_variable("fc.ins.alphacx");
    //  drg->add_variable("fc.control._TCMDQ[0]");
    //  drg->add_variable("fc.control._TCMDQ[1]");
    //  drg->add_variable("fc.control._TCMDQ[2]");
    //  drg->add_variable("fc.control._TCMDQ[3]");
    //  drg->add_variable("fc.control.errori");
    //  drg->add_variable("fc.control.thterror");
    //  drg->add_variable("fc.control.rollerror");
    //  drg->add_variable("fc.control.yawerror");
    //  drg->add_variable("fc.control.thterror");
    //  drg->add_variable("fc.control.aoaerror");
    //  drg->add_variable("fc.control.lx");
    //  drg->add_variable("fc.ins.alphacx");
     // drg->add_variable("fc.ins._TBIC[0][0]");
     // drg->add_variable("fc.ins._TBIC[0][1]");
     // drg->add_variable("fc.ins._TBIC[0][2]");
     // drg->add_variable("fc.ins._TBIC[1][0]");
     // drg->add_variable("fc.ins._TBIC[1][1]");
     // drg->add_variable("fc.ins._TBIC[1][2]");
     // drg->add_variable("fc.ins._TBIC[2][0]");
     // drg->add_variable("fc.ins._TBIC[2][1]");
     // drg->add_variable("fc.ins._TBIC[2][2]");
     // drg->add_variable("fc.ins._TEIC[0][0]");
     // drg->add_variable("fc.ins._TEIC[0][1]");
     // drg->add_variable("fc.ins._TEIC[0][2]");
     // drg->add_variable("fc.ins._TEIC[1][0]");
     // drg->add_variable("fc.ins._TEIC[1][1]");
     // drg->add_variable("fc.ins._TEIC[1][2]");
     // drg->add_variable("fc.ins._TEIC[2][0]");
     // drg->add_variable("fc.ins._TEIC[2][1]");
     // drg->add_variable("fc.ins._TEIC[2][2]");
    drg->add_variable("fc.ins.loncx");
    drg->add_variable("fc.ins.latcx");
    drg->add_variable("fc.ins.altc");

    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_GPS_FC_H_
