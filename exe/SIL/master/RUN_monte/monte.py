import time

#execfile("Modified_data/realtime.py")
execfile("Modified_data/pr_noise.dr")
execfile("Modified_data/test.dr")
#trick.exec_set_enable_freeze(True)
#trick.exec_set_freeze_command(True)
#trick.sim_control_panel_set_enabled(True)

##########MONTE CORAL############
RUN_NUM = 20
RUNNER = 20

trick.mc_set_enabled(1)
trick.mc_set_num_runs(RUN_NUM)
trick.mc_set_timeout(1200)

slave = []
for i in range(0, RUNNER):
    slave.append(trick.MonteSlave("localhost"))
    slave[i].thisown = 0 # tell Python not to free the underlying C++ class when the wrapper is garbage collected
    trick_mc.mc.add_slave(slave[i])

##########################################################
#Event0:control_on
control_on = trick.new_event("control_on")
control_on.set_cycle(0.001)
control_on.condition(0, "trick.exec_get_sim_time() == 5.001")
control_on.action(0, "rkt.rcs.mrcs_moment = 20")
control_on.action(1, "rkt.control.maut = 53")
control_on.action(2, "rkt.control.ancomx = -0.0")
control_on.action(3, "rkt.tvc.mtvc = 2")
trick.add_event(control_on)
control_on.activate()
#Event1:Stage 2 ignition
speration_1 = trick.new_event("speration_1")
speration_1.set_cycle(0.001)
speration_1.condition(0, "trick.exec_get_sim_time() == 101.001")
speration_1.action(0, "rkt.aerodynamics.refa = 1.13")
speration_1.action(1, "rkt.propulsion.aexit = 0")
speration_1.action(2, "rkt.aerodynamics.xcg_ref = 3.429")
speration_1.action(3, "rkt.propulsion.vmass0 = 3893")
speration_1.action(4, "rkt.propulsion.fmass0 = 2963")
speration_1.action(5, "rkt.propulsion.fmasse = 0")
speration_1.action(6, "rkt.propulsion.xcg_0 = 5.829")
speration_1.action(7, "rkt.propulsion.xcg_1 = 4.683")
speration_1.action(8, "rkt.propulsion.moi_roll_0 = 615.2")
speration_1.action(9, "rkt.propulsion.moi_roll_1 = 151.0")
speration_1.action(10, "rkt.propulsion.moi_trans_0 = 7407.4")
speration_1.action(11, "rkt.propulsion.moi_trans_1 = 3752.1")
speration_1.action(12, "rkt.propulsion.spi = 290")
speration_1.action(13, "rkt.propulsion.fuel_flow_rate = 29.63")
speration_1.action(14, "rkt.propulsion.mprop = 3")
speration_1.action(15, "rkt.rcs.mrcs_moment = 23")
speration_1.action(16, "trick.add_event(speration_2)")
speration_1.action(17, "speration_2.activate()")
speration_1.action(18, "rkt.aerodynamics.maero = 12")
trick.add_event(speration_1)
speration_1.activate()
###############################################################
#Event2:Fairing speration
speration_2=trick.new_event("speration_2")
speration_2.set_cycle(0.001)
speration_2.condition(0, "trick.exec_get_sim_time() == 170.001")
speration_2.action(0, "rkt.propulsion.vmass0 = 3863")
speration_2.action(1, "trick.add_event(speration_3)")
speration_2.action(2, "speration_3.activate()")
##############################################################
#Event3:Stage 3 ignition
speration_3=trick.new_event("speration_3")
speration_3.set_cycle(0.001)
speration_3.condition(0, "rkt.newton.thtvdx < 3.728")
speration_3.action(0, "rkt.rcs.rcs_thrust = 10")
speration_3.action(1, "rkt.aerodynamics.xcg_ref = 3.2489")
speration_3.action(2, "rkt.rcs.roll_mom_max = 10")
speration_3.action(3, "rkt.rcs.pitch_mom_max = 100")
speration_3.action(4, "rkt.rcs.yaw_mom_max = 20")
speration_3.action(5, "rkt.propulsion.vmass0 = 490")
speration_3.action(6, "rkt.propulsion.fmass0 = 360")
speration_3.action(7, "rkt.propulsion.fmasse = 0")
speration_3.action(8, "rkt.propulsion.xcg_0 = 1.942")
speration_3.action(9, "rkt.propulsion.xcg_1 = 2.02")
speration_3.action(10, "rkt.propulsion.moi_roll_0 = 72.7")
speration_3.action(11, "rkt.propulsion.moi_roll_1 = 61.9")
speration_3.action(12, "rkt.propulsion.moi_trans_0 = 140.7")
speration_3.action(13, "rkt.propulsion.moi_trans_1 = 88.8")
speration_3.action(14, "rkt.propulsion.spi = 300")
speration_3.action(15, "rkt.propulsion.fuel_flow_rate = 3.33")
speration_3.action(16, "rkt.propulsion.mprop = 3")
speration_3.action(17, "trick.add_event(speration_4)")
speration_3.action(18, "speration_4.activate()")
speration_3.action(19, "rkt.aerodynamics.maero = 11")
speration_3.action(20, "rkt.rcs.mrcs_moment = 23")
#############################################################
#Event4:MECO
speration_4=trick.new_event("speration_4")
speration_4.set_cycle(0.05)
speration_4.condition(0, "rkt.newton.dvbi > 7613.5")
speration_4.action(0, "rkt.propulsion.mprop = 0")
speration_4.action(1, "rkt.propulsion.vmass0 = 0")
#############################################################
#Realtime setting
# trick.exec_set_thread_cpu_affinity(0,0)
# trick.exec_set_thread_priority(0,1)
# trick.frame_log_on()
# trick.real_time_enable()
# trick.exec_set_software_frame(0.01)
# trick.itimer_enable()
##############################################################

#SLV
rkt.newton.lonx       = 120.893501    #Vehicle longitude - deg  module newton
rkt.newton.latx       = 22.138917    #Vehicle latitude - deg  module newton
rkt.newton.alt        = 100    #Vehicle altitude - m  module newton
rkt.newton.dvbe       = 2    #Vehicle geographic speed - m/s  module newton

rkt.kinematics.phibdx = 0    #Rolling angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.thtbdx = 86.635    #Pitching angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.psibdx = 90    #Yawing angle of veh wrt geod coord - deg  module kinematics
rkt.newton.alpha0x    = 0    #Initial angle-of-attack - deg  module newton
rkt.newton.beta0x     = 0    #Initial sideslip angle - deg  module newton
#environment
rkt.env.mair = 0              #mair =|matmo|mturb|mwind|
#aerodynamics
rkt.aerodynamics.maero = 13      #=11: last stage; =12: 2 stages; =13: 3 stages
rkt.aerodynamics.xcg_ref = 9.632   #Reference cg location from nose - m
rkt.aerodynamics.refa = 2.36       #Reference area for aero coefficients - m^2
rkt.aerodynamics.refd = 1.7334     #Reference length for aero coefficients - m
rkt.aerodynamics.alplimx = 20      #Alpha limiter for vehicle - deg
rkt.aerodynamics.alimitx = 5       #Structural  limiter for vehicle
#propulsion
rkt.propulsion.mprop  = 3   #'int' =0:none; =3 input; =4 LTG control  module propulsion
rkt.propulsion.vmass0 = 13970       #vehicle initial mass
rkt.propulsion.fmass0 = 8888.9      #vehicle initail fuel mass
rkt.propulsion.xcg_0  = 12.032      #vehicle initial xcg
rkt.propulsion.xcg_1  = 7.965       #vehicle final xcg
rkt.propulsion.moi_roll_0 = 2426.8  #vehicle initial moi in roll direction
rkt.propulsion.moi_roll_1 = 914.5   #vehicle final moi in roll direction
rkt.propulsion.moi_trans_0 = 244537.9   #vehicle initial transverse moi
rkt.propulsion.moi_trans_1 = 87392.2    #vehicle final transverse moi
rkt.propulsion.spi = 255.0          #Specific impusle
rkt.propulsion.fuel_flow_rate = 88.89  #fuel flow rate
rkt.propulsion.aexit = 0.258242843 #nozzle exhaust area
rkt.propulsion.payload = 98 #payload mass
#INS
rkt.ins.mins   = 1
#INS Acceleration
rkt.ins.efspb  = [0, 0, 0]
rkt.ins.ewalka = [0, 0, 0]

EMISA = [0, 0, 0]
for i in range(0, 3):
    EMISA[i] = trick.MonteVarRandom("rkt.ins.emisa[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    EMISA[i].set_seed(i)
    EMISA[i].set_mu(0)
    EMISA[i].set_sigma(1.1e-4)
    trick_mc.mc.add_variable(EMISA[i])
    EMISA[i].thisown = 0

ESCALA = [0, 0, 0]
for i in range(0, 3):
    ESCALA[i] = trick.MonteVarRandom("rkt.ins.escala[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    ESCALA[i].set_seed(i)
    ESCALA[i].set_mu(0)
    ESCALA[i].set_sigma(5e-4)
    trick_mc.mc.add_variable(ESCALA[i])
    ESCALA[i].thisown = 0

EBIASA = [0, 0, 0]
for i in range(0, 3):
    EBIASA[i] = trick.MonteVarRandom("rkt.ins.ebiasa[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    EBIASA[i].set_seed(i)
    EBIASA[i].set_mu(0)
    EBIASA[i].set_sigma(3.56e-3)
    trick_mc.mc.add_variable(EBIASA[i])
    EBIASA[i].thisown = 0

#ins gyrp
rkt.ins.eug    = [0, 0, 0]
rkt.ins.ewg    = [0, 0, 0]
rkt.ins.ewbib  = [0, 0, 0]
rkt.ins.ewalkg = [0, 0, 0]
rkt.ins.eunbg  = [0, 0, 0]

EMISG = [0, 0, 0]
for i in range(0, 3):
    EMISG[i] = trick.MonteVarRandom("rkt.ins.emisg[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    EMISG[i].set_seed(i)
    EMISG[i].set_mu(0)
    EMISG[i].set_sigma(1.1e-4)
    trick_mc.mc.add_variable(EMISG[i])
    EMISG[i].thisown = 0

ESCALG = [0, 0, 0]
for i in range(0, 3):
    ESCALG[i] = trick.MonteVarRandom("rkt.ins.escalg[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    ESCALG[i].set_seed(i)
    ESCALG[i].set_mu(0)
    ESCALG[i].set_sigma(2e-5)
    trick_mc.mc.add_variable(ESCALG[i])
    ESCALG[i].thisown = 0

EBIASG = [0, 0, 0]
for i in range(0, 3):
    EBIASG[i] = trick.MonteVarRandom("rkt.ins.ebiasg[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    EBIASG[i].set_seed(i)
    EBIASG[i].set_mu(0)
    EBIASG[i].set_sigma(1e-6)
    trick_mc.mc.add_variable(EBIASG[i])
    EBIASG[i].thisown = 0

#ins grav
rkt.ins.egravi = [0, 0, 0]

#GPS
gps_sats.sats.almanac_time = 80000    #Time since almanac epoch at sim start - sec  module gps
gps_sats.sats.sv_data = [
        [5.63, -1.600],  # A-plane, slot #1
        [5.63, 2.115],   #              #2
        [5.63, -2.309],  #              #3
        [5.63, 0.319],   #              #4

        [0.40, 1.063],   # B-plane, slot #5
        [0.40, -1.342],  #              #6
        [0.40, 0.543],   #              #7
        [0.40, 2.874],   #              #8

        [1.45, 1.705],   # C-plane, slot #9
        [1.45, -2.841],  #              #10
        [1.45, -2.321],  #              #11
        [1.45, -0.640],  #              #12

        [2.45, 1.941],   # D-plane, slot #13
        [2.45, -0.147],  #              #14
        [2.45, 1.690],   #              #15
        [2.45, 0.409],   #              #16

        [3.48, -0.571],  # E-plane, slot #17
        [3.48, -2.988],  #              #18
        [3.48, 0.858],   #              #19
        [3.48, 2.705],   #              #20

        [4.59, -0.7180],  # F-plane,slot #21
        [4.59, 2.666],    #             #22
        [4.59, -2.977],   #             #23
        [4.59, -0.2090]   #             #24
    ]
rkt.gpsr.slot = [0, 0, 0, 0];  #/< SV slot#  of quadriga

rkt.gpsr.del_rearth        = 2317000    #Delta to Earth's radius for GPS clear LOS signal reception - m  module gps
rkt.gpsr.gps_acqtime       = 10    #Acquisition time for GPS signal - s  module gps
rkt.gpsr.gps_step          = 0.1    #GPS update interval - s  module gps

rkt.gpsr.ucfreq_noise_sigma= 0.1
rkt.gpsr.ucfreq_noise_bcor = 100
ucfreq_noise = trick.MonteVarRandom("rkt.gpsr.ucfreq_noise", trick.MonteVarRandom.GAUSSIAN)
ucfreq_noise.set_mu(0)
ucfreq_noise.set_sigma(rkt.gpsr.ucfreq_noise_sigma)
trick_mc.mc.add_variable(ucfreq_noise)
ucfreq_noise.thisown = 0

ucbias_error = trick.MonteVarRandom("rkt.gpsr.ucbias_error", trick.MonteVarRandom.GAUSSIAN)
ucbias_error.set_mu(0)
ucbias_error.set_sigma(3)
trick_mc.mc.add_variable(ucbias_error)
ucbias_error.thisown = 0

PR_BIAS = [0, 0, 0, 0]
for i in range(0, 4):
    PR_BIAS[i] = trick.MonteVarRandom("rkt.gpsr.PR_BIAS[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    PR_BIAS[i].set_seed(i)
    PR_BIAS[i].set_mu(0)
    PR_BIAS[i].set_sigma(0.842)
    trick_mc.mc.add_variable(PR_BIAS[i])
    PR_BIAS[i].thisown = 0

rkt.gpsr.PR_NOISE_sigma = [0.25, 0.25, 0.25, 0.25]      #Pseudo-range noise - m MARKOV  module gps
rkt.gpsr.PR_NOISE_bcor  = [0.002, 0.002, 0.002, 0.002]  #Pseudo-range noise - m MARKOV  module gps
PR_NOISE = [0, 0, 0, 0]
for i in range(0, 4):
    PR_NOISE[i] = trick.MonteVarRandom("rkt.gpsr.PR_NOISE[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    PR_NOISE[i].set_seed(i)
    PR_NOISE[i].set_mu(0)
    PR_NOISE[i].set_sigma(rkt.gpsr.PR_NOISE_sigma[i])
    trick_mc.mc.add_variable(PR_NOISE[i])
    PR_NOISE[i].thisown = 0

rkt.gpsr.DR_NOISE_sigma = [0.03, 0.03, 0.03, 0.03]      #Delta-range noise - m/s MARKOV  module gps
rkt.gpsr.DR_NOISE_bcor  = [100, 100, 100, 100]          #Delta-range noise - m/s MARKOV  module gps
DR_NOISE = [0, 0, 0, 0]
for i in range(0, 4):
    DR_NOISE[i] = trick.MonteVarRandom("rkt.gpsr.DR_NOISE[" + str(i) + "]", trick.MonteVarRandom.GAUSSIAN)
    DR_NOISE[i].set_seed(i)
    DR_NOISE[i].set_mu(0)
    DR_NOISE[i].set_sigma(rkt.gpsr.DR_NOISE_sigma[i])
    trick_mc.mc.add_variable(DR_NOISE[i])
    PR_BIAS[i].thisown = 0

rkt.gpsr.uctime_cor = 100  #User clock correlation time constant - s=module gps
rkt.gpsr.ppos        = 5  #Init 1sig pos values of state cov matrix - m=module gps
rkt.gpsr.pvel        = 0.2  #Init 1sig vel values of state cov matrix - m/s=module gps
rkt.gpsr.pclockb     = 3  #Init 1sig clock bias error of state cov matrix - m=module gps
rkt.gpsr.pclockf     = 1  #Init 1sig clock freq error of state cov matrix - m/s=module gps
rkt.gpsr.qpos        = 0.1  #1sig pos values of process cov matrix - m=module gps
rkt.gpsr.qvel        = 0.01  #1sig vel values of process cov matrix - m/s=module gps
rkt.gpsr.qclockb     = 0.5  #1sig clock bias error of process cov matrix - m=module gps
rkt.gpsr.qclockf     = 0.1  #1sig clock freq error of process cov matrix - m/s=module gps
rkt.gpsr.rpos        = 1  #1sig pos value of meas cov matrix - m=module gps
rkt.gpsr.rvel        = 0.1  #1sig vel value of meas cov matrix - m/s=module gps
rkt.gpsr.factp       = 0  #Factor to modifiy initial P-matrix P(1+factp)=module gps
rkt.gpsr.factq       = 0  #Factor to modifiy the Q-matrix Q(1+factq)=module gps
rkt.gpsr.factr       = 0  #Factor to modifiy the R-matrix R(1+factr)=module gps
#RCS thruster
rkt.rcs.mrcs_moment = 00 #'int' Attitude control, =|rcs_type||rcs_mode|, see table  module rcs
rkt.rcs.roll_mom_max = 100 #RCS rolling moment max value - Nm  module rcs
rkt.rcs.pitch_mom_max = 200000  #RCS pitching moment max value - Nm  module rcs
rkt.rcs.yaw_mom_max = 200000  #RCS yawing moment max value - Nm  module rcs
rkt.rcs.dead_zone = 0.1   #Dead zone of Schmitt trigger - deg  module rcs
rkt.rcs.hysteresis = 0.1    #Hysteresis of Schmitt trigger - deg  module rcs
rkt.rcs.rcs_tau = 1   #Slope of the switching function - sec  module rcs
rkt.rcs.thtbdcomx = 0    #Pitch angle command - deg  module rcs
rkt.rcs.psibdcomx = -85  #Yaw angle command - deg  module rcs
rkt.rcs.rcs_thrust = 100   #rcs thrust - N  module rcs
rkt.rcs.rcs_pos = 1.66507   #rcs thruster's postion from nose - m  module rcs
rkt.rcs.rocket_r = 0.68  #rocket's radius - m  module rcs
#Guidance
rkt.guidance.alphacomx = 0   #Alpha command - deg  module guidance
rkt.guidance.betacomx = 0    #Beta command - deg  module guidance
#Control
rkt.control.maut = 0  #turn on acceleration autopilot
rkt.control.ancomx = 0 #acceleration command on normal direction
rkt.control.delimx = 10 #eta rate limiter
rkt.control.drlimx = 10 #zeta rate limiter
rkt.control.zaclp = 0.7 #damping ratio of pitch controller
rkt.control.zacly = 0.9 #damping ratio of
#TVC
rkt.tvc.mtvc = 0  #TVC mode switch
rkt.tvc.gtvc = 1
rkt.tvc.parm = 19.25 #TVC moment arm
rkt.tvc.tvclimx = 10  # TVC acuator limiter
rkt.tvc.dtvclimx = 200  #TVC acuator rate limiter
rkt.tvc.zettvc = 0.7  #TVC acuator damping ratio
rkt.tvc.wntvc = 100  #TVC natrual freqency

trick.stop(120)
