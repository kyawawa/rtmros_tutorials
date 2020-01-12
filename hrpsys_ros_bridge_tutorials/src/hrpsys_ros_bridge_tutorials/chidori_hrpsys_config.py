#!/usr/bin/env python

from urata_hrpsys_config import *

class CHIDORIHrpsysConfigurator(URATAHrpsysConfigurator):
    """
    Subclass for CHIDORI configuration.
    Please inherit this class to specify environmnet-dependent class.
    """

    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "CHIDORI")

    def getRTCList (self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            # ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            # ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['ic', "ImpedanceController"],
            # ['abc', "AutoBalancer"],
            # ['st', "Stabilizer"],
            ['abst', "AutoBalanceStabilizer"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['co', "CollisionDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def resetPose(self):
        return [0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, # rleg
                0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0] # lleg

    def resetLandingPose (self):
        # handmade
        return [0.0, 0.0, -0.698132, 1.39626, -0.698132, 0.0, # rleg
                0.0, 0.0, -0.698132, 1.39626, -0.698132, 0.0] # lleg

    def startAutoBalancer(self, limbs=None):
        '''!@brief
        Start AutoBalancer mode
        @param limbs list of end-effector name to control.
        If Groups has rarm and larm, rleg, lleg, rarm, larm by default.
        If Groups is not defined or Groups does not have rarm and larm, rleg and lleg by default.
        '''
        if limbs==None:
            if self.Groups != None and "rarm" in map (lambda x : x[0], self.Groups) and "larm" in map (lambda x : x[0], self.Groups):
                limbs=["rleg", "lleg", "rarm", "larm"]
            else:
                limbs=["rleg", "lleg"]
        self.abst_svc.startAutoBalancer(limbs)

    def stopAutoBalancer(self):
        '''!@brief
        Stop AutoBalancer mode
        '''
        self.abst_svc.stopAutoBalancer()

    def startStabilizer(self):
        '''!@brief
        Start Stabilzier mode
        '''
        self.abst_svc.startStabilizer()

    def stopStabilizer(self):
        '''!@brief
        Stop Stabilzier mode
        '''
        self.abst_svc.stopStabilizer()


    # Override parameter setter / getter for AutoBalancer and Stabilizer
    def getABCParameters(self):
        return self.abst_svc.getAutoBalancerParam()[1]

    def getGaitGeneraterParameters(self):
        return self.abst_svc.getGaitGeneratorParam()[1]

    def getSTParameters(self):
        return self.abst_svc.getStabilizerParam()

    def setABCParameters(self, param):
        return self.abst_svc.setAutoBalancerParam(param)

    def setGaitGeneraterParameters(self, param):
        return self.abst_svc.setGaitGeneratorParam(param)

    def setSTParameters(self, param):
        self.abst_svc.setStabilizerParam(param)

    def defJointGroups(self):
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        self.Groups = [rleg_group, lleg_group]

    def setDefaultKFParameters(self):
        kfp = self.getKFParameters()
        kfp.R_angle=1000
        self.setKFParameters(kfp)

    def setDefaultABCParameters(self):
        pass
        # abcp = self.getABCParameters()
        # #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0]]
        # abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        # abcp.move_base_gain=0.8
        # self.setABCParameters(abcp)

    def setDefaultGaitGeneraterParameters(self):
        # gg = self.getGaitGeneraterParameters()
        # gg.default_step_time=1.2
        # # gg.default_double_support_ratio=0.32
        # gg.default_double_support_ratio=0.35
        # # gg.stride_parameter=[0.1,0.05,10.0]
        # # gg.default_step_time=1.0
        # gg.swing_trajectory_delay_time_offset=0.2
        # gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        # gg.swing_trajectory_final_distance_weight=3.0
        # gg.default_orbit_type = OpenHRP.AutoBalanceStabilizerService.CYCLOIDDELAY
        # gg.toe_pos_offset_x = 1e-3*117.338;
        # gg.heel_pos_offset_x = 1e-3*-116.342;
        # gg.toe_zmp_offset_x = 1e-3*117.338;
        # gg.heel_zmp_offset_x = 1e-3*-116.342;
        # self.setGaitGeneraterParameters(gg)
        pass

    def setDefaultSTParameters(self):
        stp = self.getSTParameters()
        stp.st_algorithm=OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=[[20*1.6*1.1*1.5*0.5, 20*1.6*1.1*1.5*0.5, 1e5]]*2
        stp.eefm_pos_damping_gain=[[3500*1.6*3, 3500*1.6*3, 3500*1.6*1.1*1.5*0.5]]*2
        stp.eefm_rot_time_const=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*2
        stp.eefm_pos_time_const_support=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*2
        stp.eefm_use_swing_damping=True
        stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
        stp.eefm_rot_compensation_limit = [math.radians(30), math.radians(30)]
        stp.eefm_pos_compensation_limit = [0.05, 0.05]
        stp.eefm_ee_error_cutoff_freq=20.0
        stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*2
        stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*2
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        ## KAWADA foot
        #   mechanical param is => inside 0.055, front 0.13, rear 0.1
        # stp.eefm_leg_inside_margin=0.05
        # #stp.eefm_leg_inside_margin=0.04
        # stp.eefm_leg_front_margin=0.12
        # stp.eefm_leg_rear_margin=0.09
        tmp_leg_inside_margin=0.05
        tmp_leg_outside_margin=0.05
        tmp_leg_front_margin=0.12
        tmp_leg_rear_margin=0.09
        # JSK foot
        # # mechanical param is -> inside 0.075, front 0.11, rear 0.11
        # stp.eefm_leg_inside_margin=0.07
        # stp.eefm_leg_front_margin=0.1
        # stp.eefm_leg_rear_margin=0.1
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalanceStabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices])
        stp.eefm_cogvel_cutoff_freq = 4.0
        # calculated by calculate-eefm-st-state-feedback-default-gain-from-robot *chidori*
        stp.eefm_k1=[-1.38444,-1.38444]
        stp.eefm_k2=[-0.368975,-0.368975]
        stp.eefm_k3=[-0.169915,-0.169915]
        self.setSTParameters(stp)

    def setDefaultABSTParameters(self):
        self.setDefaultABCParameters()
        self.setDefaultGaitGeneraterParameters()
        self.setDefaultSTParameters()

if __name__ == '__main__':
    hcf = CHIDORIHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
