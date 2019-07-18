#!/usr/bin/env python

from jaxon_hrpsys_config import *

class JAXON_REDHrpsysConfigurator(JAXONHrpsysConfigurator):
    """
    Subclass for JAXON_RED configuration.
    This class overrides some functions for JAXON_RED.
    Please inherit this class to specify environmnet-dependent class.
    """

    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "JAXON_RED")

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

    def setDefaultForceMomentOffset(self):
        import rospkg
        self.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_JAXON_RED")

    def setDefaultABCParameters(self):
        abcp = self.getABCParameters()
        #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets=[[0.0, 0.01, 0.0], [0.0, -0.01, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.move_base_gain=0.8
        self.setABCParameters(abcp)

    def setDefaultGaitGeneraterParameters(self):
        gg = self.getGaitGeneraterParameters()
        gg.default_step_time=1.2
        gg.default_step_height=0.065
        gg.default_double_support_ratio=0.35
        gg.swing_trajectory_delay_time_offset=0.15
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalanceStabilizerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3 * 117.338;
        gg.heel_pos_offset_x = 1e-3 * -116.342;
        gg.toe_zmp_offset_x = 1e-3 * 117.338;
        gg.heel_zmp_offset_x = 1e-3 * -116.342;
        gg.optional_go_pos_finalize_footstep_num=1
        self.setGaitGeneraterParameters(gg)

    # Override for ABS service
    def setJAXONFootMarginParam(self, foot="KAWADA"):
        stp=self.getSTParameters()
        if foot == "KAWADA":
            ## KAWADA foot : mechanical param is => inside 0.055, front 0.13, rear 0.1
            stp.eefm_leg_inside_margin=0.05
            stp.eefm_leg_outside_margin=0.05
            stp.eefm_leg_front_margin=0.12
            stp.eefm_leg_rear_margin=0.09
        elif foot == "JSK":
            ## JSK foot : mechanical param is -> inside 0.075, front 0.11, rear 0.11
            stp.eefm_leg_inside_margin=0.07
            stp.eefm_leg_outside_margin=0.07
            stp.eefm_leg_front_margin=0.1
            stp.eefm_leg_rear_margin=0.1
        elif foot == "LEPTRINO":
            stp.eefm_leg_inside_margin=0.05
            stp.eefm_leg_outside_margin=0.05
            stp.eefm_leg_front_margin=0.115
            stp.eefm_leg_rear_margin=0.115
        rleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_inside_margin])]
        lleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_outside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalanceStabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        self.setSTParameters(stp)

    def setDefaultSTParameters(self):
        stp = self.getSTParameters()
        stp.st_algorithm=OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP
        stp.emergency_check_mode=OpenHRP.AutoBalanceStabilizerService.CP # enable EmergencyStopper for JAXON @ 2015/11/19
        stp.cp_check_margin=[0.05, 0.045, 0, 0.095]
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        #stp.eefm_body_attitude_control_gain=[0, 0.5]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain = [[20*1.6*1.1*1.5, 20*1.6*1.1*1.5, 1e5],
                                     [20*1.6*1.1*1.5, 20*1.6*1.1*1.5, 1e5],
                                     [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5],
                                     [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5]]
        stp.eefm_pos_damping_gain = [[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5],
                                     [3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5],
                                     [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8],
                                     [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8]]
        stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
        stp.eefm_rot_compensation_limit = [math.radians(10), math.radians(10), math.radians(10), math.radians(10)]
        stp.eefm_pos_compensation_limit = [0.025, 0.025, 0.050, 0.050]
        stp.eefm_swing_damping_force_thre=[200]*3
        stp.eefm_swing_damping_moment_thre=[15]*3
        stp.eefm_use_swing_damping=True
        stp.eefm_ee_error_cutoff_freq=20.0
        stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*4
        stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*4
        stp.eefm_ee_moment_limit = [[90.0,90.0,1e4], [90.0,90.0,1e4], [1e4]*3, [1e4]*3]
        stp.eefm_rot_time_const = [[1.5/1.1, 1.5/1.1, 1.5/1.1]]*4
        stp.eefm_pos_time_const_support = [[3.0/1.1, 3.0/1.1, 1.5/1.1]]*4
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_cogvel_cutoff_freq = 4.0
        stp.eefm_k1=[-1.48412,-1.48412]
        stp.eefm_k2=[-0.486727,-0.486727]
        stp.eefm_k3=[-0.198033,-0.198033]
        self.setSTParameters(stp)
        self.setJAXONFootMarginParam(foot="KAWADA")

    def setDefaultABSTParameters(self):
        self.setDefaultABCParameters()
        self.setDefaultGaitGeneraterParameters()
        self.setDefaultSTParameters()

if __name__ == '__main__':
    hcf = JAXON_REDHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
