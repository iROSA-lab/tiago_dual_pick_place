class ArmConf(object):
    def __init__(self, group_arm, group_arm_torso, grasp_frame, gripper_joints):
        self.group_arm = group_arm
        self.group_arm_torso = group_arm_torso
        self.grasp_frame = grasp_frame
        self.gripper_joints = gripper_joints