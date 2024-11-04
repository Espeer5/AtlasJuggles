# Useful constants

GRAV = -9.81

# All joint names in the robot
JOINT_NAMES = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']

# The frames used to strike the ball with their corresponding chain links
STRIKE_FRAMES = {
    'head' : ("pelvis", ['back_bkz', 'back_bky', 'back_bkx', 'neck_ry']),
    'r_foot': ("pelvis", ['r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky',
                'r_leg_akx']),
    'r_hand': ("utorso", ['r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry',
                'r_arm_wrx', 'r_arm_wry2']),
    'l_foot': ("pelvis", ['l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky',
                'l_leg_akx']),
    'l_hand': ("utorso", ['l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry',
                'l_arm_wrx', 'l_arm_wry2']),
    'r_lleg': ("pelvis", ['r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny']),
    'l_lleg': ("pelvis", ['l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny']),
}

BALL_RAD = 0.1

BALL_ELAS = 0.7

LAMBDA = 20
