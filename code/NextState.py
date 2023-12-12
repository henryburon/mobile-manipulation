import modern_robotics as mr
import numpy as np

# Milestone 1: youBot Kinematics Simulator and csv output

################################################################################
# NextState Function
################################################################################


def NextState(current_config, controls, dt, max_speed):
    #############################################################################################
    # INPUTS
    # current_config: 12-vector (3 for chassis config, 5 for arm config, 4 for wheel angles)
    # controls: 9-vector (4 for wheel speeds, 5 for arm joint speeds)
    # dt: timestamp
    # max_speed: max angular speed of the arm joints and wheels

    # OUTPUT
    # new_config: 12-vector (new_chassis_config, new_arm_config, new_wheel_angles)
    #############################################################################################

    # Extract individual configs
    chassis_config = current_config[:3]
    arm_config = current_config[3:8]
    wheel_angles = current_config[8:12]

    # Apply speed limits: values less than max_speed become -max_speed, values larger than max_speed become max_speed
    limited_controls = np.clip(controls, -max_speed, max_speed)

    # Get speeds
    joint_speed = limited_controls[4:]
    wheel_speeds = limited_controls[:4]

    # Find new configs
    new_arm_config = arm_config + joint_speed * dt
    new_wheel_angles = wheel_angles + wheel_speeds * dt

    # Given dimensions of youBot
    l = 0.235
    w = 0.15
    r = 0.0475

    # Find chassis config, 13.4 odom...
    F = (r / 4) * np.array(
        [
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
        ]
    )
    V_b = F @ (wheel_speeds * dt).T

    rot_z, x_new, y_new = V_b

    if rot_z == 0:
        dconfig = np.array([0, x_new, y_new])
    else:
        dconfig = np.array(
            [
                rot_z,
                (x_new * np.sin(rot_z) + y_new * (np.cos(rot_z) - 1)) / rot_z,
                (y_new * np.sin(rot_z) + x_new * (1 - np.cos(rot_z))) / rot_z,
            ]
        )

    dir = chassis_config[0]

    Tsb = np.array(
        [[1, 0, 0], [0, np.cos(dir), -np.sin(dir)], [0, np.sin(dir), np.cos(dir)]]
    )

    dconfig = Tsb @ dconfig

    chassis_config = chassis_config + dconfig

    # Format new config
    new_config = np.concatenate((chassis_config, new_arm_config, new_wheel_angles))

    return new_config

# Set values...
dtheta = np.array([0, 0, 0, 0, 0])
u = np.array([-10, 10, 10, -10])
controls = np.concatenate((u, dtheta))

initial_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
current_config = initial_config

max_speed = 5

movement = []

for i in range(100):
    current_config = NextState(
        current_config=current_config, controls=controls, dt=0.01, max_speed=max_speed
    )
    movement.append(current_config)


np.savetxt(
    "/home/henry/Desktop/Classes/ME_449/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/milestone2.csv",
    np.asarray(movement),
    delimiter=",",
)
