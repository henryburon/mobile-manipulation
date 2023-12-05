import modern_robotics as mr
import numpy as np

# Milestone 1: youBot Kinematics Simulator and csv output

################################################################################
# Initial configurations
################################################################################















################################################################################
# NextState Function
################################################################################



current_config = np.array([1,2,3,4,5,6,7,8,9,10,11,12])
controls = np.array([1,2,3,4,5,6,7,8,9])




def NextState(current_config, controls, dt, max_speed):

    # INPUTS
    # current_config: 12-vector (3 for chassis config, 5 for arm config, 4 for wheel angles)
    # controls: 9-vector (4 for wheel speeds, 5 for arm joint speeds)
    # dt: timestamp
    # max_speed: max angular speed of the arm joints and wheels

    # OUTPUT
    # new_config: 12-vector (new_chassis_config, new_arm_config, new_wheel_angles)


    # Extract individual components from the current_config
    chassis_config = current_config[:3]
    arm_config = current_config[3:8]
    wheel_angles = current_config[8:]

    # Apply speed limits: values less than max_speed become -max_speed, values larger than max_speed become max_speed
    limited_controls = np.clip(controls, -max_speed, max_speed)

    # Find new arm joint angles
    new_arm_config = arm_config + limited_controls[3:] * dt

    # Find new wheel angles
    new_wheel_angles = wheel_angles + limited_controls[:3] * dt

    # Find new chassis configuration
    # Means updating the robot's position and orientation based on the movement of its wheels
    # Estimating the chassis configuration from wheel motions
    # Essentially, integrating the effects of wheel velocities
    # 1. Measure the wheel displacements
    # 2. Assume constant wheel speeds
    # 3. Find the chassis planar twist Vb that corresponds to dtheta
    # 4. Use the matrix exponential to integrate the corresponding 6-dimensional twist for time dt = 1 to find the config of the chassis frame
    # at step k + 1 relative to the chassis frame at step k.
    # 5. Finally, express new chassis frame relative to space frame







    new_chassis_config = chassis_config

    # Put together the new configuration
    new_config = np.concatenate((new_chassis_config, new_arm_config, new_wheel_angles))

    return new_config



