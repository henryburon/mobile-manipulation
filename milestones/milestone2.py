import modern_robotics as mr
import numpy as np

################################################################################
# Use the code in the triple quotes to run the function
################################################################################

"""
T_sb = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]
                ])

# The fixed offset from chassis frame {b} to the base frame of the arm {0}
T_b0 = np.array([
                [1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]
                ])

# At home configuration, the end-effector frame {e} relative to the arm base frame {0}
M_0e = np.array([
                [1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]
                ])

# The cube's initial configuration
T_sc_i = np.array([
                [1, 0, 0, 1],
                [0, 1, 0, 0],
                [0, 0, 1, 0.025],
                [0, 0, 0, 1]
                ])

# The cube's desired final configuration
T_sc_f = np.array([
                [0, 1, 0, 0],
                [-1, 0, 0, -1],
                [0, 0, 1, 0.025],
                [0, 0, 0, 1]
                ])

T_se_i = T_sb @ T_b0 @ M_0e

test = TrajectoryGenerator(T_se_i, T_sc_i, T_sc_f, scaling=1)
"""

################################################################################
# Milestone 2: Generate the reference trajectory for the end-effector frame {e}
################################################################################

# Given configurations...

T_sb = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]
                ])

# The fixed offset from chassis frame {b} to the base frame of the arm {0}
T_b0 = np.array([
                [1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]
                ])

# At home configuration, the end-effector frame {e} relative to the arm base frame {0}
M_0e = np.array([
                [1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]
                ])

# The cube's initial configuration
T_sc_i = np.array([
                [1, 0, 0, 1],
                [0, 1, 0, 0],
                [0, 0, 1, 0.025],
                [0, 0, 0, 1]
                ])

# The cube's desired final configuration
T_sc_f = np.array([
                [0, 1, 0, 0],
                [-1, 0, 0, -1],
                [0, 0, 1, 0.025],
                [0, 0, 0, 1]
                ])

################################################################################
# Additional inputs
################################################################################

# Initial configuration of the end-effector in the reference trajectory
T_se_i = T_sb @ T_b0 @ M_0e

# The end-effector's standoff configuration above the cube, before and after grasping
T_ce_s = np.array([
                [0, 0, -1, 0],
                [0, 1, 0, 0],
                [1, 0, 0, 0.5],
                [0, 0, 0, 1]
                ])

# End effector final position
T_se_f = np.array([
                [0, 0, -1, 0],
                [0, 1, 0, -1],
                [1, 0, 0, 0.025],
                [0, 0, 0, 1]
                ])

################################################################################
# Define function
################################################################################

# Smaller scaling = shorter time, larger scaling = longer time

def TrajectoryGenerator(T_se_i, T_sc_i, T_sc_f, scaling):

    def format_for_csv(T, grasp):
        r11 = T[0,0]
        r12 = T[0,1]
        r13 = T[0,2]
        r21 = T[1,0]
        r22 = T[1,1]
        r23 = T[1,2]
        r31 = T[2,0]
        r32 = T[2,1]
        r33 = T[2,2]
        px = T[0,3]
        py = T[1,3]
        pz = T[2,3]
        csv_trajectory = [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, grasp]

        return csv_trajectory
    
    ##### Create first standoff

    angle = np.radians(135)  # Define a 135 degrees rotation matrix
    R_y = np.array([
                    [np.cos(angle), 0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle), 0, np.cos(angle), 0],
                    [0, 0, 0, 1]
                    ])

    T_s1 = T_sc_i.copy()
    T_s1[2, 3] += 0.3
    T_s1 = T_s1 @ R_y

    ##### Create first grasp

    T_g1 = T_s1.copy()
    T_g1[2, 3] -= 0.315

    ##### Create second standoff

    T_s2 = T_sc_f.copy()
    T_s2[2, 3] += 0.3
    T_s2 = T_s2 @ R_y
    
    trajectory = []

    ##### Create second grasp

    T_g2 = T_s2.copy()
    T_g2[2, 3] -= 0.315

    ########## End-Effector to initial standoff

    # Calculate the screw trajectory
    initial2standoff1 = mr.ScrewTrajectory(T_se_i, T_s1, 2, 300 * scaling, 3)

    # Format into values for csv
    for config in initial2standoff1:
        reformatted_value = format_for_csv(config, 0)
        trajectory.append(reformatted_value)

    ########## End-Effector to grasp position

    standoff12grasp1 = mr.ScrewTrajectory(T_s1, T_g1, 2, 150 * scaling, 3)

    for config in standoff12grasp1:
        reformatted_value = format_for_csv(config, 0)
        trajectory.append(reformatted_value)

    ########## Grasp

    # Create a csv entry with the correct values
    grasp = format_for_csv(T_g1, grasp=1)

    # Create enough of those values to complete the grasp action, which takes 0.625 seconds
    for i in range(100):
        trajectory.append(grasp)


    ########## End-Effector back to standoff1

    grasp12standoff1 = mr.ScrewTrajectory(T_g1, T_s1, 2, 150 * scaling, 3)

    for config in grasp12standoff1:
        reformatted_value = format_for_csv(config, 1)
        trajectory.append(reformatted_value)

    ########## End-Effector to standoff2

    standoff12standoff2 = mr.ScrewTrajectory(T_s1, T_s2, 2, 450 * scaling, 3)

    for config in standoff12standoff2:
        reformatted_value = format_for_csv(config, 1)
        trajectory.append(reformatted_value)

    ########## End-Effector to grasp2

    standoff22grasp2 = mr.ScrewTrajectory(T_s2, T_g2, 2, 150 * scaling, 3)

    for config in standoff22grasp2:
        reformatted_value = format_for_csv(config, 1)
        trajectory.append(reformatted_value)

    ########## Release grasp

    release_grasp = format_for_csv(T_g2, grasp=0)

    for i in range(100):
        trajectory.append(release_grasp)

    ########## Back to standoff2

    grasp22standoff2 = mr.ScrewTrajectory(T_g2, T_s2, 2, 150 * scaling, 3)

    for config in grasp22standoff2:
        reformatted_value = format_for_csv(config, 0)
        trajectory.append(reformatted_value)

    np.savetxt("/home/henry/Desktop/Classes/ME_449/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/milestone2.csv", np.asarray(trajectory), delimiter = ",")

    print("Generated and saved the end effector's trajectory!")

    
    return trajectory


test = TrajectoryGenerator(T_se_i, T_sc_i, T_sc_f, scaling=2)
print(test[9])
print(len(test[9]))