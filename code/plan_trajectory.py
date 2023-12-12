import modern_robotics as mr
from NextState import NextState
from TrajectoryGenerator import TrajectoryGenerator
from FeedbackControl import FeedbackControl
import numpy as np
import matplotlib.pyplot as plt
import logging


file = "overshoot_run.log"
logging.basicConfig(filename=file, level=logging.DEBUG)

Kp_gain = 1
Ki_gain = 6

logging.debug("Starting program.")
T_sc_i = np.array([[1, 0, 0, 0.5], [0, 1, 0, -0.5], [0, 0, 1, 0.025], [0, 0, 0, 1]])

T_sc_f = np.array([[0, 1, 0, 2], [-1, 0, 0, -2], [0, 0, 1, 0.025], [0, 0, 0, 1]])

# Initialize PI gain matrices
Kp = np.eye(6) * Kp_gain
Ki = np.eye(6) * Ki_gain

# Define starting config of robot
starting_config = np.array([0, -0.25, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])
config = starting_config

# The fixed offset from the chassis frame {b} to the base frame of the arm {0}
T_b0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])

# end-effector in reference trajectory
T_se_i = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

# At the home configuration...
M_0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])

B_list = np.array(
    [
        [0, 0, 1, 0, 0.033, 0],
        [0, -1, 0, -0.5076, 0, 0],
        [0, -1, 0, -0.3526, 0, 0],
        [0, -1, 0, -0.2176, 0, 0],
        [0, 0, 1, 0, 0, 0],
    ]
).T

final_path = []
X_err_list = []

final_path.append(config)
max_speed = 3.141592**2
dt = 0.01


logging.debug("Generating trajectory (from milestone2)")
traj = TrajectoryGenerator(T_se_i, T_sc_i, T_sc_f, scaling=4.594)

logging.debug("Generating configs with FeedbackControl and NextState")
for i in range(5999):
    # extract configuration components for the robot
    th_list = config[3:8]
    th = config[0]
    x = config[1]
    y = config[2]
    z = 0.0963

    # calculate robot transformations
    T_sb = np.array(
        [
            [np.cos(th), -np.sin(th), 0, x],
            [np.sin(th), np.cos(th), 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ]
    )

    T_0e = mr.FKinBody(M_0e, B_list, th_list)

    T_be = np.dot(T_b0, T_0e)
    X = np.dot(T_sb, T_be)

    # define desired and next desired configs
    current_traj = traj[i]
    new_traj = traj[i + 1]

    X_d = np.array(
        [
            current_traj[:3] + current_traj[9:10],
            current_traj[3:6] + current_traj[10:11],
            current_traj[6:9] + current_traj[11:12],
            [0, 0, 0, 1],
        ]
    )

    X_dnext = np.array(
        [
            new_traj[:3] + new_traj[9:10],
            new_traj[3:6] + new_traj[10:11],
            new_traj[6:9] + new_traj[11:12],
            [0, 0, 0, 1],
        ]
    )

    # Calculate the control speeds and error value from Milestone3
    V_d, V, X_err, controls, Ve, X_err_int = FeedbackControl(
        X, X_d, X_dnext, Kp, Ki, dt, config
    )
    X_err_list.append(X_err)

    w_spds = controls[:4]
    a_spds = controls[4:9]
    final_controls = np.concatenate((a_spds, w_spds), axis=None)

    current_config = config[:12]

    # Create new config for next loop...

    config = NextState(current_config, final_controls, dt, max_speed)

    traj_new = traj[i][12]
    robo_current_traj = np.concatenate((config[:12], traj_new), axis=None)
    final_path.append(robo_current_traj)

logging.debug("Done generating configs.")

# np.savetxt("/home/henry/Desktop/Classes/ME_449/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/milestone6.csv", np.asarray(final_path), delimiter = ",",)
# np.savetxt("/home/henry/Desktop/milestone6.csv", np.asarray(final_path), delimiter = ",",)

# np.savetxt("/home/henry/Desktop/XX.csv", np.asarray(X_err_list), delimiter = ",",)

logging.debug("Creating plot...")

# Plotting the error...
x_axis = np.linspace(0, 14, 5999)
y_axis = np.asarray(X_err_list)

plt.plot(x_axis, y_axis[:, 0])
plt.plot(x_axis, y_axis[:, 1])
plt.plot(x_axis, y_axis[:, 2])
plt.plot(x_axis, y_axis[:, 3])
plt.plot(x_axis, y_axis[:, 4])
plt.plot(x_axis, y_axis[:, 5])

plt.title("X_err vs Time")
plt.xlabel("Time (s)")
plt.ylabel("X_err")
plt.legend(
    [
        r"$X_{err1}$",
        r"$X_{err2}$",
        r"$X_{err3}$",
        r"$X_{err4}$",
        r"$X_{err5}$",
        r"$X_{err6}$",
    ]
)
plt.show()
logging.debug("Done with program.")
