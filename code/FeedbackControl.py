import modern_robotics as mr
import numpy as np

########################################################################################
# Milestone 3: Calculate the kinematic task-space feedforward plus feedback control law
########################################################################################

current_config = [0, 0, 0, 0, 0, 0.2, -1.6, 0]

Xd = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

Xd_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])

X = np.array(
    [[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]]
)

# Define PID gains, time step, initial configuration
Kp = np.zeros(6)
Ki = np.zeros(6)
dt = 0.01
config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
X_err_int = None

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, config):
    # Given dimensions of youBot chassis
    l = 0.235
    w = 0.15
    r = 0.0475

    # The fixed offset from chassis frame {b} to the base frame of the arm {0}
    T_b0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])

    # At home configuration, the end-effector frame {e} relative to the arm base frame {0}
    M_0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])

    # At home configuration, screw axis B for the 5 joints
    B_list = np.array(
        [
            [0, 0, 1, 0, 0.033, 0],
            [0, -1, 0, -0.5076, 0, 0],
            [0, -1, 0, -0.3526, 0, 0],
            [0, -1, 0, -0.2176, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ]
    ).T

    F = (r / 4) * np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
            [0, 0, 0, 0],
        ]
    )

    thlist = config[3:8]

    # Calculate forward kinematics and velocity jacobian
    T_0e = mr.FKinBody(M_0e, B_list, thlist)
    V_d = mr.se3ToVec((1 / dt) * mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next)))
    Adj = np.dot(mr.Adjoint(np.dot(mr.TransInv(X), Xd)), V_d)

    # Calculate error in the task space
    X_err = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ Xd))
    X_err_int = np.zeros((6,)) + 2 * (X_err * dt)

    # Calculate control commands for the robot arm
    V = Adj + np.dot(X_err, Kp) + np.dot(X_err_int, Ki)
    Jarm = mr.JacobianBody(B_list, config[3:8])
    T_0e = mr.FKinBody(M_0e, B_list, config[3:8])
    Tbe = T_b0 @ T_0e
    Jbase = (mr.Adjoint(mr.TransInv(Tbe))) @ F
    Jee = np.concatenate((Jbase, Jarm), axis=1)
    Jee_pseudoinv = np.linalg.pinv(Jee)
    controls = np.round(np.dot(Jee_pseudoinv, V), decimals=2)
    Ve = np.hstack((V, controls, X_err))

    return V_d, V, X_err, controls, Ve, X_err_int

# call FeedbackControl function
V_d, V, X_err, controls, Ve, X_err_int = FeedbackControl(
    X, Xd, Xd_next, Kp, Ki, dt, config
)