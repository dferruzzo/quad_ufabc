class KF():
    def __init__(self, arg1):
        pass

    def LKF(self, Phi, Gamma, u, x, P, Q, H, R, y):
        # ---------------------------------------------------------------------
        # Attitude Linear Kalman Filter
        # It takes the following inputs:
        #   - Phi: The matrix of the discrete state transition model
        #   - Gamma: The matrix of the discrete input control model
        #   - u: The input control vector
        #   - x: The state vector, shape (6, 1), where: 
        #       - x[0:3]: The attitude quaternion
        #       - x[3:6]: The angular velocity
        #   - P: The state error covariance matrix, shape (6, 6)
        #   - Q: The process noise covariance matrix, shape (6, 6)
        #   - H: The matrix of the measurement model
        #   - R: The measurement noise covariance matrix
        #   - y: The measurement vector, shape (3, 1), where:
        #       - y[0:3]: The gyroscope measurement
        #
        # It returns the following outputs:
        #   - x: The updated state vector
        #   - P: The updated state error covariance matrix
        # ---------------------------------------------------------------------
        x = Phi@x + Gamma@u
        P = Phi@P@Phi.T + Q
        K = P@H.T@(H@P@H.T + R)**-1
        x = x + K@(y - H@x)
        P = (np.eye(6) - K@H)@P
        return x, P
