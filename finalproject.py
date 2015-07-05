from math import *
from matrix import matrix
from utils import compute_error, convert_data_to_matrix, save_predictions

import argparse

# ------------- BEGIN HELPER FUNCTIONS ---------------

# helper function to map all angles onto [-pi, pi]
def angle_truncate(a):
        while a < 0.0:
            a += pi * 2
        return ((a + pi) % (pi * 2)) - pi

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Helper function to get the boundaries of the wall from data set, assuming perfectly perpendicular walls
# This also assumes the robot explores all possible minimum and maximum coordinates in the X,Y space
# Input: array of measurements [[X,Y], [X2,Y2]...] / Output: min X, max X, min Y, max Y
def get_wall_boundaries(measurements):
    X = []
    Y = []

    for i in range(len(measurements)):
        X.append(measurements[i][0])
        Y.append(measurements[i][1])

    return min(X), max(X), min(Y), max(Y)

# ------------- END HELPER FUNCTIONS ---------------

def estimate_next_pos(target_measurement, OTHER = None):

    # Assumed measurement noise for the data
    measurement_noise = 0.05

    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    I = matrix([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]]) #identity matrix for Kalman Filter

    R = matrix([[measurement_noise, 0], [0, measurement_noise]])

    # OTHER is a data storage variable. It is initialized as "None" but accumulates data over the first
    # two iterations of the algorithm. These if/else statements prevent us from trying to make predictions
    # until OTHER has enough data to intelligently execute EKF
    if OTHER is not None:
        current_measurement = target_measurement
        last_measurement = OTHER['last_measurement']

        heading = atan2(target_measurement[1] - last_measurement[1], target_measurement[0] - last_measurement[0])

        # Load the current X (state) and P (covariance) matrices for Kalman Filter
        X = OTHER['X']
        P = OTHER['P']

        if 'last_heading' not in OTHER:
            OTHER['last_heading'] = heading
            xy_estimate = [X.value[1][0], X.value[2][0]]
            OTHER['last_measurement'] = target_measurement
            OTHER['last_d'] = 0 #The robot hasn't moved, so D = 0 for now
            OTHER['iterator_counter'] = 1 #Initialize iterator with count of 1
        else:
            OTHER['iterator_counter'] += 1
            turning_angle = heading - OTHER['last_heading']
            OTHER['last_measurement'] = target_measurement
            OTHER['last_heading'] = heading

            # This code block will calculate cumulative moving average for distance over lifetime of the HexBug
            # This allows us to look at bug velocity holistically, rather than narrowly using velocity between
            # last two points
            D = distance_between(target_measurement, last_measurement)
            OTHER['last_d'] = (OTHER['last_d']*OTHER['iterator_counter'] + D)/(OTHER['iterator_counter'] + 1)

            # --------------------- PREDICTION STEP ---------------------

            # We assume the robot is attracted to linear motion and express this attraction with a "centering affinity"
            # variable which reduces the turning angle by 7% each time.
            # Empirically we found this improved our L-squared by 10-20%
            # Thus, our model assumes the next turning angle is 93% of the last turning angle.
            centering_affinity = 0.93
            turning_angle = turning_angle*centering_affinity

            # Store the turning angle in memory and then calculate the next robot heading, theta
            OTHER['last_turning_angle'] = turning_angle
            theta = (heading+turning_angle)%(2*pi)

            delta_x = D * cos(theta)
            delta_y = D * sin(theta)

            nextX = target_measurement[0] + delta_x
            nextY = target_measurement[1] + delta_y

            # Predicted state matrix X, prior to linearization calculations in EKF
            X = matrix([[theta],
                         [nextX],
                         [nextY]])

            # This is the Jacobian of the State Transition Model (essential for EKF)
            F = matrix([[1, 0, 0],
                        [-D*sin(theta), 1, 0],
                        [D*cos(theta), 0, 1]])

            P = OTHER['P']

            # H = Jacobian of the Measurement Function (essential for EKF)
            H = matrix([[0, 1, 0],
                        [0, 0, 1]])

            # Predicted covariance estimate
            # Process noise term Q is ignored here
            P = F * P * F.transpose() # + Q

            # --------------------- END PREDICTION STEP ---------------------

            # --------------------- MEASUREMENT UPDATE ---------------------

            observations = matrix([[target_measurement[0]],
                                    [target_measurement[1]]])
            Z = H*X
            Y = observations - Z    #Measurement residual
            S = H * P * H.transpose() + R # Residual covariance
            K = P * H.transpose() * S.inverse() # Near-optimal Kalman Gain
            X = X + (K*Y)   #Updated state estimate

            P = (I - (K * H)) * P # Updated covariance estimate

            # --------------------- END MEASUREMENT UPDATE ---------------------

            # Normalize the angle
            X.value[0][0] = angle_truncate(X.value[0][0])

            OTHER['X'] = X
            OTHER['P'] = P

            x_estimate = OTHER['X'].value[1][0]
            y_estimate = OTHER['X'].value[2][0]
            xy_estimate = [x_estimate, y_estimate]

    else:
        # Create an initial guess for X
        # Robot heading assumed 0, initial X,Y guess set to first measurement
        X = matrix([[0],
                    [target_measurement[0]],
                    [target_measurement[1]]])

        # convariance matrix initialized with high uncertainty in theta
        # Lower uncertainty in X, Y since we generally trust Open CV and data set
        P = matrix([[1000, 0, 0],
                    [0, 200, 0],
                    [0, 0, 200]])

        OTHER = {'last_measurement': target_measurement, 'X': X, 'P': P}
        xy_estimate = [X.value[1][0], X.value[2][0]]

    return xy_estimate, OTHER

def forward_extrapolate_move(target_measurement, theta, OTHER):
    D = OTHER['last_d'] #naively assume travel distance is the same per frame, as the last measurement

    # Get our minimum and maximum X, Y values for rectangular box.
    min_x = OTHER['boundaries'][0]
    max_x = OTHER['boundaries'][1]
    min_y = OTHER['boundaries'][2]
    max_y = OTHER['boundaries'][3]

    delta_x = D * cos(theta)
    delta_y = D * sin(theta)

    nextX = target_measurement[0] + delta_x
    nextY = target_measurement[1] + delta_y

    # ------------- BILLIARD BALL COLLISION MODEL -------------
    if nextX <= min_x:  # collision with left wall
        nextX = min_x
        theta = pi - theta

    if nextX >= max_x:  # collision with right wall
        nextX = max_x
        theta = pi - theta

    if nextY <= min_y:  # collision with bottom wall
        nextY = min_y
        theta =  2*pi - theta

    if nextY >= max_y:  # collision with top wall
        nextY = max_y
        theta =  2*pi - theta

    # ------------- END BILLIARD BALL COLLISION MODEL -------------

    # Empirical centering affinity factor used to reduce turning angle over time
    centering_affinity = 0.93

    OTHER['last_turning_angle'] = OTHER['last_turning_angle']*centering_affinity

    X = matrix([[theta + OTHER['last_turning_angle']],
                [nextX],
                [nextY]])

    return X, OTHER

def robot_EKF(measurements):
    OTHER = None
    predictions = []

    for measurement in measurements:
        position_guess, OTHER = estimate_next_pos(measurement, OTHER)
        predictions.append(position_guess)

    return predictions, OTHER

def predict_next_frames(OTHER, last_measurement, boundaries, frame_count):

    results = []
    OTHER['boundaries'] = boundaries
    X = matrix([[OTHER['last_heading']],
                [last_measurement[0]],
                [last_measurement[1]]])

    for i in range(frame_count):
        X, OTHER = forward_extrapolate_move([X.value[1][0], X.value[2][0]], X.value[0][0], OTHER)
        results.append([int(round(X.value[1][0])), int(round(X.value[2][0]))])

    return results


if __name__ == "__main__":

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--text', '-t', help='text file')
    parser.add_argument('--debug', '-d', help='debug file', action='store_true')
    args = parser.parse_args()

    filename = args.text
    if args.debug:
        filename = filename or 'hexbug-training_video-centroid_data'

    input_data = convert_data_to_matrix(filename)
    boundaries = get_wall_boundaries(input_data)

    # Debugging vs Final code
    if args.debug:
        #3000 will lead to a a corner collision
        #5000 will lead to an almost perpendicular collision with a flat wall, then steering to the robot's left
        #8000 will lead to an almost perfect 45 degree ricochet off the bottom wall
        #11000 will lead to a bunching up in the bottom left corner

        #start_index = 11000 #this will be the frame we start at for our test
        start_index = 2000 # 10000 #this will be the frame we start at for our test  # CLW

        len_data = 3600

        test_data = input_data[start_index:start_index+len_data]
        actual = input_data[start_index+len_data:start_index+len_data+60] #take the next 60 frames
        past_predictions, OTHER = robot_EKF(test_data)

        predicted = predict_next_frames(OTHER, test_data[-1], boundaries, 60)
        print predicted

    else:
        test_data = input_data[:]
        past_predictions, OTHER = robot_EKF(test_data)
        predicted = predict_next_frames(OTHER, test_data[-1], boundaries, 60)
        save_predictions(predicted)


