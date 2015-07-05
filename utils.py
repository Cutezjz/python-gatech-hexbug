from math import sqrt


def compute_error(prediction, actual):
    """CLW 20150407:  Compute error between data points,
    following example from Final Project Specs
    """

    sum_diffs_squared = 0

    prediction_points = len(prediction)
    actual_points = len(actual)

    num_data_points = prediction_points

    # check if both arrays are the same size.
    # set number of points compared to the minimum of the two sets.
    if prediction_points != actual_points:
        print "WARNING! prediction_points != actual_points"

        if actual_points < prediction_points:
            num_data_points = actual_points

    for i in range(num_data_points):
        x_diff_squared = ( prediction[i][0] - actual[i][0] ) ** 2
        y_diff_squared = ( prediction[i][1] - actual[i][1] ) ** 2

        sum_diffs_squared = sum_diffs_squared + x_diff_squared + y_diff_squared

    distance_error = sqrt(sum_diffs_squared)

    return distance_error


def convert_data_to_matrix(filename):
    """This method will take a robot data file and
    return a matrix containing all of the INTEGER data points
    """
    return eval(open(filename).read())


def save_predictions(results):
    """Saves the results to prediction.txt file."""

    with open('prediction.txt', 'w') as fout:
        fout.write('[')
        fout.write(',\n'.join(str(item) for item in results))
        fout.write(']\n')




