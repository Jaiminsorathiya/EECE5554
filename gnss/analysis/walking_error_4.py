import matplotlib.pyplot as plt
import numpy as np
import rosbag
import utm

def process_bag_and_calculate_error(bag_file, gps_topic):
    try:
        eastings, northings = get_utm_coordinates_from_bag(bag_file, gps_topic)

        fit_coefficients, residuals = fit_line_and_get_residuals(northings, eastings)

        error = calculate_total_error(residuals, northings)

        print(f'Total error from the best-fit line: {error} meters')

    except FileNotFoundError:
        print(f'Error: File {bag_file} not found.')


def get_utm_coordinates_from_bag(bag_file, gps_topic):
    utm_eastings, utm_northings = [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[gps_topic]):
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            utm_eastings.append(e)
            utm_northings.append(n)
    return utm_eastings, utm_northings


def fit_line_and_get_residuals(northings, eastings):
    coefficients, residuals, _, _, _ = np.polyfit(northings, eastings, 1, full=True)
    return coefficients, residuals


def calculate_total_error(residuals, northings):
    total_error = np.sqrt(np.sum(residuals**2) / len(northings))
    return total_error


bag_file_path = '/home/jaiminsorathiya/EECE5554/gnss/data/walkingRTK.bag'
gps_topic_name = '/rtk'
process_bag_and_calculate_error(bag_file_path, gps_topic_name)

