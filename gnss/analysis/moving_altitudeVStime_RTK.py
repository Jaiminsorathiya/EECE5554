import rosbag
import matplotlib.pyplot as plt

def extract_altitude_over_time(bag_path, topic_name):
    times, altitudes = [], []
    start_time = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            # Convert ROS Time to seconds since start
            if start_time is None:
                start_time = t.to_sec()
            times.append(t.to_sec() - start_time)
            altitudes.append(msg.altitude)

    return times, altitudes

def plot_altitude_over_time(times, altitudes):
    plt.figure(figsize=(10, 6))
    plt.plot(times, altitudes, label='Altitude over Time', marker='.', linestyle='-', color='b')
    plt.title('Moving Data: Altitude vs. Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
    # Specify the path to your ROS bag and the topic
    bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/walkingRTK.bag'
    topic_name = '/rtk_gps'  # Adjust based on your topic name for altitude data

    # Extract altitude data over time
    times, altitudes = extract_altitude_over_time(bag_path, topic_name)

    # Plot altitude over time
    plot_altitude_over_time(times, altitudes)

if __name__ == "__main__":
    main()

