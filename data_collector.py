#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pandas as pd

class DataCollector:
    def __init__(self):
        self.data = []
        # Initialize the ROS node
        rospy.init_node('data_collector', anonymous=True)
        # Subscribe to the relevant topic
        self.subscriber = rospy.Subscriber('/ti_mmwave/radar_scan_pcl_0', PointCloud2, self.callback)
        rospy.loginfo("DataCollector node initialized and subscribed to /ti_mmwave/radar_scan_pcl_0")
        print("DataCollector node initialized and subscribed to /ti_mmwave/radar_scan_pcl_0")
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("Received a message!")
        print("Received a message!")
        # Extract data from the message
        timestamp = msg.header.stamp.to_sec()
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "velocity"), skip_nans=True))
        # Save the data
        for point in points:
            self.data.append({
                'timestamp': timestamp,
                'x': point[0],
                'y': point[1],
                'z': point[2],
                'intensity': point[3],
                'velocity': point[4]
            })
        # Print or log info to indicate data reception
        rospy.loginfo(f"Collected {len(points)} points at timestamp {timestamp}")
        print(f"Collected {len(points)} points at timestamp {timestamp}")

    def save_data(self, file_path):
        if self.data:
            # Convert the collected data to a pandas DataFrame and save as CSV
            df = pd.DataFrame(self.data)
            df.to_csv(file_path, index=False)
            rospy.loginfo(f"Data saved to {file_path}")
            print(f"Data saved to {file_path}")
        else:
            rospy.loginfo("No data to save.")
            print("No data to save.")

if __name__ == '__main__':
    try:
        collector = DataCollector()
        rospy.sleep(5)
        collector.save_data('data.csv')
    except rospy.ROSInterruptException:
        rospy.loginfo("DataCollector node interrupted.")
        print("DataCollector node interrupted.")

