import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import sys
import loca_lidar.lidar_data_pb2 as lidar_data
import time


class LidarCloudDisplay(): 
    """ Manages subscription tu an eCAL Topic of type "Lidar" and display the cloud point associated
    """
    def __init__(self, fig: plt.figure, topic_name: str, color: str, y_button = 0.1):
        """_summary_

        Args:
            fig (plt.figure): _description_
            topic_name (str): name of eCAL topic, type Lidar. (example : lidar_data)
            color (str): _description_ (example : 'y')
            y_button (float, optional): _description_. Defaults to 0.1.
        """
        self.topic_name = topic_name
        self.color = color

        # Drawing cloudpoint
        self.button_ax = fig.add_axes([0.005, y_button, 0.1, 0.1])
        self.button_ax.get_xaxis().set_visible(False)
        self.button_ax.get_yaxis().set_visible(False)

        # Drawing Button
        self.bdisplay = Button(self.button_ax, topic_name, color=color)
        self.bdisplay.label.set_fontsize(8)
        self.bdisplay.on_clicked(self.on_button_click)

        # Datas to display
        self.lidar_dist = np.array([0])
        self.lidar_theta = np.array([0])
        self.is_displaying = True

        # init ecal subscription
        self.sub_lidar = ProtoSubscriber(topic_name,lidar_data.Lidar)
        self.sub_lidar.set_callback(self.on_lidar_scan)

    def on_lidar_scan(self, topic_name, msg, time):
        self.lidar_dist = np.array(msg.distances)
        self.lidar_theta = np.array(msg.angles)

    def on_button_click(self, event): 
        self.is_displaying = not self.is_displaying


plt.style.use('ggplot')
fig = plt.figure()
ax = fig.add_subplot(projection='polar')

# on_lidar_scan


# position_filtered_scan

# Amalgames center

# calculated beacons positions

# real_position

if __name__ == "__main__":
    print("CHANGE Y LIMIT TO 3.1 meters for display !")
    print("angle display set to clockwise to keep coherencewith current lidar data")
    # Init ecal Communication
    ecal_core.initialize(sys.argv, "ecal_lidar_vizualisation")

    #Tuple of cloud points to display
    cloud_pts = (
        LidarCloudDisplay(fig, 'lidar_data', 'y', 0.1),
        LidarCloudDisplay(fig, 'lidar_filtered', 'g', 0.3),
        LidarCloudDisplay(fig, 'amalgames', 'b', 0.5),
    )
    # Init matplotlib plot
    plt.show(block=False)
    while ecal_core.ok():
        ax.cla() # clear last cloud points
        ax.set_theta_zero_location("N") # North
        # https://stackoverflow.com/questions/26906510/rotate-theta-0-on-matplotlib-polar-plot
        ax.set_ylim([0, 3.1]) # maximum is 3 m after basic filter, so no need to see further

        for cloud in cloud_pts: # Plot each cloud point from various data stream
            if cloud.is_displaying: #button clicked or not
                ax.scatter(
                    np.deg2rad(cloud.lidar_theta), 
                    cloud.lidar_dist, 
                    color=cloud.color
                )

        plt.pause(0.1)

    ecal_core.finalize()

