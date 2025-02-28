# mmWave-Radar

## Data Collection Process

### Steps to set up data collection environment. 

Tested on Ubuntu 20.04.6 LTS on Nano Jetson and AWR1843BOOST mmWave radar board

1. Create conda environment `conda create -n <env_name> python=3.9`

2. Install ROS1 noetic from https://foxglove.dev/blog/installing-ros1-noetic-on-ubuntu

3. Clone the following repo and ROS serial to your `<workspace>/src:`
   ```   
   git clone https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git
   git clone https://github.com/wjwwood/serial.git
   ```
5. The folder `mmwave_ti_ros` from the cloned repo, will have three subfolders; create an empty file with the name `CATKIN_IGNORE` with no extension and place it in the `ros2_driver` and `autonomous_robotics_ros` subfolders. Also, place this in the `ros1_driver/src/serial` folder.

6. To resolve a potential issue with Rviz, go to `ros1_driver/src/ti_mmwave_rospkg/launch` and locate the launch file `1843_multi_3d_0.launch`. In the launch file change this line `<param name="frame_id" value="/ti_mmwave_0"/>` to `<param name="frame_id" value="ti_mmwave_0"/>`, basically remove the / symbol.

7. Then, from `<workspace>/src` where the repo was cloned, go back to `<workspace>` and run the following commands:
   ```
   source /opt/ros/noetic/setup.bash
   catkin_make && source devel/setup.bash
   echo "source <workspace_dir>/devel/setup.bash" >> ~/.bashrc
   ```

8. To enable data ports, run the following commands:
   ```
   sudo chmod 666 /dev/ttyACM0
   sudo chmod 666 /dev/ttyACM1
   ```

10. Then, go to the launch folder in ros1_driver and run the following command to collect data:
    ```
    roslaunch ti_mmwave_rospkg 1843_multi_3d_0.launch
    ```

11. Then open a new terminal and run the following command to subscribe to the ros topic
    ```
    python3 data_collector.py
    ```

12. Remember to toggle SOP2,1,0 to “001" on the mmwave radar board to get the board into functional mode to produce pcl data

### Debugging

1. If you encounter the error `AttributeError: module 'em' has no attribute 'RAW_OPT’`:
   ```
   Uninstall empy: `pip uninstall empy`
   And install version 3.3.4: `pip install empy==3.3.4`
   ```

2. Check the following repo for debugging other errors `https://github.com/radar-lab/ti_mmwave_rospkg/tree/master`


### Subsequent data collection
Once the environment is set, subsequently, repeat steps 6 to 10 to collect data with the mmWave baord
