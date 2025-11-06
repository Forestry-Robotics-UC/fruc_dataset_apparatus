# Getting Started

To begin recording data on any Linux-based computer, follow the steps below:

1. **Install Docker**: First, make sure you have Docker installed on your system. Docker is a platform that allows you to package and distribute applications in lightweight, isolated containers. You can download Docker from the official website [here](https://www.docker.com/) and follow the installation instructions specific to your operating system.

2. **Download the Repository**: Next, download the repository containing the necessary files for recording. You can do this by cloning the repository using Git or by downloading the repository as a ZIP file and extracting it to a local directory on your computer.

3. **Initialize Git Submodules**: Within the downloaded repository, there may be Git submodules that need to be initialized. Git submodules are separate Git repositories embedded within a parent repository. To initialize the submodules, navigate to the repository's root directory using a terminal or command prompt and execute the following command:
   ```bash
   git submodule init
   git submodule update
   ```

4. **Configure Topic Selection**: Create a `.env` file in the `docker/` directory of the repository. This file will specify the topic names you want to record. Open the `.env` file in a text editor and define the topics using the following format:
   ```bash
   USER=username

   TOPICS="
       /imu/data
       /imu/mag
       /imu/temperature
       /livox/lidar
       /mynteye/left_rect/image_rect/compressed
       /mynteye/left_rect/camera_info
       /mynteye/right_rect/image_rect/compressed
       /mynteye/right_rect/camera_info
       /mynteye/depth/image_raw
       /realsense/color/image_raw/compressed
       /realsense/color/camera_info
       /realsense/aligned_depth_to_color/image_raw/compressedDepth
       /realsense/aligned_depth_to_color/camera_info
       /realsense/accel/sample
       /realsense/gyro/sample
       /mimix3/gps/assisted
   "
   ```

   In the above example, we have provided a list of example topics that you may want to record. You can customize this list based on your specific requirements.

5. **Connect the Sensors**: Connect the required sensors to your computer, making sure they are properly powered. This may involve connecting devices such as the Mynt Eye s1030 camera, Xsens IMU, Intel Realsense D435i camera, Livox LiDAR, and any other sensors you wish to use.

6. **Launch the Recording System**: Open a terminal or command prompt and navigate to the `fruc_dataset_apparatus/docker` directory within the downloaded repository. Run the following command to start the recording system using Docker Compose:
   ```bash
   docker-compose up
   ```

   Docker Compose will read the configuration from the `docker-compose.yml` file and launch the required containers with the specified settings. You will see the output and logs from each container in the terminal window.

   The recording system is now up and running, capturing data from the selected topics and storing it in a rosbag file under `fruc_dataset_apparatus/docker`. You can customize the behavior and settings of the recording system by modifying the `docker-compose.yml` file or other relevant configuration files within the repository.


# How to Record with Steam Deck

1. Power-on the Steam Deck
2. Click on the "STEAM" Physical button (in-front bottom left);
3. Click on "Power" and then "Switch to Desktop";

<img width="451" height="514" alt="steam_boot" src="https://github.com/user-attachments/assets/b18542b7-f332-47c1-a9a0-fc1b31b275cf" />

4. To **start** a Recording session click simultaneously on the in-back bottom left button with a long press (red circle with a 1) and in-back bottom right with a single click (green circle with a 2);
5. Follow the prompted texts;
6. To **stop** recording click simultaneously on the in-back bottom left button with a long press (red circle with a 1) and in-back top left with a single click (blue circle with a 3);

<img width="592" height="239" alt="steam_deck" src="https://github.com/user-attachments/assets/0ce90f07-6e70-433b-a408-51156d218d85" />

7. The ros2 bags will be under `~/fruc/fruc_dataset_apparatus/recordings/`


--------------

For more details, please check the Apparatus [Wiki](https://github.com/Forestry-Robotics-UC/fruc_dataset_apparatus/wiki)!
