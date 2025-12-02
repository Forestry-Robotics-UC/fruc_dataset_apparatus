# How to Record with Steam Deck 

## ROS2

1. Power-on the Steam Deck
2. Click on the "STEAM" Physical button (in-front bottom left);
3. Click on "Power" and then "Switch to Desktop";

<img width="451" height="514" alt="steam_boot" src="https://github.com/user-attachments/assets/b18542b7-f332-47c1-a9a0-fc1b31b275cf" />

4. To **start** a Recording session click simultaneously on the in-back bottom left button with a long press (red circle with a 1) and in-back bottom right with a single click (green circle with a 2);
5. Follow the prompted texts;
6. To **stop** recording click simultaneously on the in-back bottom left button with a long press (red circle with a 1) and in-back top left with a single click (blue circle with a 3);

<img width="592" height="239" alt="steam_deck" src="https://github.com/user-attachments/assets/0ce90f07-6e70-433b-a408-51156d218d85" />

7. The ros2 bags will be under `~/fruc/fruc_dataset_apparatus/recordings/`

## ROS1

1. Power-on the Steam Deck
2. Click on the "STEAM" Physical button (in-front bottom left);
3. Click on "Power" and then "Switch to Desktop";

<img width="451" height="514" alt="steam_boot" src="https://github.com/user-attachments/assets/b18542b7-f332-47c1-a9a0-fc1b31b275cf" />

4. Open a new terminal window and change directories to the **~/fruc/fruc_dataset_apparatus/** folder:
  `cd ~/fruc/fruc_dataset_apparatus/`
5. Change the git branch for the ROS1 variant:
  `git branch ros1`
6. Change to the cd ~/fruc/fruc_dataset_apparatus/docker folder:
   `cd ~/fruc/fruc_dataset_apparatus/docker`
**Configure Topic Selection**: Create a `.env` file in the `docker/` directory of the repository. This file will specify the topic names you want to record. Open the `.env` file in a text editor and define the topics using the following format:
   ```bash
   USER=username

   TOPICS="
       /imu/data
       /imu/mag
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
   "
   ```

   In the above example, we have provided a list of example topics that you may want to record. You can customize this list based on your specific requirements.
8. Run `podman compose up` to start the recording session;

   The recording system is now up and running, capturing data from the selected topics and storing it in a rosbag file under `fruc_dataset_apparatus/docker`. You can customize the behavior and settings of the recording system by modifying the `docker-compose.yml` file or other relevant configuration files within the repository.

---

For more details, please check the Apparatus [Wiki](https://github.com/Forestry-Robotics-UC/fruc_dataset_apparatus/wiki)!
