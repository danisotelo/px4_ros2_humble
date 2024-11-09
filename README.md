# Docker VNC ROS2 Humble + PX4 + Gazebo Harmonic

![vnc_px4_ros2_dev](https://github.com/user-attachments/assets/ce466767-dc35-4628-8fee-2093d2b98597)

https://github.com/user-attachments/assets/8e8c75f2-6713-42a0-82e4-c754222bef5c

## Overview

This Dockerfile provides a HTML5 VNC interface to access Ubuntu Desktop 22.04 with ROS 2 Humble, PX4 and Gazebo Harmonic. It includes additional tools such as Mozilla Firefox Navigator, Terminator and VSCodium for easy development. This project can be used as a template for different environment configurations through easy modifications. This work was greatly inspired by the work done by [Tiryoh](https://github.com/Tiryoh/docker-ros2-desktop-vnc/tree/master).

The implemented project consists of a UR5e robotic arm visually tracking a Holybro x500 drone equipped with an ArUco tag. Since there is no official support for robot control with ROS 2 Humble + Gazebo Harmonic, this project has additionally developed a custom ROS package to make them compatible (see `src/gz_ros2_control`). This project also provides a ROS 2 package with the description files of the UR5e arm equipped with the Robotiq-85 gripper and one Intel RealSense D435 camera attached to the arm through a custom-designed support assembly.

## Get Started

### Install Docker
You can install Docker using the `apt` repository or install it from a package. Here, the `apt` repository installation method is indicated. In first place, you need to set up the Docker `apt` repository. The first step is to add the Docker's official GPG key:
```
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```
Next, add the repository to `apt` sources:
```
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
Afterwards, to install the Docker packages you need to run:
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin
```
You can run `hello-world` image to verify the installation:
```
sudo docker run hello-world
```
If you see `Hello from Docker!` message, it's a sign that you have succesfully installed and started Docker Engine. As you noticed, you have to run Docker with `sudo` and you will get permissions denied if you try to run `docker run` without it. If you don't want to use `sudo` create a Unix group called `docker` and add users to it:
```
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
docker run hello-world
```

### Install Docker Compose
Docker Compose is a versatile tool that enables to define and handle multi-container applications in an easy way. This project uses Docker Compose commands for the building and running of the Docker image and container. To install it simply run:
```
sudo apt install docker-compose
```

### Clone the repository
After installing Docker, you need to clone this repository using:
```
cd ~/
git clone https://github.com/danisotelo/px4_ros2_humble.git
```

### Build Docker Image and Run
Now, go into the `.env` file and change the `HOST_USER` environment variable to your actual host user. Building the Docker image and running the container from the Dockerfile using Docker-Compose is really simple, you just need to run:
```
cd ~/px4_ros2_humble
docker-compose build
docker-compose up
```

It will take a while for the image to be built due to the large amount of files to be downloaded and installed (around 20 minutes). Once finished, you should see a message displaying a local host path. You can click on it or you can go to your navigator and type:
```
http://localhost:6080/
```

Click on `Connect` and now you have access to the container GUI through the HTML5 VNC interface.

### Install PX4-Autopilot
Since the PX4-Autopilot is a heavy repository, it was decided not to include it directly inside the container but in a shared volume with the host. This way, it only has to be downloaded once and changes will be saved even after deleting the container. Run the following commands from a terminal inside the container to install it:
```
cd ~/shared_volume
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
Next, enter the `PX4-Autopilot` directory, clean and build the source code:
```
cd PX4-Autopilot
make clean
make distclean
make submodulesclean
```
You can test if PX4 can run with the Gazebo Harmonic simulation with:
```
make px4_sitl gz_x500
```
The XRCE-DDS Agent is already built with the Docker image. In a new terminal, run the XRCE-DDS Agent. You should see them connect to each other.
```
MicroXRCEAgent udp4 -p 8888
```
For loading the PX4 files (airframes, models, and worlds) for the UR5e drone-tracking project, you need to additionally run a bash script, `file_transfer` that will be in charge of modifying the original PX4-Autopilot repository accordingly. If you want to use this simulation run in a container terminal:
```
cd ~/px4_files
bash file_transfer.sh
cd ~/shared_volume/PX4-Autopilot
make clean
make distclean
make submodulesclean
make px4_sitl gz_x500_tag
```

### Build and Run ROS 2 Workspace
The ROS 2 workspace is named `catkin_ws` and belongs to the shared volume. This way, you should first manually move the folder `src` from the repository to the shared volume that was created after running the container, `px4_ros2_humble_shared_volume/catkin_ws`.

Then, you can access these files from the container, and changes will be saved even after deleting the container. The next step is to build the ROS 2 workspace and launch the simulation using the following commands from the `catkin_ws` folder:
```
cd ~/shared_volume/catkin_ws
colcon build --packages-select drone_following gz_ros2_control px4_msgs ur5e_gripper_description
ros2 launch drone_following simulation.launch.py
```
While working with the container if ever the user session is closed, the user and password are both `ubuntu`.

### Stop the Container
To stop the container from running simply open a new terminal in your host and run:
```
docker-compose down
```
In case you want to delete the images and volumes created by the container to rebuild it run:
```
docker-compose down --volumes
docker system prune -a --volumes
```

## Bugs & Feature Requests
Feedback to improve the project is welcomed. Please report bugs and request features using the [Issue Tracker](https://github.com/danisotelo/px4_ros2_humble/issues).
