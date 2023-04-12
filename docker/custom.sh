export DOCKER_ROS_FILES_PATH=/docker-ros/files
# Install ROS version dependent apt packages
if [ "$ROS_VERSION" = "2" ]; then
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz2"
else
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz
                         ros-$ROS_DISTRO-opencv-apps
                         ros-$ROS_DISTRO-rospy
                         ros-$ROS_DISTRO-rospy-message-converter
                         ros-$ROS_DISTRO-pcl-ros
                         ros-$ROS_DISTRO-derived-object-msgs
                         python3-catkin-tools
                         python3-catkin-pkg
                         python3-catkin-pkg-modules"
fi
apt-get install --no-install-recommends -y $ADDITIONAL_PACKAGES
# Install Python dependencies
pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $DOCKER_ROS_FILES_PATH/requirements.txt
# Check if user provided CARLA PythonAPI. If not, download it as artifact from CARLA CI pipeline
mkdir -p /opt/carla/PythonAPI
if [ -d "$DOCKER_ROS_FILES_PATH/carla" ]; then
    mv $DOCKER_ROS_FILES_PATH/carla /opt/carla/PythonAPI
else
    mkdir -p /opt/carla/PythonAPI/carla
     curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/develop/download?job=carla:extract_artifacts&job_token=$GIT_HTTPS_PASSWORD"
    unzip artifacts.zip -d /opt/carla/PythonAPI/carla/
fi
# Create a script to append necessary paths to PYTHONPATH and make .bashrc source it
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$ROS_PYTHON_VERSION.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc
