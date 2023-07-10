export DOCKER_ROS_FILES_PATH=/docker-ros/additional-files
# Install ROS version dependent apt packages
if [ "$ROS_DISTRO" = "noetic" ]; then
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz
                         ros-$ROS_DISTRO-opencv-apps
                         ros-$ROS_DISTRO-rospy
                         ros-$ROS_DISTRO-rospy-message-converter
                         ros-$ROS_DISTRO-pcl-ros
                         python3-catkin-tools
                         python3-catkin-pkg
                         python3-catkin-pkg-modules"
else
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz2"
fi
apt-get install --no-install-recommends -y $ADDITIONAL_PACKAGES
# Install Python dependencies
pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $DOCKER_ROS_FILES_PATH/requirements.txt
# Check if user provided CARLA PythonAPI. If not, download it as artifact from CARLA CI pipeline
mkdir -p /opt/carla
if [ -d "$DOCKER_ROS_FILES_PATH/PythonAPI" ]; then
    mv $DOCKER_ROS_FILES_PATH/PythonAPI /opt/carla/PythonAPI
else
    mkdir -p /opt/carla
    curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/update/ubuntu20_python3.10/download?job=carla:extract_artifacts&job_token=$GIT_HTTPS_PASSWORD"
    unzip artifacts.zip
    mv artifacts_ci/PythonAPI /opt/carla
    rm -rf artifacts_ci
fi
# Create a script to append necessary paths to PYTHONPATH and make .bashrc source it
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$ROS_PYTHON_VERSION.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc
