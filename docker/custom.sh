# Download PythonAPI as artifact from CARLA CI pipeline
mkdir -p /opt/carla
curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/main/download?job=provide-python-api&job_token=$GIT_HTTPS_PASSWORD"
unzip artifacts.zip
mv artifacts/PythonAPI /opt/carla
rm -rf artifacts

# Create a script to append necessary paths to PYTHONPATH and make .bashrc source it
export PYTHON_VERSION_SHORT=$(python --version | awk -F'[ .]' '{print $2"."$3}')
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$PYTHON_VERSION_SHORT.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc

# Allow proj to automatically download remote grids to interpret the projection string in OpenDRIVE maps
echo "export PROJ_NETWORK=ON" >> /opt/carla/setup.bash

# make sure to install the required transforms3d version
apt-get remove -y python3-transforms3d
pip install transforms3d==0.4.1

apt-get install -y ros-$ROS_DISTRO-vision-opencv
apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp
apt-get install -y ros-$ROS_DISTRO-rqt-image-view
apt-get install -y ros-$ROS_DISTRO-rqt-gui-py
