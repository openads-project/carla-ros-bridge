export DOCKER_ROS_FILES_PATH=/docker-ros/additional-files

# Check if user provided CARLA PythonAPI. If not, download it as artifact from CARLA CI pipeline
export PYTHON_VERSION_SHORT=$(python --version | awk -F'[ .]' '{print $2"."$3}')
mkdir -p /opt/carla
if [ -d "$DOCKER_ROS_FILES_PATH/artifacts/PythonAPI" ]; then
    mv $DOCKER_ROS_FILES_PATH/artifacts/PythonAPI /opt/carla/PythonAPI
else
    mkdir -p /opt/carla
    curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/main/download?job=provide-python-api&job_token=$GIT_HTTPS_PASSWORD"
    unzip artifacts.zip
    mv artifacts/PythonAPI /opt/carla
    rm -rf artifacts
fi
# Create a script to append necessary paths to PYTHONPATH and make .bashrc source it
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$PYTHON_VERSION_SHORT.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc

# Allow proj to automatically download remote grids to interpret the projection string in OpenDRIVE maps
echo "export PROJ_NETWORK=ON" >> /opt/carla/setup.bash
