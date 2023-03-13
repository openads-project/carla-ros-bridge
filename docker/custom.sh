export DOCKER_ROS_FILES_PATH=/docker-ros/files
pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $DOCKER_ROS_FILES_PATH/requirements.txt
mkdir -p /opt/carla/PythonAPI
if [ -d "$DOCKER_ROS_FILES_PATH/carla" ]; then
    mv $DOCKER_ROS_FILES_PATH/carla /opt/carla/PythonAPI
else
    mkdir -p /opt/carla/PythonAPI/carla
     curl --location --output artifacts.zip --header "PRIVATE-TOKEN: $GIT_HTTPS_PASSWORD" "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/develop/download?job=carla:extract_artifacts"
    unzip artifacts.zip -d /opt/carla/PythonAPI/carla/
fi
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$ROS_PYTHON_VERSION.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc
