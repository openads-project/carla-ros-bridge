# Setup venv
virtualenv -p python3 ./venv && touch ./venv/COLCON_IGNORE
echo "source $WORKSPACE/venv/bin/activate" /root/.bashrc
source ./venv/bin/activate

pip install pep8==1.7.1 \
            autopep8==2.0.4 \
            cmake_format==0.6.11 \
            pylint==3.0.3 \
            transforms3d==0.4.1 \
            pygame==2.5.2 \
            pexpect==4.9.0 \
            simple-pid==2.0.0 \
            networkx==3.2.1 \
            pyproj==3.6.1

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
