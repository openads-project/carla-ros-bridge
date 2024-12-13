# install python3.10 as the carla-simulator currently only supports python3.10 and python3.8

# extend /etc/apt/sources.list with ubuntu 22 repositories to install python3.10
echo "deb http://archive.ubuntu.com/ubuntu/ jammy main restricted" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy-updates main restricted" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy universe" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy-updates universe" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy multiverse" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy-updates multiverse" >> /etc/apt/sources.list
echo "deb http://archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb http://security.ubuntu.com/ubuntu/ jammy-security main restricted" >> /etc/apt/sources.list
echo "deb http://security.ubuntu.com/ubuntu/ jammy-security universe" >> /etc/apt/sources.list
echo "deb http://security.ubuntu.com/ubuntu/ jammy-security multiverse" >> /etc/apt/sources.list

# install python3.10
apt-get update
apt-get install -y python3.10
ln -sf /usr/bin/python3.10 /usr/bin/python3

# install required packages with python3.10
python3.10 -m pip install   pep8==1.7.1 \
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
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py3.10.)" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "source /opt/carla/setup.bash" >> /root/.bashrc

# Allow proj to automatically download remote grids to interpret the projection string in OpenDRIVE maps
echo "export PROJ_NETWORK=ON" >> /opt/carla/setup.bash
