apt-get install -y --no-install-recommends \
    libpng16-16 \
    libtiff6 \
    libjpeg8 \
    build-essential \
    wget \
    git \
    libxerces-c-dev \
    python3-pip 

# Ensure libtiff5 compatibility for CARLA on Ubuntu 24.04
if ! ldconfig -p | grep -q "libtiff.so.5"; then
    command -v apt-get >/dev/null 2>&1 && apt-get update && apt-get install -y libtiff5 && ldconfig
    if ! ldconfig -p | grep -q "libtiff.so.5"; then
        if [ -f /usr/lib/x86_64-linux-gnu/libtiff.so.6 ] && [ ! -e /usr/lib/x86_64-linux-gnu/libtiff.so.5 ]; then
            ln -sf /usr/lib/x86_64-linux-gnu/libtiff.so.6 /usr/lib/x86_64-linux-gnu/libtiff.so.5
            ldconfig
        else
            echo "Unable to provide libtiff.so.5 compatibility" >&2
            exit 1
        fi
    fi
fi

export DOCKER_ROS_FILES_PATH=/docker-ros/additional-files

# Download PythonAPI as artifact from CARLA CI pipeline
mkdir -p /opt/carla
curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/ue5-ika/download?job=build-release-docker-image&job_token=$GIT_HTTPS_PASSWORD"
unzip artifacts.zip
mv artifacts/PythonAPI /opt/carla
rm -rf artifacts

# Install the CARLA wheel that matches the current Python minor version
pyver=$(python3 -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
wheel=$(echo /opt/carla/PythonAPI/carla/dist/*${pyver}*.whl)
pip install --no-cache-dir "$wheel" --break-system-packages

# Create a script to append necessary paths to PYTHONPATH
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/agents" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash

# Allow proj to automatically download remote grids to interpret the projection string in OpenDRIVE maps
echo "export PROJ_NETWORK=ON" >> /opt/carla/setup.bash

# Default file cache for CARLA client-side map files
echo "export CARLA_CACHE_DIR=/tmp/carlaCache" >> /opt/carla/setup.bash
echo "mkdir -p /tmp/carlaCache" >> /opt/carla/setup.bash

# .bashrc sources the setup script
echo "source /opt/carla/setup.bash" >> /root/.bashrc
