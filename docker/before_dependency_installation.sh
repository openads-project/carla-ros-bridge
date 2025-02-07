# add apt key for ros2-tier3-pkgs
apt-get update
apt-get install -y software-properties-common
add-apt-repository universe
wget -O /etc/apt/keyrings/ros2-tier3-pkgs-pub.gpg.key https://raw.githubusercontent.com/meetgandhi-dev/ros2_tier3_packages/main/ros2-tier3-pkgs-pub.gpg.key
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2-tier3-pkgs-pub.gpg.key] https://raw.githubusercontent.com/meetgandhi-dev/ros2_tier3_packages/main/debian_packages $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2-tier3-pkgs.list > /dev/null

# add rosdep list for jazzy (ubuntu 22.04)
apt-get update
apt-get install -y ros-jazzy-rosdep-jammy
(rosdep init || true)
rosdep update