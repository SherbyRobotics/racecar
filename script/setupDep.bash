export ROS_DISTRO=humble

currentPath= pwd
cd ../.. # now in src folder
git clone -b ros2 https://github.com/chameau5050/web_video_server
git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git
git clone -b ros2-master https://github.com/rst-tu-dortmund/teb_local_planner.git

sudo rosdep init
rosdep update 
sudo rosdep install --rosdistro=humble --from-paths src --ignore-src -y
colcon build --symlink-install