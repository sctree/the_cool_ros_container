cd /root/catkin_ws
python3 -m pip install -e ./src/spot_ros/spot_wrapper/
rm -rf build devel
catkin_make
catkin_make
catkin_make
. ./devel/setup.bash
pip3 install transforms3d
echo roslaunch spot_driver driver.launch username:= password:= hostname:=10.0.0.3 # 192.168.50.3 192.168.80.3
# export PYTHONPATH="$PYTHONPATH:/root/catkin_ws/src/spot_ros"