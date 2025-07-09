cd /root/catkin_ws
python3 -m pip install -e ./src/spot_ros/spot_wrapper/
rm -rf build devel
catkin_make
catkin_make
catkin_make
. ./devel/setup.bash
pip3 install transforms3d
echo 'pick one of the IP addresses to connect to spot 192.168.50.3 192.168.80.3'
echo roslaunch spot_driver driver.launch username:= password:= hostname:=10.0.0.3 # 192.168.50.3 192.168.80.3
# export PYTHONPATH="$PYTHONPATH:/root/catkin_ws/src/spot_ros"