sh /usr/local/etc/spawn-desktop.sh;. "/opt/ros/$ROS_DISTRO/setup.bash" --
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

source ~/.bashrc.ignore
echo
echo 'INSTRUCTIONS'
echo '- make sure you are connected to spot either by ethernet or wifi'
echo '- find the IP address of your spot by trying to ping each of the following IP addresses:'
echo '    - 10.0.0.3'
echo '    - 192.168.50.3'
echo '    - 192.168.80.3'
echo '- if NONE of those show up, then you will likely need to change the'
echo '  networking settings of your host computer. Make sure the connection to '
echo '  spot has the following settings:'
echo '    - IPv4 Method: Manual'
echo '    - Address: 10.0.0.2'
echo '    - Netmask: 255.255.255.0'
echo '    - Gateway: 10.0.0.1'
echo '- if you did all that and still cant ping any of the above IP addresses, then IDK man'
echo '  go read the spot docs '
echo '- once you can ping spot, put your username and password in the following command:'
echo '  roslaunch spot_driver driver.launch username:=YOUR_USERNAME_HERE password:=YOUR_PASSWORD_HERE hostname:=YOUR_IP_ADDRESS_HERE'
echo

