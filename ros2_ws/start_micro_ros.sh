stty -F /dev/tty_rover_esp 115200 raw -echo -hupcl -crtscts -ixon -ixoff
sleep 1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty_rover_esp -b 115200
