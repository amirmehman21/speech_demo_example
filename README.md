# speech_demo_example
MatchboxNet + some robot action

## setup 
* first connect to the robot:
1: make a new ethernet connection with manual settings:
ipv4: address: 130.230.37.117, netmask: 255.255.255.0
2: connect to the arm and start the controller: 
in browser go to the web interface with the ip: 130.230.37.117 and turn on the arm


* installation:
1. mkdir -p catkin_ws/src
2. cd catkin_ws/src
3. git clone ....
4. cd 
5. install libfranka based on the instructions:
https://frankaemika.github.io/docs/installation_linux.html#building-from-source
6. sudo apt-get install ros-melodic-sound-play
7. cd ~/catkin_ws
8. catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/folder/build
9. source ~/catkin_ws/devel/setup.bash
10. run demo: roslaunch roslaunch audio_demo start_demo.launch



