# Accessing the bot

SSH into the robot, and cd to `~/cg2111a/Desktop/cg2111a_git`!

```sh
cd ~/cg2111a/Desktop/cg2111a_git
```

# Commands

1. Check which port is the Arduino being connected to the RPi. If it is not ttyACM0, change the defined port in `alex-pi.cpp`

```sh
ls /dev/ttyACM*
```

2. Sync code on the RPi to latest version from GitHub (if needed).

```sh
git stash --all && gh repo sync && git log -1
```

3. Use VNC to compile & upload code to the Arduino. The folder with the .ino file is pinned in the sidebar, so you can just open it directly.
<!--  (Sorry I tried to figure out a way to do it with ssh but I cannot figure it out ffs). -->

4. Run code for RPi side (and pray it works).

```sh
gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi && ./alex-pi
```

Extra: Steps 2 to 4 in one command:

```sh
git stash --all && gh repo sync && git log -1 && gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi && ./alex-pi
```

# Troubleshooting

- Check if the port is defined correctly! Sometimes its not ttyACM0
- Anything else ask me (kyuu)
# ROS Networking
If you attempt to install from the scratch, it will be very time consuming (literally 1 hour or even more). I suggest install Ubuntu and proceed step 1, 3.1 Creating New Workspace and Building at home first.
Before attempt to do ROS Networking, remember to ensure at least you can run SLAM as tutorial 3 (otherwise there is high chance there may be hardware problem :)))
https://www.comp.nus.edu.sg/~guoyi/tutorial/cg2111a/ros-network/
1. Set up ROS on your PC: Follow this step on tutorial 4.
2. Activate ROS and LiDAR Node on Raspberry Pi and 3. Activate ROS on PC
   - I will assume you have installed Ubuntu on your device/VM. Can actually see how to install them on Internet.
   - REMEMBER: You need to set up your Ubuntu to be in same network as you.
     Eg: RPi IP is 172.20.10.5. If the IP of your Ubuntu is not 172.20.10.x (x is some number) then you are connected to same network yet.
   You may proceed to find the way yourself. For UTM user, to save time can just change to Bridged Networking (search on Internet for better instruction, or can ask me)
   - Regardless of what may come next, ensure you at least have installed workspace and Hector_slam on your PC first. Follow "3.1 Creating a New Workspace and Building". Only do step 2.Activate ROS and LiDAR Node on Raspberry Pi if you choose to set RPi as master (Jayden and I cannot use that on Mac though)
   - This one serves for setting RPi as master. However, by some reasons cannot then there maybe some problems:
     a. Forget to setup
     ```sh
     source ~/cg2111a/devel/setup.bash
     ```
     at new terminal. Remember this is the code to set the source for ros, not source /opt/ros/noetic/setup.bash (this only serves to build rplidar.ros on your PC)
     b. Try to run
     ```sh
     rostopic list
     ```
     If you cannot see the /scan then probably you did not launch from RPi
     c. If you see the /scan node, then run
     ```sh
     rostopic echo /scan
     ```
     If there is no scan, terminate the action and try to run
     ```sh
     rostopic get info
     ```
     If you see "no subscribers", there may actually be some problems relating to VM or MAC that neither me or prof can explain yet :))). What I suggest is to follow step 5 of tutorial to set up our PC as Master instead (what im doing actually)
  4. Visualize Mapping Data Using Python on PC: This is optional, I have not done this yet
  5. Visualize Mapping Data Using RViz on PC. Follow this if you want to set your PC as the master. Just follow it. You may want to install ros-noectis-hector-slam on your PC as well (just check if you have not installed yet. Below is the code for installation)
       sudo apt-get install ros-noetic-hector-slam
Additionally, when running rViz, having the pointing arrow will help a lot in detecting direction. Can see how to add this feature in Tutorial 3.
If there are any problems, may consider to ask me or Prof.
#Additional Notes on ROS Networking.
Make sure to ony have 1 terminal only on 1 of 2 devices (RPi or PC) running roscore. If you want restart roscore on another terminal or on another device (RPi to PC or vice versa), remember to kill it on old terminal first (control C)
