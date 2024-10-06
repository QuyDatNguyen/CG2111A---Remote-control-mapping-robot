# Accessing the bot

SSH into the robot, and cd to `~/cg2111a/Desktop/cg2111a_git`!


```sh
ssh pi@raspberrypi.local
```

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

3. Use VNC to compile & upload code to the Arduino. The ino file is on the desktop, you can just open it. (Ignore the warning about making a folder by clicking `Cancel` -> `Ok`, it will open normally)
<!--  (Sorry I tried to figure out a way to do it with ssh but I cannot figure it out ffs). -->

4. Run code for RPi side (and pray it works).

```sh
gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi && ./alex-pi
```

Extra: Steps 2 to 4 in one command:

```sh
sh letsgo.sh
```

# Troubleshooting

- Check if the port is defined correctly! Sometimes its not ttyACM0
- Anything else ask me (kyuu)
