# Commands

!! Run all of this in `~/cg2111a/Desktop/cg2111a_git`!

```sh
cd ~/cg2111a/Desktop/cg2111a_git
```

Check which port is the Arduino being connected to the RPi

```sh
ls /dev/ttyACM*
```

Sync code on the RPi to latest version from GitHub

```sh
git stash --all && gh repo sync
```

Run code for RPi side

```sh
gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi && ./alex-pi
```
