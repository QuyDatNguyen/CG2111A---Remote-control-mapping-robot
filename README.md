# Commands

Check which port is being used

```sh
ls /dev/ttyACM*
```

Sync RPi code to latest version from GitHub

```sh
gh repo sync
```

Run RPi side

```sh
gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi && ./alex-pi
```
