#!/bin/bash

# special thanks to chatgpt for helping me write this thank god

# Find the first connected port dynamically
port=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)

# Extract the last number from the port name (assuming it's ttyACM0/ttyACM1 format)
port_number=$(echo "$port" | sed 's/.*ttyACM\([0-9]\)/\1/')

# Check if a port is found
if [ -z "$port_number" ]; then
    echo "[!] No Arduino port found."
    exit 1
fi

echo "\n\n  ^~^  ,\n ('Y') )\n /   \/ \n(\|||/) Let's go, Alex! [hjw & kgur]\n\n\033[1m\033[1;35mStartup steps...\033[0m\033[0m\n1. Connected port is: \033[1;33m$(ls /dev/ttyACM*)\033[0m" 

git stash --all -q
gh repo sync > ~/nul 

git log -1 --pretty=format:"2. Code updated to most recent commit %C(Cyan bold)\"%s\"%Creset" 

gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o alex-pi 

echo -n "3. Have you compiled the code on the RPi again? \033[2mPress Enter to continue\033[0m" 
read input 

./alex-pi "$port_number"