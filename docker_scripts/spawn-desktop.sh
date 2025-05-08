#!/bin/sh
# (c) Pete Birley

#this sets the vnc password
vnc_password="cbd109" # PARAMETER, feel free to change this
resolution="1280x800" # PARAMETER, feel free to change this

mkdir -p /root/.vnc
echo "$vnc_password" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

#fixes a warning with starting nautilus on firstboot - which we will always be doing.
mkdir -p ~/.config/nautilus

#this starts the vnc server
USER=root vncserver :1 -geometry "$resolution" -depth 24 &