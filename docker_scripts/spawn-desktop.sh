#!/bin/sh
# (c) Pete Birley

#this sets the vnc password
# VNC_PASSWORD is set in the Dockerfile
# VNC_RESOLUTION is set in the Dockerfile

mkdir -p /root/.vnc
echo "$vNC_PASSWORD" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

#fixes a warning with starting nautilus on firstboot - which we will always be doing.
mkdir -p ~/.config/nautilus

#this starts the vnc server
USER=root vncserver :1 -geometry "$VNC_RESOLUTION" -depth 24 &