export DESKTOP_SESSION="/usr/share/xsessions/ubuntu.desktop"
export XDG_CURRENT_DESKTOP="ubuntu:GNOME"
export GNOME_SHELL_SESSION_MODE="ubuntu"
export XDG_DATA_DIRS="/usr/share/ubuntu:/usr/local/share/:/usr/share/:/var/lib/snapd/desktop"
dbus-launch --exit-with-session /usr/bin/gnome-session --systemd --session=ubuntu


export XDG_SESSION_TYPE=x11
export DESKTOP_SESSION=gnome

# Start dbus
eval $(dbus-launch --sh-syntax)
export DBUS_SESSION_BUS_ADDRESS
export DBUS_SESSION_BUS_PID

# Start GNOME
gnome-session &

echo 'export XDG_SESSION_TYPE=x11
export DESKTOP_SESSION=gnome

# Start dbus
eval $(dbus-launch --sh-syntax)
export DBUS_SESSION_BUS_ADDRESS
export DBUS_SESSION_BUS_PID

# Start GNOME
gnome-session &' > /root/.vnc/xstartup

chmod +x ~/.vnc/xstartup
vncserver -kill :1
vncserver :1