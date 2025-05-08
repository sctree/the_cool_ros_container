export DESKTOP_SESSION="/usr/share/xsessions/ubuntu.desktop"
export XDG_CURRENT_DESKTOP="ubuntu:GNOME"
export GNOME_SHELL_SESSION_MODE="ubuntu"
export XDG_DATA_DIRS="/usr/share/ubuntu:/usr/local/share/:/usr/share/:/var/lib/snapd/desktop"
dbus-launch --exit-with-session /usr/bin/gnome-session --systemd --session=ubuntu