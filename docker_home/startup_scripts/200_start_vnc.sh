# start vnc 
echo "[ $HOME/.bashrc ] starting vnc by running: /usr/local/etc/spawn-desktop.sh"

read -r -d '' CODE_FOR_BACKGROUND_TASK <<'HEREDOC_NAME'
    if [ -z "$VNC_PASSWORD" ]
    then
        VNC_PASSWORD="password"
    fi
    if [ -z "$VNC_RESOLUTION" ]
    then
        VNC_RESOLUTION="1280x800"
    fi

    # fix nautilus on boot
    mkdir -p "$HOME"/.config/nautilus
    
    # setup vnc
    mkdir -p "$HOME/.vnc"
    chmod 755 "$HOME/.vnc/xstartup"
    echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME"/.vnc/passwd
    chmod 600 "$HOME"/.vnc/passwd
    touch ~/.Xauthority
    chmod 600 "$HOME"/.Xauthority
    mkdir -p "$HOME/.logs"
    vncserver :0 -geometry "$VNC_RESOLUTION" -depth 24 &>"$HOME"/.logs/vnc.log
HEREDOC_NAME
bash -c "$CODE_FOR_BACKGROUND_TASK" &