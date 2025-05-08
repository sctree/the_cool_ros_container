mkdir -p /root/.vnc
echo "cbd109" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd