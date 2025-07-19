FROM ubuntu:20.04

# 
# setup root
# 

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# setup timezone and basic tools
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y --no-install-recommends \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# 
# 
# Setup non-root user
# 
# 
ENV NORMAL_USERNAME=user1
ENV NORMAL_PASSWORD=password
RUN useradd -m -s /bin/bash "$NORMAL_USERNAME" && \
    echo "$NORMAL_USERNAME:$NORMAL_PASSWORD" | chpasswd && \
    usermod -aG sudo "$NORMAL_USERNAME"

ENV ROOT_HOME=/root

# 
# install basics
# 
RUN apt-get update && apt-get install -q -y --no-install-recommends \
        tzdata \
        iputils-ping \
        dirmngr \
        gnupg2 \
        build-essential \
        git \
        apt-transport-https \
        vim \
        nano \
        curl \
        unzip \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 
# VS Code debugging link
# 
ENV CODE_SERVER_VERSION=4.22.1
RUN cd /tmp && \
    curl -fOL https://github.com/coder/code-server/releases/download/v${CODE_SERVER_VERSION}/code-server-${CODE_SERVER_VERSION}-linux-amd64.tar.gz && \
    tar -xzf code-server-${CODE_SERVER_VERSION}-linux-amd64.tar.gz && \
    mv code-server-${CODE_SERVER_VERSION}-linux-amd64/bin/code-server /usr/local/bin/code-server && \
    chmod +x /usr/local/bin/code-server && \
    rm -rf code-server-${CODE_SERVER_VERSION}*

# 
# 
# setup ros
# 
# 
ENV ROS_DISTRO=noetic
ENV ROS_VERSION=1.5.0-1

# keys and sources
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros1-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME" && \
    echo "deb [ signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# install
RUN apt-get update && apt-get install --no-install-recommends -y \
        python3-rosdep \
        python3-rosinstall \
        python3-vcstools \
    && rm -rf /var/lib/apt/lists/* && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full="$ROS_VERSION"* \
    && rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get install -y \
        python3-pip \
        python3-dev \
        ros-noetic-joy \
        ros-noetic-interactive-marker-twist-server \
        ros-noetic-teleop-twist-joy \
        ros-noetic-twist-mux \
    && rm -rf /var/lib/apt/lists/* && \
    echo '# for ROS'                              >> "$ROOT_HOME/.bashrc" && \
    echo '. "/opt/ros/$ROS_DISTRO/setup.bash" --' >> "$ROOT_HOME/.bashrc"
    

# 
# boston dynamics sdk setup
# 
ENV BOSDYN_VERSION=5.0.0
RUN python3 -m pip install --upgrade bosdyn-client=="$BOSDYN_VERSION" bosdyn-mission=="$BOSDYN_VERSION" bosdyn-choreography-client=="$BOSDYN_VERSION" bosdyn-orbit=="$BOSDYN_VERSION" bosdyn-choreography-protos=="$BOSDYN_VERSION" bosdyn-api=="$BOSDYN_VERSION" bosdyn-core=="$BOSDYN_VERSION"

# 
# foxglove
# 
RUN apt-get update && \
    apt-get install -y ros-noetic-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install foxglove-websocket

EXPOSE 8765

# Node.js and npm (needed for Foxglove Studio)
#RUN curl -fsSL https://deb.nodesource.com/setup_16.x | bash - && \
#    apt-get install -y nodejs

# # Foxglove Studio (Listener)
#RUN npm install -g @foxglove/studio

# expose port 3000 for the web app
#EXPOSE 3000

#CMD ["foxglove-studio", "--listen", "0.0.0.0"]
# COPY foxglove_bridge.service /etc/systemd/system/foxglove_bridge.service
# RUN systemctl enable foxglove_bridge.service
# install dependencies for Foxglove Studio
# RUN apt-get update && apt-get install -y \
#     curl \
#     gnupg2 \
#     lsb-release \
#     build-essential \
#     libgconf-2-4 \
#     libnss3 \
#     libx11-xcb1 \
#     libsecret-1-0 \
#     libasound2 \
#     libappindicator3-1 \
#     libatk-bridge2.0-0 \
#     libgtk-3-0 \
#     libgbm1 \
#     libxss1 \
#     libxtst6 \
#     ca-certificates \
#     && rm -rf /var/lib/apt/lists/*

# 
# Deno
# 
ENV DENO_VERSION=2.4.1
ENV DENO_INSTALL=/root/.deno
ENV PATH="$DENO_INSTALL/bin:${PATH}"
RUN curl -fsSL https://deno.land/install.sh | sh -s v"$DENO_VERSION" && \
    echo 'export DENO_INSTALL="$HOME/.deno"' >> "$ROOT_HOME/.bashrc" && \
    :

# 
# 
# novnc setup
#
#
# disable it for now
    # ################################################################################
    # # base system
    # ################################################################################

    # RUN sed -i 's#http://archive.ubuntu.com/ubuntu/#mirror://mirrors.ubuntu.com/mirrors.txt#' /etc/apt/sources.list;


    # # built-in packages
    # ENV DEBIAN_FRONTEND noninteractive
    # RUN apt update \
    #     && apt install -y --no-install-recommends software-properties-common curl apache2-utils \
    #     && apt update \
    #     && apt install -y --no-install-recommends --allow-unauthenticated \
    #         supervisor nginx sudo net-tools zenity xz-utils \
    #         dbus-x11 x11-utils alsa-utils \
    #         mesa-utils libgl1-mesa-dri \
    #     && apt autoclean -y \
    #     && apt autoremove -y \
    #     && rm -rf /var/lib/apt/lists/*
    # # install debs error if combine together
    # RUN apt update \
    #     && apt install -y --no-install-recommends --allow-unauthenticated \
    #         xvfb x11vnc \
    #         vim-tiny firefox ttf-ubuntu-font-family ttf-wqy-zenhei  \
    #     && apt autoclean -y \
    #     && apt autoremove -y \
    #     && rm -rf /var/lib/apt/lists/*

    # RUN apt update \
    #     && apt install -y gpg-agent \
    #     && curl -LO https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb \
    #     && (dpkg -i ./google-chrome-stable_current_amd64.deb || apt-get install -fy) \
    #     && curl -sSL https://dl.google.com/linux/linux_signing_key.pub | apt-key add \
    #     && rm google-chrome-stable_current_amd64.deb \
    #     && rm -rf /var/lib/apt/lists/*

    # RUN apt update \
    #     && apt install -y --no-install-recommends --allow-unauthenticated \
    #         lxde gtk2-engines-murrine gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine arc-theme \
    #     && apt autoclean -y \
    #     && apt autoremove -y \
    #     && rm -rf /var/lib/apt/lists/*


    # # Additional packages require ~600MB
    # # libreoffice  pinta language-pack-zh-hant language-pack-gnome-zh-hant firefox-locale-zh-hant libreoffice-l10n-zh-tw

    # # tini to fix subreap
    # ARG TINI_VERSION=v0.18.0
    # ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
    # RUN chmod +x /bin/tini

    # # ffmpeg
    # RUN apt update \
    #     && apt install -y --no-install-recommends --allow-unauthenticated \
    #         ffmpeg \
    #     && rm -rf /var/lib/apt/lists/* \
    #     && mkdir /usr/local/ffmpeg \
    #     && ln -s /usr/bin/ffmpeg /usr/local/ffmpeg/ffmpeg

    # # python library
    # COPY rootfs/usr/local/lib/web/backend/requirements.txt /tmp/
    # RUN apt-get update \
    #     && dpkg-query -W -f='${Package}\n' > /tmp/a.txt \
    #     && apt-get install -y python3-pip python3-dev build-essential \
    # 	&& pip3 install setuptools wheel && pip3 install -r /tmp/requirements.txt \
    #     && ln -s /usr/bin/python3 /usr/local/bin/python \
    #     && dpkg-query -W -f='${Package}\n' > /tmp/b.txt \
    #     && apt-get remove -y `diff --changed-group-format='%>' --unchanged-group-format='' /tmp/a.txt /tmp/b.txt | xargs` \
    #     && apt-get autoclean -y \
    #     && apt-get autoremove -y \
    #     && rm -rf /var/lib/apt/lists/* \
    #     && rm -rf /var/cache/apt/* /tmp/a.txt /tmp/b.txt


    # ################################################################################
    # # builder
    # ################################################################################
    # FROM ubuntu:20.04 as builder


    # RUN sed -i 's#http://archive.ubuntu.com/ubuntu/#mirror://mirrors.ubuntu.com/mirrors.txt#' /etc/apt/sources.list;


    # RUN apt-get update \
    #     && apt-get install -y --no-install-recommends curl ca-certificates gnupg patch

    # # nodejs
    # RUN curl -sL https://deb.nodesource.com/setup_12.x | bash - \
    #     && apt-get install -y nodejs

    # # yarn
    # RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - \
    #     && echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list \
    #     && apt-get update \
    #     && apt-get install -y yarn

    # # build frontend
    # COPY web /src/web
    # RUN cd /src/web \
    #     && yarn \
    #     && yarn build
    # RUN sed -i 's#app/locale/#novnc/app/locale/#' /src/web/dist/static/novnc/app/ui.js


    # ################################################################################
    # # merge
    # ################################################################################
    # FROM system
    # LABEL maintainer="fcwu.tw@gmail.com"

    # COPY --from=builder /src/web/dist/ /usr/local/lib/web/frontend/
    # COPY rootfs /
    # RUN ln -sf /usr/local/lib/web/frontend/static/websockify /usr/local/lib/web/frontend/static/novnc/utils/websockify && \
    # 	chmod +x /usr/local/lib/web/frontend/static/websockify/run

    # EXPOSE 80
    # WORKDIR /root
    # ENV HOME=/home/ubuntu \
    #     SHELL=/bin/bash
    # HEALTHCHECK --interval=30s --timeout=5s CMD curl --fail http://127.0.0.1:6079/api/health

# 
# 
# Setup VNC
# 
# 

#
# Ubuntu Desktop (Gnome) Dockerfile
#
# https://github.com/Lvious/Docker-Ubuntu-Desktop-Gnome
#
RUN set -xe && echo '#!/bin/sh' > /usr/sbin/policy-rc.d && echo 'exit 101' >> /usr/sbin/policy-rc.d && \
    chmod +x /usr/sbin/policy-rc.d && \
    dpkg-divert --local --rename --add /sbin/initctl && \
    cp -a /usr/sbin/policy-rc.d /sbin/initctl && \
    sed -i 's/^exit.*/exit 0/' /sbin/initctl && \
    echo 'force-unsafe-io' > /etc/dpkg/dpkg.cfg.d/docker-apt-speedup && \
    echo 'DPkg::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };' > /etc/apt/apt.conf.d/docker-clean && \
    echo 'APT::Update::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };' >> /etc/apt/apt.conf.d/docker-clean && \
    echo 'Dir::Cache::pkgcache ""; Dir::Cache::srcpkgcache "";' >> /etc/apt/apt.conf.d/docker-clean && \
    echo 'Acquire::Languages "none";' > /etc/apt/apt.conf.d/docker-no-languages && \
    echo 'Acquire::GzipIndexes "true"; Acquire::CompressionTypes::Order:: "gz";' > /etc/apt/apt.conf.d/docker-gzip-indexes && \
    rm -rf /var/lib/apt/lists/* && \
    sed -i 's/^#\s*\(deb.*universe\)$/\1/g' /etc/apt/sources.list
    
# Setup enviroment variables
ENV DEBIAN_FRONTEND=noninteractive

RUN : && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ubuntu-desktop \
        gnome-panel \
        gnome-settings-daemon \
        metacity \
        nautilus \
        tightvncserver \
        gnome-terminal \
        ubuntu-session \
        yaru-theme-gtk \
        yaru-theme-icon \
        yaru-theme-sound \
        gnome-shell-extension-appindicator \
        xfonts-75dpi \
        xfonts-100dpi \
        light-themes \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

ENV DEBUG="1"
ENV VNC_PASSWORD=password
ENV VNC_RESOLUTION=1280x800
ENV _setup_vnc_script_location="/usr/local/etc/spawn-desktop.sh"
RUN :                            && \
    : '### spawn-desktop.sh ###' && \
    :                            && \
    echo 'if [ -z "$VNC_PASSWORD" ]'                                                                                                                                         >> "$_setup_vnc_script_location" && \
    echo 'then'                                                                                                                                                              >> "$_setup_vnc_script_location" && \
    echo '    VNC_PASSWORD="password"'                                                                                                                                       >> "$_setup_vnc_script_location" && \
    echo 'fi'                                                                                                                                                                >> "$_setup_vnc_script_location" && \
    echo 'if [ -z "$VNC_RESOLUTION" ]'                                                                                                                                       >> "$_setup_vnc_script_location" && \
    echo 'then'                                                                                                                                                              >> "$_setup_vnc_script_location" && \
    echo '    VNC_RESOLUTION="1280x800"'                                                                                                                                     >> "$_setup_vnc_script_location" && \
    echo 'fi'                                                                                                                                                                >> "$_setup_vnc_script_location" && \
    echo ''                                                                                                                                                                  >> "$_setup_vnc_script_location" && \
    echo '# fix nautilus on boot'                                                                                                                                            >> "$_setup_vnc_script_location" && \
    echo '[ -n "$DEBUG" ] && echo "SHLVL is $SHLVL"'                                                                                                                         >> "$_setup_vnc_script_location" && \
    echo '[ -n "$DEBUG" ] && echo "whoami is $(whoami)"'                                                                                                                     >> "$_setup_vnc_script_location" && \
    echo 'mkdir -p "$HOME"/.config/nautilus'                                                                                                                                 >> "$_setup_vnc_script_location" && \
    echo 'if [ "$(whoami)" != "root" ] && [ "$SHLVL" = "1" ] || [ "$SHLVL" = "2" ]; then'                                                                                    >> "$_setup_vnc_script_location" && \
    echo '        mkdir -p "$HOME/.vnc"'                                                                                                                                     >> "$_setup_vnc_script_location" && \
    echo '        # setup vnc'                                                                                                                                               >> "$_setup_vnc_script_location" && \
    echo '        rm -f "$HOME"/.vnc/xstartup'                                                                                                                                               >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'unset SESSION_MANAGER'"'"'                                                                             >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'unset DBUS_SESSION_BUS_ADDRESS'"'"'                                                                    >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'export XKL_XMODMAP_DISABLE=1'"'"'                                                                      >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'export XDG_CURRENT_DESKTOP="GNOME-Flashback:GNOME"'"'"'                                                >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'export XDG_MENU_PREFIX="gnome-flashback-"'"'"'                                                         >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup'"'"'                                                >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'[ -r "$HOME"/.Xresources ] && xrdb "$HOME"/.Xresources'"'"'                                            >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"''"'"'                                                                                                  >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'xsetroot -solid grey'"'"'                                                                              >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'vncconfig -iconic &'"'"'                                                                               >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"''"'"'                                                                                                  >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'gnome-session --builtin --session=gnome-flashback-metacity --disable-acceleration-check --debug &'"'"' >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'nautilus &'"'"'                                                                                        >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'gnome-terminal &'"'"'                                                                                  >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'gnome-panel &'"'"'                                                                                     >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'gnome-settings-daemon &'"'"'                                                                           >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        echo '"'"'metacity &'"'"'                                                                                        >> "$HOME/.vnc/xstartup" && \'            >> "$_setup_vnc_script_location" && \
    echo '        chmod 755 "$HOME/.vnc/xstartup"'                                                                                                                           >> "$_setup_vnc_script_location" && \
    echo '        '                                                                                                                                                          >> "$_setup_vnc_script_location" && \
    echo '        mkdir -p "$HOME"/.vnc'                                                                                                                                     >> "$_setup_vnc_script_location" && \
    echo '        echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME"/.vnc/passwd'                                                                                                 >> "$_setup_vnc_script_location" && \
    echo '        chmod 600 "$HOME"/.vnc/passwd'                                                                                                                             >> "$_setup_vnc_script_location" && \
    echo '        touch ~/.Xauthority'                                                                                                                                       >> "$_setup_vnc_script_location" && \
    echo '        chmod 600 "$HOME"/.Xauthority'                                                                                                                             >> "$_setup_vnc_script_location" && \
    echo '        mkdir -p "$HOME/.logs"'                                                                                                                                    >> "$_setup_vnc_script_location" && \
    echo '        echo "starting vnc server"'                                                                                                                                >> "$_setup_vnc_script_location" && \
    echo '        vncserver :0 -geometry "$VNC_RESOLUTION" -depth 24 &>"$HOME"/.logs/vnc.log'                                                                                >> "$_setup_vnc_script_location" && \
    echo 'fi'                                                                                                                                                                >> "$_setup_vnc_script_location" && \
    chmod +x "$_setup_vnc_script_location" && \
    :                                && \
    : '### add to global bashrc ###' && \
    :                                && \
    echo '# if were at the inital non-root login'                                                     >> "$ROOT_HOME/.bashrc" && \
    echo 'if [ "$SHLVL" = "2" ] && ! [ "$(whoami)" = "root" ]'                                        >> "$ROOT_HOME/.bashrc" && \
    echo 'then'                                                                                       >> "$ROOT_HOME/.bashrc" && \
    echo '    # start vnc '                                                                           >> "$ROOT_HOME/.bashrc" && \
    echo '    echo "[ $HOME/.bashrc ] starting vnc by running: '"$_setup_vnc_script_location"'"'      >> "$ROOT_HOME/.bashrc" && \
    echo '    bash "'"$_setup_vnc_script_location"'" &'                                               >> "$ROOT_HOME/.bashrc" && \
    echo 'fi'                                                                                         >> "$ROOT_HOME/.bashrc" && \
    :

# Expose ports.
EXPOSE 5900

# 
# add to global bashrc
#
RUN echo 'normal_username="'"$NORMAL_USERNAME"'"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           >> "/etc/bash.bashrc" && \
    echo 'normal_user_home="/home/'"$NORMAL_USERNAME"'"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    >> "/etc/bash.bashrc" && \
    echo 'root_home="'"$ROOT_HOME"'"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       >> "/etc/bash.bashrc" && \
    echo '# if inital login'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                >> "/etc/bash.bashrc" && \
    echo 'if [ "$SHLVL" = "1" ] && [ "$(whoami)" = "root" ]'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                >> "/etc/bash.bashrc" && \
    echo 'then'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             >> "/etc/bash.bashrc" && \
    echo '    # if first time setup'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        >> "/etc/bash.bashrc" && \
    echo '    if ! [ -f "$normal_user_home/.ignore.init" ]'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 >> "/etc/bash.bashrc" && \
    echo '    then'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         >> "/etc/bash.bashrc" && \
    echo '        echo "[ /etc/bash.bashrc ] setting up home for the first time (checked for .ignore.init file)"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                >> "/etc/bash.bashrc" && \
    echo '        # this loop is so stupidly complicated because of many inherent-to-shell reasons, for example: https://stackoverflow.com/questions/13726764/while-loop-subshell-dilemma-in-bash'                                                                                                                                                                                                                                                                                                                                                                          >> "/etc/bash.bashrc" && \
    echo '        for_each_item_in="$root_home"; [ -z "$__NESTED_WHILE_COUNTER" ] && __NESTED_WHILE_COUNTER=0;__NESTED_WHILE_COUNTER="$((__NESTED_WHILE_COUNTER + 1))"; trap '"'"'rm -rf "$__temp_var__temp_folder"'"'"' EXIT; __temp_var__temp_folder="$(mktemp -d)"; mkfifo "$__temp_var__temp_folder/pipe_for_while_$__NESTED_WHILE_COUNTER"; (find "$for_each_item_in" -maxdepth 1 ! -path "$for_each_item_in" -print0 2>/dev/null | sort -z > "$__temp_var__temp_folder/pipe_for_while_$__NESTED_WHILE_COUNTER" &); while read -d $'"'"'\0'"'"' each'                  >> "/etc/bash.bashrc" && \
    echo '        do'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       >> "/etc/bash.bashrc" && \
    echo '            if [ -d "$each" ]'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    >> "/etc/bash.bashrc" && \
    echo '            then'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 >> "/etc/bash.bashrc" && \
    echo '                chown -R "$normal_username" "$each"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              >> "/etc/bash.bashrc" && \
    echo '            else'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 >> "/etc/bash.bashrc" && \
    echo '                chown "$normal_username" "$each"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 >> "/etc/bash.bashrc" && \
    echo '            fi'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   >> "/etc/bash.bashrc" && \
    echo '            echo "    moving" to normal users home "$each"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       >> "/etc/bash.bashrc" && \
    echo '            mv "$each" "$normal_user_home"'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       >> "/etc/bash.bashrc" && \
    echo '        done < "$__temp_var__temp_folder/pipe_for_while_$__NESTED_WHILE_COUNTER";__NESTED_WHILE_COUNTER="$((__NESTED_WHILE_COUNTER - 1))"'                                                                                                                                                                                                                                                                                                                                                                                                                        >> "/etc/bash.bashrc" && \
    echo '        # setup home of root and normal user'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     >> "/etc/bash.bashrc" && \
    echo '        mkdir -p "$HOME"/Desktop'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 >> "/etc/bash.bashrc" && \
    echo '        mkdir -p "$HOME"/.config/nautilus'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        >> "/etc/bash.bashrc" && \
    echo '        mkdir -p "$normal_user_home"/Desktop'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     >> "/etc/bash.bashrc" && \
    echo '        mkdir -p "$normal_user_home"/Downloads'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   >> "/etc/bash.bashrc" && \
    echo '        mkdir -p "$normal_user_home"/.config/nautilus'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            >> "/etc/bash.bashrc" && \
    echo '        '                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         >> "/etc/bash.bashrc" && \
    echo '        touch "$normal_user_home"/.ignore.init'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   >> "/etc/bash.bashrc" && \
    echo '    fi'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           >> "/etc/bash.bashrc" && \
    echo '    su "$normal_username"; exit'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  >> "/etc/bash.bashrc" && \
    echo 'fi'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               >> "/etc/bash.bashrc" && \
    : 

# 
# add stuff to (what will become) the normal user's bashrc
# 
ENV SILENCE_INSTUCTIONS_DURING_LOGIN=""
RUN : && \
    touch "$ROOT_HOME/.hushlogin" && \
    echo 'if ! [ "$(whoami)" = "root" ]; then'                                                                                                                                                        >> "$ROOT_HOME/.bashrc" && \
    echo '    export SUDO_PROMPT=""'                                                                                                                                                                  >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -n "$DEBUG" ] && echo running normal bashrc as $(whoami)'                                                                                                                                              >> "$ROOT_HOME/.bashrc" && \
    echo '    if [ -f "$HOME/.bashrc.ignore" ]'                                                                                                                                                       >> "$ROOT_HOME/.bashrc" && \
    echo '    then'                                                                                                                                                                                   >> "$ROOT_HOME/.bashrc" && \
    echo '        [ -n "$DEBUG" ] && echo running "$HOME/.bashrc.ignore"'                                                                                                                                              >> "$ROOT_HOME/.bashrc" && \
    echo '        source "$HOME/.bashrc.ignore"'                                                                                                                                                      >> "$ROOT_HOME/.bashrc" && \
    echo '    fi'                                                                                                                                                                                     >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo'                                                                                                                                                                                   >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'INSTRUCTIONS'"'"''                                                                                                                                                            >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- run the following command (in docker) every time you change the ros packages:'"'"''                                                                                         >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'     run/build'"'"''                                                                                                                                                          >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- then make sure you are connected to spot either by ethernet or wifi'"'"''                                                                                                   >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- find the IP address of your spot by trying to ping each of the following IP addresses:'"'"''                                                                                >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - 10.0.0.3'"'"''                                                                                                                                                          >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - 192.168.50.3'"'"''                                                                                                                                                      >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - 192.168.80.3'"'"''                                                                                                                                                      >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- if NONE of those show up, then you will likely need to change the'"'"''                                                                                                     >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'  networking settings of your host computer. Make sure the connection to '"'"''                                                                                               >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'  spot has the following settings:'"'"''                                                                                                                                      >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - IPv4 Method: Manual'"'"''                                                                                                                                               >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - Address: 10.0.0.2'"'"''                                                                                                                                                 >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - Netmask: 255.255.255.0'"'"''                                                                                                                                            >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'    - Gateway: 10.0.0.1'"'"''                                                                                                                                                 >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- if you did all that and still cant ping any of the above IP addresses, then IDK man'"'"''                                                                                   >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'  go read the spot docs '"'"''                                                                                                                                                >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'- once you can ping spot, put your username and password in the following command:'"'"''                                                                                      >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '"'"'  roslaunch spot_driver driver.launch username:=YOUR_USERNAME_HERE password:=YOUR_PASSWORD_HERE hostname:=YOUR_IP_ADDRESS_HERE'"'"''                                          >> "$ROOT_HOME/.bashrc" && \
    echo '    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo'                                                                                                                                                                                   >> "$ROOT_HOME/.bashrc" && \
    echo 'fi'                                                                                                                                                                                         >> "$ROOT_HOME/.bashrc" && \
    echo '    '                                                                                                                                                                                       >> "$ROOT_HOME/.bashrc" && \
    :

# ENV USER="$NORMAL_USERNAME"
VOLUME [ "/home/$NORMAL_USERNAME" ]

CMD ["bash", "-c", "/sbin/init && bash"]