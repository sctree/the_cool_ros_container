FROM ubuntu:20.04

# 
# 
# ROS setup
# 
# 

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/* && \
# ping
    apt update && apt install iputils-ping -y && \
# install packages
    apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
        key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
        export GNUPGHOME="$(mktemp -d)"; \
        gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
        mkdir -p /usr/share/keyrings; \
        gpg --batch --export "$key" > /usr/share/keyrings/ros1-latest-archive-keyring.gpg; \
        gpgconf --kill all; \
        rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=noetic

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/* && \
# bootstrap rosdep
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
# install ros packages
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/* && \
#install python3, pip, Spot SDK dependencies
    apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    git \
    sudo \
    apt-transport-https \ 
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --upgrade bosdyn-client bosdyn-mission bosdyn-choreography-client bosdyn-orbit bosdyn-choreography-protos

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

ENV USER=root

RUN apt-get update && \
    apt-get install -y --no-install-recommends ubuntu-desktop && \
    apt-get install -y gnome-panel gnome-settings-daemon metacity nautilus gnome-terminal && \
    apt-get install -y tightvncserver && \
    apt-get install -y expect && \
    mkdir /root/.vnc && \
#Update the package manager and upgrade the system
    apt-get update && \
    apt-get upgrade -y && \
    apt-get update && \ 
    apt-get update && \
    apt-get install -y ubuntu-session yaru-theme-gtk yaru-theme-icon yaru-theme-sound gnome-shell-extension-appindicator && \
    apt-get install -y xfonts-75dpi && \
    apt-get install -y xfonts-100dpi && \
    apt-get install -y light-themes

ADD ./docker_scripts/xstartup.sh /root/.vnc/xstartup
ADD ./docker_scripts/spawn-desktop.sh /usr/local/etc/spawn-desktop.sh

RUN chmod 755 /root/.vnc/xstartup && chmod +x /usr/local/etc/spawn-desktop.sh
# CMD bash -C '/usr/local/etc/spawn-desktop.sh';'bash'
RUN echo 'sh /usr/local/etc/spawn-desktop.sh;. "/opt/ros/$ROS_DISTRO/setup.bash" --' > /root/.bashrc

# Expose ports.
EXPOSE 5901/tcp/3000

#CMD ["bash"]

# install dependencies for systemd
RUN apt-get update && apt-get install -y systemd systemd-sysv

# install dependencies for Foxglove Studio
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    libgconf-2-4 \
    libnss3 \
    libx11-xcb1 \
    libsecret-1-0 \
    libasound2 \
    libappindicator3-1 \
    libatk-bridge2.0-0 \
    libgtk-3-0 \
    libgbm1 \
    libxss1 \
    libxtst6 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Node.js and npm (needed for Foxglove Studio)
RUN curl -fsSL https://deb.nodesource.com/setup_16.x | bash - && \
    apt-get install -y nodejs

# Foxglove Studio (Listener)
RUN npm install -g @foxglove/studio

# expose port 3000 for the web app
#EXPOSE 3000

#CMD ["foxglove-studio", "--listen", "0.0.0.0"]


ENV container=docker
STOPSIGNAL SIGRTMIN+3
VOLUME [ "/sys/fs/cgroup" ]

# ROS and systemd already set up before this point...

# Set up ROS workspace
# Create and initialize ROS workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    #git clone git clone https://github.com/boston-dynamics/spot-sdk.git && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace" && \
    cd /root/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# Install dependencies and Deno
RUN apt-get update && \
    apt-get install -y curl unzip ca-certificates && \
    curl -fsSL https://deno.land/install.sh | sh && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Set environment variables to add Deno to PATH
ENV DENO_INSTALL=/root/.deno
ENV PATH="${DENO_INSTALL}/bin:${PATH}"

# Set the working directory
WORKDIR /app

# Copy your Deno app into the container
COPY . .

# Install code-server (adjust version as needed)
ARG CODE_SERVER_VERSION=4.22.1
RUN cd /tmp && \
    curl -fOL https://github.com/coder/code-server/releases/download/v${CODE_SERVER_VERSION}/code-server-${CODE_SERVER_VERSION}-linux-amd64.tar.gz && \
    tar -xzf code-server-${CODE_SERVER_VERSION}-linux-amd64.tar.gz && \
    mv code-server-${CODE_SERVER_VERSION}-linux-amd64/code-server /usr/local/bin/code-server && \
    chmod +x /usr/local/bin/code-server && \
    rm -rf code-server-${CODE_SERVER_VERSION}*

# Set up openssh server for ssh

# Install ssh
RUN apt-get update && \
    apt-get install -y openssh-server && \
    mkdir /var/run/sshd

# Create user with password
RUN useradd -m -s /bin/bash myuser && \
    echo 'myuser:mypassword' | chpasswd && \
    usermod -aG sudo myuser

# Allow password authentication for sshd
RUN sed -i 's/^#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Enable sshd service in systemd
RUN systemctl enable ssh

# Expose ssh port
EXPOSE 22


CMD ["bash", "-c", "/sbin/init && bash"]
# CMD ["/bin/bash", "-c", "/sbin/init && foxglove-studio --listen 0.0.0.0"] start foxglove

# bash -c runs string as a command in a new bash shell
# /sbin/init starts systemd and if successful, start new interactive bash shell

# interestingly enough, for some reason tightvnvserver: 1 is no longer needed?