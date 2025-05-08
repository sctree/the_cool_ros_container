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
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
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
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

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
    echo 'Acquire::GzipIndexes "true"; Acquire::CompressionTypes::Order:: "gz";' > /etc/apt/apt.conf.d/docker-gzip-indexes

RUN rm -rf /var/lib/apt/lists/*

RUN sed -i 's/^#\s*\(deb.*universe\)$/\1/g' /etc/apt/sources.list
	
# Setup enviroment variables
ENV DEBIAN_FRONTEND=noninteractive

ENV USER=root

RUN apt-get update && \
    apt-get install -y --no-install-recommends ubuntu-desktop && \
    apt-get install -y gnome-panel gnome-settings-daemon metacity nautilus gnome-terminal && \
    apt-get install -y tightvncserver && \
    apt-get install -y expect && \
    mkdir /root/.vnc

#Update the package manager and upgrade the system
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get update

ADD ./docker_scripts/xstartup.sh /root/.vnc/xstartup
ADD ./spawn-desktop.sh /usr/local/etc/spawn-desktop.sh
ADD ./docker_scripts/start-vnc-expect-script.sh /usr/local/etc/start-vnc-expect-script.sh


RUN chmod 755 /root/.vnc/xstartup && chmod +x /usr/local/etc/start-vnc-expect-script.sh && chmod +x /usr/local/etc/spawn-desktop.sh
# CMD bash -C '/usr/local/etc/spawn-desktop.sh';'bash'

# Expose ports.
EXPOSE 5901/tcp

# ENTRYPOINT ["/startup.sh"]
# ENV DEBIAN_FRONTEND=noninteractive

CMD ["bash", "-c", "/usr/local/etc/spawn-desktop.sh; source \"/opt/ros/$ROS_DISTRO/setup.bash\" --; bash"]