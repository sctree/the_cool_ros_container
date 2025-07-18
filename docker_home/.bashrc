# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
#if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
#    . /etc/bash_completion
#fi
# for ROS
. "/opt/ros/$ROS_DISTRO/setup.bash" --
export DENO_INSTALL="$HOME/.deno"
# if were at the inital non-root login
if [ "$SHLVL" = "2" ] && ! [ "$(whoami)" = "root" ]
then
    # start vnc 
    echo "[ $HOME/.bashrc ] starting vnc by running: /usr/local/etc/spawn-desktop.sh"
    bash "/usr/local/etc/spawn-desktop.sh" &
fi
if ! [ "$(whoami)" = "root" ]; then
    export SUDO_PROMPT=""
    [ -n "$DEBUG" ] && echo running normal bashrc as $(whoami)
    if [ -f "$HOME/.bashrc.ignore" ]
    then
        [ -n "$DEBUG" ] && echo running "$HOME/.bashrc.ignore"
        source "$HOME/.bashrc.ignore"
    fi
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo 'INSTRUCTIONS'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- run the following command (in docker) every time you change the ros packages:'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '     run/build'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- then make sure you are connected to spot either by ethernet or wifi'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- find the IP address of your spot by trying to ping each of the following IP addresses:'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - 10.0.0.3'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - 192.168.50.3'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - 192.168.80.3'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- if NONE of those show up, then you will likely need to change the'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '  networking settings of your host computer. Make sure the connection to '
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '  spot has the following settings:'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - IPv4 Method: Manual'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - Address: 10.0.0.2'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - Netmask: 255.255.255.0'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '    - Gateway: 10.0.0.1'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- if you did all that and still cant ping any of the above IP addresses, then IDK man'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '  go read the spot docs '
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '- once you can ping spot, put your username and password in the following command:'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo '  roslaunch spot_driver driver.launch username:=YOUR_USERNAME_HERE password:=YOUR_PASSWORD_HERE hostname:=YOUR_IP_ADDRESS_HERE'
    [ -z "$SILENCE_INSTUCTIONS_DURING_LOGIN" ] && echo
fi
    
