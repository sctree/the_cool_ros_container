right now-ish
- get frontend of HTML visualizer thing working:
    ✅ - create docker_home/visualizer/
        - put the HTML and js in there
        - name the html file `index.html`
    ✅ - create docker_home/run/viz
        - make it an executable shell script
        - have it do:
            - `cd "$HOME/visualizer"; archy .`
    ✅ - run/viz # in docker
    ✅ - that command should print out a port number, add that to run/enter (follow the pattern of the vnc port)
    ✅ - in the browser (outside of docker) open the url printed by the run/viz
    ✅ - if it works, that means the frontend is functional
- get the fake backend working in docker
    ✅ - add docker_home/run/serve_rosbag
    ✅ - make it an executable shell script
    ✅ - have it just call `rosbagAsBackend "$@" &;_pid=$!;echo "to stop the server do:"; echo "   kill $_pid"`
    ✅ - put that test rosbag file youve got into docker_home/visualizer/test.rosbag
    ✅ - `run/serve_rosbag --rosbag-file ./visualizer/test.rosbag`
    ✅ - `run/viz`
    ❌ - check the website see if its successfully connecting to the backend. if yes, great fake backend is done.

- Networking
- search for websocket problems through docker (how ws works with docker)
    - acquire general knowledge like (when does hostname matter when starting a local server)
    - whats the diff b/t localhost and 127.0.0.1 (may b something with docker/sockets or hostnames)
    - confuosn b/t hostnames in and out of the docker container


- get real backend working
  - google roslibjs tutorial
  - look for the "ros bridge" part of the tutorial
  - understand how it's supposed to work (install steps, etc), and be ready to tell me about it
  - follow the steps, probably install a ros package
  - try to follow the workspace patters (make file under run/that makes sense)
  - turn spot on
  - start the rosbridge or whatever
  - run/viz
  - see if its visualizing the live ros data, if yes fantastic we can actually visuzlize stuff
 
- make docker a bit more user friendly
   - dont do this until after rosbridge/visuals are working
   - install Firefox in the dockerfile (and gitingore the Mozilla files)
   - install zsh (dont make it the default shell, just install it)
   - add docker_home/startup_scripts
      - put Jeff's_git_shortcuts (file in old discord message) inside that folder
      - add "source docker_home/startup_scripts/jeffs_git_shortcuts" to the bottom of docker_home/.bashrc
    - I know you probably want to add starship terminal, but I think the switch from bash to zsh might be too time consuming (and that is needed for startship terimal to work)
      
- Make some real visuals: try to visualize spot on a canvas with spot at the center, and a trail of where spot has been
    - record a 1min rosbag with spot walking around the room
    - use the rosbag to test the visuals locally on your Mac (e.g. find a way to send yourself the rosbag, dont use docker at all just open the HTML and use the fake rosbag server thing)
    - find the topic that has Spot's position, figure out what the data looks like
    - install Cursor (the AI editor) or Amazon Q (vs code extension) on your Mac 
    - "hey here's my ros callback, here's what the x,y data from ros looks like, help me visualize it on my canvas element with spot at the center (blah blah blah)"
    - Be ready to explain to me  everything in the code
      - Ask it lots of questions
      - tell it to make helper functions for redundant things
      - tell it "write example usage jsdoc comments for [helper functions ]"
      - etc
    - check if it actually works
    - eventually test it on spot running live
    - If that works great, next step is repeat the process:
    - display spots battery level
    - render obstacles from Spots obstacle avoidance
    - add a plotly.js graph of Spot's speed over the last minute
    - etc




Learning tasks:
- ## Understanding bash/linux
    - Explain to me how to find and kill a running process
    - Convince me you know what a PID is
    - Tell me about process exit codes 
    - What are common edgecases and painpoints in bash
    - Self quiz on bash argument parsing specifically with env vars as arguments. Tell GPT to make quiz questions that are likely to trip up people new to bash.
    - How does bash/shell startup work (.profile, .bashrc, .zshenv, etc) tell me the difference between all those files 
    - Tell me how to add (and remove!) a command from the command line. What's the difference between a function, executable, and binary
    - Tell me how hashbangs work and what they are. Make a hashbang for a python script, explain to me how you'd make a hashbang for a different language
- ## Code For this Project
    - Your code should always be easily and reliably runnable by someone else. Thats my rule, and I have really high standards for that. This section for that. 
    - Install deno
    - Youll probably have to read deno documentation because chatgpt usually sucks at deno related stuff and hallucinates a lot. Maybe use GPT to search deno docs. 
    - Make a hello world for deno and run it
    - Figure out how to run a remote script  (url) using deno
    - Push your hello world to a public github repo (cool ros repo is fine)
    - Run your hello world using a URL (github raw)
    - Send me the command so I can run it on my machine
    - In a new JS file, make a function, export it, push that file to github
    - Edit your hello world script. Import the function from that other file, but import it from a raw github URL. Call the function
    - Find one of my functions on my good-js github repo, import it, use it, run the script
    - Google "JSR", explain to me what it is, show me a package from it
    - Go to to [https://esm.sh/](https://esm.sh/) explain to me how ESM can be used with JSR. Explain to me how to pick a specific version of a package
    - Use esm.sh to import a library called deno dax
    - Use deno dax to run an echo command (the deno dax github readme has all the info)
    - Use dax to run an echo command AND get the exit code
    - Use dax to run an echo command and store the output (stdout) in a string
    - Use esm.sh to import a library named cliffy (search deno cliffy)
    - Use cliffy to make a little command line app that asks for some different things, fav color, etc
    - Push to github
    - Send me the command so I can run it on my system
    - Go to my github, find deno_bundler, binaryify, html bundle, and archy and install all four of those.
    - Reminder: have me tell you about bundling and debugging URL code
- ## Networking
    - Show me a hello world deno server that uses no packages/libraries
    - What is the difference between a public and private IP address. Tell me what a hostname is
    - Get a deno server running on your computer that I can access from my computer
    - Make a HTML endpoint on the server
    - Make a JSON endpoint on the server
    - Show me how to "hit" those enpoints using the browser or CURL
    - Tell gpt: You have two laptops, both are running Python code. What are some different ways you can get them to send data to eachother, what needs to be done. How can those methods be debugged. 
    - Show me an https server running on your computer. Explain what's necessary for https thats not necessary for http
    - Explain to me the benefits and downsides of websockets. What can they do that http requests cannot. 
    - Explain how to get a certificate / domain from a certificate authority. See if you can get a free domain somewhere 
- ## Later
    - Explain to me how async/await works in JavaScript. Look up visuals that compare it to synchronous and parallel processing. Have gpt quiz you on common async footguns. Try to trip me up with some examples.
    - Explain to me what CORS is, and how it can be a problem when you're using URL imports in JavaScript
    - Tell me the difference between EcmaScript (esm) imports and CommonJS imports. 

# Actual Tasks:
- get systemd installed on docker
- get foxglove listener installed on docker
- install Boston dynamics wrapper for ROS in docker
- install deno in docker
- install VS Code in docker
- Add your hello-world deno server to docker and have it start as part of the bashrc. Access the server (running in docker) from your browser. Debug it with chatgpt.
- See if I can access the server while its running in docker on your computer
- Figure out how to ssh into docker (harder than it might seem, need to install ssh server and do port mapping)
- Google ssh-copy-id and explain to me what it does and how to use it
- Figure out how to run the hello-world of Boston Dynamics spot on your laptop (install python sdk, connect to spot, run a script)
- Figure out how to ssh into the Linux machine on spot (not docker)
- Figure out how to use VS Code remote ssh with the Linux machine (not docker)
- Test building/running your docker image on spot
- Once its running connect to it from your laptop in several different ways
- 1. Be able to ssh from your laptop into docker. This is going to be hard. 
- 2. Use VS Code's remote ssh to connect to the docker filesystem
- 3. Connect to VNC from your laptop into docker
- 4. Get ros running, then try to render foxglove on your laptop 
- Make a nice looking 3D foxglove interface

# Extras / Whenever
- Make the dockerfile more configurable
    - How to set env var from docker run command 
    - vnc password should be env var
    - vnc screen resolution should be env var
    - toggle vnc entirely
- Figure out how Linux desktop stuff works
    - What are the names of common desktops (gnome, unity)
    - How to switch between them
    - How is gnome typically configured
    - How to change gnome themes
    - How to set the font
    - How to add the gnome dock
    - What is X11
- Get webvnc working
    - Understand the commented out code in the docker. Which parts setup the webvnc
- Use your deno server to serve an HTML page that loads a Three.js example with orbit controls
- Setup a websocket connection. Make a deno endpoint for it and have your HTML page connect to it.  
