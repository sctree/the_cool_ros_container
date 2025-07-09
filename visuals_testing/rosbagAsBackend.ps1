#!/usr/bin/env sh
"\"",`$(echo --% ' |out-null)" >$null;function :{};function dv{<#${/*'>/dev/null )` 2>/dev/null;dv() { #>
echo "2.2.11"; : --% ' |out-null <#'; }; deno_version="$(dv)"; deno="$HOME/.deno/$deno_version/bin/deno"; if [ -x "$deno" ];then  exec "$deno" run -q -A --no-lock --no-config "$0" "$@";  elif [ -f "$deno" ]; then  chmod +x "$deno" && exec "$deno" run -q -A --no-lock --no-config "$0" "$@"; fi; has () { command -v "$1" >/dev/null; };  set -e;  if ! has unzip && ! has 7z; then echo "Can I try to install unzip for you? (its required for this command to work) ";read ANSWER;echo;  if [ "$ANSWER" =~ ^[Yy] ]; then  if ! has brew; then  brew install unzip; elif has apt-get; then if [ "$(whoami)" = "root" ]; then  apt-get install unzip -y; elif has sudo; then  echo "I'm going to try sudo apt install unzip";read ANSWER;echo;  sudo apt-get install unzip -y;  elif has doas; then  echo "I'm going to try doas apt install unzip";read ANSWER;echo;  doas apt-get install unzip -y;  else apt-get install unzip -y;  fi;  fi;  fi;   if ! has unzip; then  echo ""; echo "So I couldn't find an 'unzip' command"; echo "And I tried to auto install it, but it seems that failed"; echo "(This script needs unzip and either curl or wget)"; echo "Please install the unzip command manually then re-run this script"; exit 1;  fi;  fi;   if ! has unzip && ! has 7z; then echo "Error: either unzip or 7z is required to install Deno (see: https://github.com/denoland/deno_install#either-unzip-or-7z-is-required )." 1>&2; exit 1; fi;  if [ "$OS" = "Windows_NT" ]; then target="x86_64-pc-windows-msvc"; else case $(uname -sm) in "Darwin x86_64") target="x86_64-apple-darwin" ;; "Darwin arm64") target="aarch64-apple-darwin" ;; "Linux aarch64") target="aarch64-unknown-linux-gnu" ;; *) target="x86_64-unknown-linux-gnu" ;; esac fi;  print_help_and_exit() { echo "Setup script for installing deno  Options: -y, --yes Skip interactive prompts and accept defaults --no-modify-path Don't add deno to the PATH environment variable -h, --help Print help " echo "Note: Deno was not installed"; exit 0; };  for arg in "$@"; do case "$arg" in "-h") print_help_and_exit ;; "--help") print_help_and_exit ;; "-"*) ;; *) if [ -z "$deno_version" ]; then deno_version="$arg"; fi ;; esac done; if [ -z "$deno_version" ]; then deno_version="$(curl -s https://dl.deno.land/release-latest.txt)"; fi;  deno_uri="https://dl.deno.land/release/v${deno_version}/deno-${target}.zip"; deno_install="${DENO_INSTALL:-$HOME/.deno/$deno_version}"; bin_dir="$deno_install/bin"; exe="$bin_dir/deno";  if [ ! -d "$bin_dir" ]; then mkdir -p "$bin_dir"; fi;  if has curl; then curl --fail --location --progress-bar --output "$exe.zip" "$deno_uri"; elif has wget; then wget --output-document="$exe.zip" "$deno_uri"; else echo "Error: curl or wget is required to download Deno (see: https://github.com/denoland/deno_install )." 1>&2; fi;  if has unzip; then unzip -d "$bin_dir" -o "$exe.zip"; else 7z x -o"$bin_dir" -y "$exe.zip"; fi; chmod +x "$exe"; rm "$exe.zip";  exec "$deno" run -q -A --no-lock --no-config "$0" "$@";     #>}; $DenoInstall = "${HOME}/.deno/$(dv)"; $BinDir = "$DenoInstall/bin"; $DenoExe = "$BinDir/deno.exe"; if (-not(Test-Path -Path "$DenoExe" -PathType Leaf)) { $DenoZip = "$BinDir/deno.zip"; $DenoUri = "https://github.com/denoland/deno/releases/download/v$(dv)/deno-x86_64-pc-windows-msvc.zip";  [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12;  if (!(Test-Path $BinDir)) { New-Item $BinDir -ItemType Directory | Out-Null; };  Function Test-CommandExists { Param ($command); $oldPreference = $ErrorActionPreference; $ErrorActionPreference = "stop"; try {if(Get-Command "$command"){RETURN $true}} Catch {Write-Host "$command does not exist"; RETURN $false}; Finally {$ErrorActionPreference=$oldPreference}; };  if (Test-CommandExists curl) { curl -Lo $DenoZip $DenoUri; } else { curl.exe -Lo $DenoZip $DenoUri; };  if (Test-CommandExists curl) { tar xf $DenoZip -C $BinDir; } else { tar -Lo $DenoZip $DenoUri; };  Remove-Item $DenoZip;  $User = [EnvironmentVariableTarget]::User; $Path = [Environment]::GetEnvironmentVariable('Path', $User); if (!(";$Path;".ToLower() -like "*;$BinDir;*".ToLower())) { [Environment]::SetEnvironmentVariable('Path', "$Path;$BinDir", $User); $Env:Path += ";$BinDir"; } }; & "$DenoExe" run -q -A --no-lock --no-config "$PSCommandPath" @args; Exit $LastExitCode; <# 
# */0}`;
import Bag from "./subrepos/foxglove_rosbag/src/Bag.ts"
import FileReader from "./subrepos/foxglove_rosbag/src/node/FileReader.ts"
// import ArrayReader from "./subrepos/foxglove_rosbag/src/web/ArrayReader.ts"
import { FileSystem, glob } from "https://deno.land/x/quickr@0.8.1/main/file_system.js"
import { certFileContents, keyFileContents } from "./main/dummyCertFiles.js"

import { parseArgs, flag, required, initialValue } from "https://esm.sh/gh/jeff-hykin/good-js@1.14.3.0/source/flattened/parse_args.js"
import { didYouMean } from "https://esm.sh/gh/jeff-hykin/good-js@1.14.3.0/source/flattened/did_you_mean.js"
import stringForIndexHtml from "./main/old/index.html.binaryified.js"

const argsInfo = parseArgs({
    rawArgs: Deno.args,
    fields: [
        [["--debug", "-d", ], flag, ],
        [["--help"], flag, ],
        [["--bag-file"], initialValue(`${FileSystem.thisFolder}/data.ignore/co_ral_narrow.bag`), (str)=>str],
        [["--port"], initialValue(`9093`), (str)=>str],
        [["--address"], initialValue(`127.0.0.1`), (str)=>str],
        [["--list-topics"], flag, ],
        [["--playback-speed", "-s"], initialValue(1), (str)=>parseFloat(str)],
        [["--dummy-wss"], flag, ],
    ],
    namedArgsStopper: "--",
    allowNameRepeats: true,
    valueTransformer: JSON.parse,
    isolateArgsAfterStopper: false,
    argsByNameSatisfiesNumberedArg: true,
    implicitNamePattern: /^(--|-)[a-zA-Z0-9\-_]+$/,
    implictFlagPattern: null,
})
didYouMean({
    givenWords: Object.keys(argsInfo.implicitArgsByName).filter(each=>each.startsWith(`-`)),
    possibleWords: Object.keys(argsInfo.explicitArgsByName).filter(each=>each.startsWith(`-`)),
    autoThrow: true,
    suggestionLimit: 1,
})
const args = argsInfo.simplifiedNames
if (args.help) {
    console.log(`
Usage: rrs [options]

Options:
    --debug, -d
        Run in debug mode (prints more stuff, maybe)
    
    --list-topics
        List all the topics in the rosbag file, then exit

    --playback-speed, -s
        The relative speed to play back the rosbag file at
        default: 1
    
    --bag-file [path]
        The path to the rosbag file to serve
        default: ./data.ignore/co_ral_narrow.bag
    
    --port
        The port to run the server on
        default: 9093

    --address
        The address to run the server on
        default: 127.0.0.1
    
    --dummy-wss
        Use a "secure" websocket connection
        (self-signed cert/key, not actually secure)
`)
}

if (args.debug) {
    console.log(`Loading rosbag file: ${args.bagFile}`)
}
const bag = new Bag(new FileReader(args.bagFile))
await bag.open()
// const bag = new Bag(new FileReader(import.meta.resolve("./data.ignore/co_ral_narrow.bag").slice("file://".length)))
    // bag.startTime
    // bag.endTime
    // bag.bagOpt
const topics = [...bag.connections.values()].map(({ topic, type, messageDefinition, latching }) => ({ topic, type, latching, }))
// messageDefinition
const topicNames = topics.map(({ topic }) => topic)
import * as yaml from "https://deno.land/std@0.168.0/encoding/yaml.ts"
if (args.listTopics) {
    console.log(`# the output is valid yaml (e.g. machine parsable/safe)`)
    console.log(yaml.stringify({topics}))
    Deno.exit()
}

import { BSON } from "https://esm.sh/bson@6.10.4"
import * as CBOR from "https://esm.sh/cbor-js@0.1.0"
function rosEncode(message, compression = "json") {
    const { op, id, topic, msg, service, action } = message
    // op is one of:
        // "publish"
        // "service_response"
        // "call_service"
        // "send_action_goal"
        // "cancel_action_goal"
        // "action_feedback"
        // "action_result"
        // "png"
        // "status"
    // compression is one of:
    // "json"
    // "cbor"
    // "bson"

    let rawData
    if (compression == "json") {
        message = JSON.stringify(message)
    } else if (compression == "cbor") {
        message = CBOR.encode(message)
    } else if (compression == "bson") {
        message = BSON.serialize(message)
    } else {
        throw Error(`Unknown compression type: ${compression}`)
    }

    return message
}

let subscribers = []

//
// start sending out messages
//
;(async () => {
    const playbackSpeed = args.playbackSpeed
    let prevFakeTime = null
    let prevRealTime = 0
    // TODO: to be more efficient, there should be some batching+lookahead here
    for await (const item of bag.messageIterator({ topics: topicNames })) {
        const { topic, connectionId, timestamp, data, message } = item
        const { sec, nsec } = timestamp
        if (prevFakeTime == null) {
            prevFakeTime = sec * 1000 + nsec / 1000000
            prevRealTime = performance.now()
        } else {
            const realTimeGap = performance.now() - prevRealTime
            prevRealTime = performance.now()
            const fakeTime = sec * 1000 + nsec / 1000000
            const desiredTimeGap = (fakeTime - prevFakeTime) / playbackSpeed
            prevFakeTime = sec * 1000 + nsec / 1000000
            if (prevFakeTime >= 2) {
                // 2ms is the smallest realistic amount of time
                await new Promise((r) => setTimeout(r, desiredTimeGap))
            }
        }
        
        // console.log(`sending message of ${topic}`)
        for (const each of subscribers) {
            console.debug(`publishing item.topic is:`,item.topic)
            // FIXME: ensure these are always encoded correctly (how are services handled?)
            each.send(
                rosEncode({
                    op: "publish",
                    topic: item.topic,
                    timestamp,
                    msg: item.message,
                })
            )
        }

        // {
        //     topic: "/clock",
        //     connectionId: 0,
        //     timestamp: { sec: 1720510809, nsec: 899027777 },
        //     data: Uint8Array(8) [
        //         65,  86, 237, 101,
        //         232, 242,  86,  50
        //     ],
        //     message: Record { clock: { sec: 1710052929, nsec: 844559080 } }
        // }
    }
})()

let extras = {}
if (args.dummyWss) {
    extras = {
        cert: certFileContents,
        key: keyFileContents,
    }
}
Deno.serve(
    {
        port: args.port-0,
        hostname: args.address,
        ...extras,
        // onListen: () => {
        //   console.log(`Running on http://127.0.0.1:9093`)
        // },
    },
    (req) => {
        console.debug(`req is:`, req)
        //
        // asked for something other than websocket
        //
        if (req.headers.get("upgrade") != "websocket") {
            return new Response(new TextEncoder().encode("howdee"), { status: 200, headers: { "content-type": "text/plain" } })
        }

        const { socket, response } = Deno.upgradeWebSocket(req)
        subscribers.push(socket)
        socket.addEventListener("open", () => {
            console.log("a client connected!")
        })
        socket.addEventListener("message", (event) => {
            // TODO: clean up
            if (event.data === "ping") {
                console.log(`got ping`)
            }
        })
        socket.addEventListener("close", () => {
            subscribers = subscribers.filter((each) => each !== socket)
        })

        return response
    }
)

// (this comment is part of deno-guillotine, dont remove) #>