const canvas = document.getElementById('rosCanvas');
const ctx = canvas.getContext('2d');
ctx.fillStyle = 'black'
ctx.font = '30px Arial'

let previousMessage = null
//previousMessage.timestamp = 0
let timelapsed = 0
let count = 0;
let movingStatus = false;
let sittingStatus = false;
let standingStatus = false;

let movingStatusTime = 0;
let sittingStatusTime = 0;
let standingStatusTime = 0;

function logToPage(...args) {
    const logE1 = document.getElementById('debugLog');
    const message = args.map(arg => {
        if (typeof arg === 'object') {
            return JSON.stringify(arg, null, 2)
        }
        return arg
    }, 2000).join(' ')

    logE1.innerHTML += message + '\n'


    //console.debug(...args)
}

function convertROSTimeToMillis(ts) {
    return ts.sec * 1000 + ts.nsec / 1e6;
}

function updateStatus(data, prevData, delta) {
    if (data.standing == prevData.standing) {
        standingStatusTime += delta
    } else {
        standingStatusTime = 0
    }
    if (data.standing) {
        console.debug("Time spent standing: ", standingStatusTime)
    } else {
        console.debug("Time spent NOT standing: ", standingStatusTime)
    }

    if (data.sitting == prevData.sitting) {
        sittingStatusTime += delta
    } else {
        sittingStatusTime = 0
    }
    if (data.sitting) {
        console.debug("Time spent sitting: ", sittingStatusTime)
    } else {
        console.debug("Time spent NOT sitting: ", sittingStatusTime)
    }

    if (data.moving == prevData.moving) {
        movingStatusTime += delta
    } else {
        movingStatusTime = 0
    }
    if (data.moving) {
        console.debug("Time spent moving: ", movingStatusTime)
    } else {
        console.debug("Time spent NOT moving: ", movingStatusTime)
    }
}

function whenStatusUpdateTopicGiven(message) {
    // clear page
    document.getElementById('rosCanvas').innerHTML = '';

    let data = message.data
    let timestamp = convertROSTimeToMillis(message.timestamp)

    let moving = data.moving
    let sitting = data.sitting
    let standing = data.standing

    console.debug(`message is:`, message)
    console.debug('count: ', count)
    logToPage("testing:::: ", count)
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.fillText('moving statussss: ' + data.moving, canvas.width / 2 - 300, canvas.height / 2 + 100)
    ctx.fillText('sit statuss: ' + data.moving, canvas.width / 2 - 300, canvas.height / 2)
    ctx.fillText('stando: ' + data.moving, canvas.width / 2 - 300, canvas.height / 2 - 100)
    ctx.fillText('tIMESTAMP: ' + timestamp, canvas.width / 2 - 300, canvas.height / 2 - 200)

    if (previousMessage !== null) {
        let prevTimestamp = convertROSTimeToMillis(previousMessage.timestamp)
        let delta = timestamp - prevTimestamp
        timelapsed += delta
        console.debug("time since last message: ", delta)

        let prevData = previousMessage.data
        updateStatus(data, prevData, delta)
    } else {
        timelapsed = 0;
        let movingStatus = false;
        let sittingStatus = false;
        let standingStatus = false;

        let movingStatusTime = 0;
        let sittingStatusTime = 0;
        let standingStatusTime = 0;
        console.debug("first message, no time lapsed yet")
    }

    console.debug("time since first message: ", timelapsed)



    count++
    previousMessage = message


    return `moving: ${moving}`
}


