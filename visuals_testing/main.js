let previousMessage = null
//previousMessage.timestamp = 0
let timelapsed = 0
let count = 0;

function convertROSTimeToMillis(ts) {
    return ts.sec * 1000 + ts.nsec / 1e6;
}

function whenStatusUpdateTopicGiven(message) {
    let data = message.data
    let timestamp = convertROSTimeToMillis(message.timestamp)

    let moving = data.moving
    let sitting = data.sitting
    let standing = data.standing

    console.debug(`message is:`, message)
    console.debug('count: ', count)

    if (previousMessage !== null) {
        let prevTimestamp = convertROSTimeToMillis(previousMessage.timestamp)
        let delta = timestamp - prevTimestamp
        timelapsed += delta
        console.debug("time since last message: ", delta)
    } else {
        console.debug("first message, no time lapsed yet")
    }

    console.debug("time since first message: ", timelapsed)

    count++
    previousMessage = message


    return `moving: ${moving}`
}


