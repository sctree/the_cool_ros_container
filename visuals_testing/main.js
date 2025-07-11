let previousMessage = null
previousMessage.timestamp = 0
let timelapsed = 0

function whenStatusUpdateTopicGiven(message) {
    let data = message.data
    let timestamp = message.timestamp

    let moving = data.moving
    let sitting = data.sitting
    let standing = data.standing

    let timelapsed = timelapsed + message.timestamp - previousMessage.timestamp

    console.debug(`message is:`, message)
    previousMessage = message
    console.debug('timelapsed is:', timelapsed)

    return `moving: ${moving}`
}


