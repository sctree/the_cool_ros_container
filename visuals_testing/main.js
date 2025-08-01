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

// Position tracking variables
let spotPosition = { x: 0, y: 0, z: 0 };
let previousPosition = null;

// Initialize the canvas when the page loads
window.addEventListener('load', function() {
    drawSpotPosition();
    
    // Add a test function to manually move the dot (for debugging)
    window.testMoveSpot = function(x, y, z) {
        spotPosition.x = x;
        spotPosition.y = y;
        spotPosition.z = z;
        drawSpotPosition();
        console.log("Test position set to:", spotPosition);
    };
    
    // Test the visualization with a small movement
    setTimeout(() => {
        testMoveSpot(0.1, 0.05, 0); // Move 0.1m right, 0.05m up
    }, 2000);
});

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

function drawSpotPosition() {
    console.log("Drawing Spot position:", spotPosition);
    
    // Clear the canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Set background
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Draw grid lines
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    
    // Vertical center line
    ctx.beginPath();
    ctx.moveTo(canvas.width / 2, 0);
    ctx.lineTo(canvas.width / 2, canvas.height);
    ctx.stroke();
    
    // Horizontal center line
    ctx.beginPath();
    ctx.moveTo(0, canvas.height / 2);
    ctx.lineTo(canvas.width, canvas.height / 2);
    ctx.stroke();
    
    // Draw origin marker
    ctx.fillStyle = '#666';
    ctx.font = '12px Arial';
    ctx.fillText('Origin (0,0)', canvas.width / 2 + 5, canvas.height / 2 - 5);
    
    // Scale factor for visualization (meters to pixels)
    const scale = 100; // 1 meter = 100 pixels
    
    // Calculate position on canvas (center is origin)
    const canvasX = canvas.width / 2 + spotPosition.x * scale;
    const canvasY = canvas.height / 2 - spotPosition.y * scale; // Invert Y for standard coordinate system
    
    console.log("Canvas coordinates:", { canvasX, canvasY, scale });
    
    // Draw Spot as a red dot
    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(canvasX, canvasY, 8, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw position label
    ctx.fillStyle = 'black';
    ctx.font = '14px Arial';
    ctx.fillText(`Spot (${spotPosition.x.toFixed(3)}, ${spotPosition.y.toFixed(3)})`, canvasX + 15, canvasY - 10);
    
    // Draw Z coordinate separately
    ctx.fillText(`Z: ${spotPosition.z.toFixed(3)}m`, canvasX + 15, canvasY + 10);
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
    
    // Draw the position visualization
    drawSpotPosition();
    
    // Draw status text on top
    ctx.fillStyle = 'black';
    ctx.font = '20px Arial';
    ctx.fillText('Moving: ' + data.moving, 20, 30);
    ctx.fillText('Sitting: ' + data.sitting, 20, 60);
    ctx.fillText('Standing: ' + data.standing, 20, 90);
    ctx.fillText('Timestamp: ' + timestamp, 20, 120);
    ctx.fillText('Position X: ' + spotPosition.x.toFixed(3) + 'm', 20, 150);
    ctx.fillText('Position Y: ' + spotPosition.y.toFixed(3) + 'm', 20, 180);
    ctx.fillText('Position Z: ' + spotPosition.z.toFixed(3) + 'm', 20, 210);

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

// Function to handle odometry updates
function whenOdometryUpdateTopicGiven(message) {
    try {
        console.log("=== ODOMETRY MESSAGE RECEIVED ===");
        console.log("Full message:", message);
        console.log("Message keys:", Object.keys(message));
        
        let actualMessage = message.data;
        console.log("actualMessage.pose:", actualMessage.pose);
        
        if (!actualMessage.pose) {
            console.error("Message does not have 'pose' property");
            console.log("Available properties:", Object.keys(actualMessage));
            return;
        }
        
        let data = actualMessage.pose.pose.position;
        
        console.log("Position data:", data);
        updatePositionFromData(data);
        
        console.log("=== END ODOMETRY PROCESSING ===");
        
    } catch (error) {
        console.error("Error processing odometry data:", error);
        console.error("Message structure:", message);
        console.error("Message keys:", Object.keys(message));
    }
}

// Function to update position from data and redraw
function updatePositionFromData(data) {
    console.log("Raw position data:", data);
    console.log("Previous position:", spotPosition);
    
    // Validate that position data has the expected properties
    if (typeof data.x === 'undefined' || typeof data.y === 'undefined' || typeof data.z === 'undefined') {
        console.error("Position data missing x, y, or z coordinates");
        console.log("Position data:", data);
        return;
    }
    
    // Check if position actually changed
    const oldX = spotPosition.x;
    const oldY = spotPosition.y;
    const oldZ = spotPosition.z;
    
    spotPosition.x = data.x;
    spotPosition.y = data.y;
    spotPosition.z = data.z;
    
    console.log("New position:", spotPosition);
    console.log("Position changed:", 
        oldX !== spotPosition.x || 
        oldY !== spotPosition.y || 
        oldZ !== spotPosition.z
    );
    
    // Always redraw the canvas with new position, regardless of previousMessage
    drawSpotPosition();
    
    // Redraw status text if we have status data
    if (previousMessage !== null && previousMessage.data) {
        ctx.fillStyle = 'black';
        ctx.font = '20px Arial';
        ctx.fillText('Moving: ' + previousMessage.data.moving, 20, 30);
        ctx.fillText('Sitting: ' + previousMessage.data.sitting, 20, 60);
        ctx.fillText('Standing: ' + previousMessage.data.standing, 20, 90);
    }
    
    // Always show position data
    ctx.fillStyle = 'black';
    ctx.font = '20px Arial';
    ctx.fillText('Position X: ' + spotPosition.x.toFixed(3) + 'm', 20, 150);
    ctx.fillText('Position Y: ' + spotPosition.y.toFixed(3) + 'm', 20, 180);
    ctx.fillText('Position Z: ' + spotPosition.z.toFixed(3) + 'm', 20, 210);
}

