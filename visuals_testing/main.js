/**
 * Main application file for Spot Robot Visualization
 * Handles ROS connections and initializes the visualizer
 */

import { SpotVisualizer } from './SpotVisualizer.js';

let spotVisualizer
// Initialize the application when t`h`e page loads
window.addEventListener('load', function() {
    console.log("Page loaded, initializing SpotVisualizer...");
    
    // Create log div with fixed position
    const logDiv = document.createElement('div');
    logDiv.id = 'spotLog';
    logDiv.style.cssText = `
        position: fixed;
        top: 10px;
        right: 10px;
        width: 350px;
        height: 500px;
        background: rgba(0, 0, 0, 0.9);
        color: white;
        font-family: monospace;
        font-size: 12px;
        border-radius: 5px;
        z-index: 1000;
        display: flex;
        flex-direction: column;
    `;
    
    // Create latest values section
    const latestValuesDiv = document.createElement('div');
    latestValuesDiv.id = 'latestValues';
    latestValuesDiv.style.cssText = `
        padding: 10px;
        border-bottom: 1px solid #444;
        background: rgba(0, 100, 0, 0.3);
        flex-shrink: 0;
    `;
    latestValuesDiv.innerHTML = '<strong>Latest Values:</strong><br>';
    
    // Create log stream section
    const logStreamDiv = document.createElement('div');
    logStreamDiv.id = 'logStream';
    logStreamDiv.style.cssText = `
        padding: 10px;
        overflow-y: auto;
        flex-grow: 1;
        max-height: 400px;
    `;
    
    logDiv.appendChild(latestValuesDiv);
    logDiv.appendChild(logStreamDiv);
    document.body.appendChild(logDiv);
    
    // Get canvas and context
    const canvas = document.getElementById('rosCanvas');
    const ctx = canvas.getContext('2d');
    
    
    // Initialize the SpotVisualizer with log div
    spotVisualizer = new SpotVisualizer(ctx, canvas.width, canvas.height, logDiv);
});

// Legacy function names for backward compatibility with HTML
function whenStatusUpdateTopicGiven(message) {
    // example message: {"name":"/spot/odometry","timestamp":{"sec":1620054112,"nsec":382509383},"data":{"header":{"seq":404,"stamp":{"sec":1620054112,"nsec":315159864},"frame_id":"odom"},"child_frame_id":"body","pose":{"pose":{"position":{"x":-0.3386975796391525,"y":-0.422409087535606,"z":1.1677021958084646},"orientation":{"x":0.0036055801901966333,"y":0.004783695098012686,"z":0.9078847169876099,"w":-0.4191771149635315}},"covariance":{"0":0,"1":0,"2":0,"3":0,"4":0,"5":0,"6":0,"7":0,"8":0,"9":0,"10":0,"11":0,"12":0,"13":0,"14":0,"15":0,"16":0,"17":0,"18":0,"19":0,"20":0,"21":0,"22":0,"23":0,"24":0,"25":0,"26":0,"27":0,"28":0,"29":0,"30":0,"31":0,"32":0,"33":0,"34":0,"35":0}},"twist":{"twist":{"linear":{"x":-0.0004962182138115168,"y":0.00037515314761549234,"z":-0.00021295643819030374},"angular":{"x":0.000249216565862298,"y":0.0011622352758422494,"z":-0.0035485343541949987}},"covariance":{"0":0,"1":0,"2":0,"3":0,"4":0,"5":0,"6":0,"7":0,"8":0,"9":0,"10":0,"11":0,"12":0,"13":0,"14":0,"15":0,"16":0,"17":0,"18":0,"19":0,"20":0,"21":0,"22":0,"23":0,"24":0,"25":0,"26":0,"27":0,"28":0,"29":0,"30":0,"31":0,"32":0,"33":0,"34":0,"35":0}}}}
    
    // if (spotVisualizer) {
    //     return spotVisualizer.handleStatusUpdate(message);
    // }
}
    
function whenOdometryUpdateTopicGiven(message) {
    // Odometry message example: message.data.pose.pose.position
    if (window.spotVisualizer && message && message.data && message.data.pose && message.data.pose.pose && message.data.pose.pose.position) {
        const pos = message.data.pose.pose.position;
        spotVisualizer.updatePosition(pos);
    }
}

// Export functions for potential external use
export { whenStatusUpdateTopicGiven, whenOdometryUpdateTopicGiven };

