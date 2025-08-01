/**
 * Main application file for Spot Robot Visualization
 * Handles ROS connections and initializes the visualizer
 */

import { SpotVisualizer } from './SpotVisualizer.js';

// Global visualizer instance
let spotVisualizer;

// Initialize the application when the page loads
window.addEventListener('load', function() {
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

// Export functions for potential external use (no-ops for now)
export function whenStatusUpdateTopicGiven() {}
export function whenOdometryUpdateTopicGiven() {}

