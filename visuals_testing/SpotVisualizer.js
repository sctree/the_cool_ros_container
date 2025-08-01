/**
 * Spot Robot Visualizer Class
 * Handles all visualization and data processing for Spot robot status and position
 */

import { convertROSTimeToMillis, updateStatus, toHumanDuration } from './utils.js';

export class SpotVisualizer {
    /**
     * @param {CanvasRenderingContext2D} ctx - Canvas context for drawing
     * @param {number} canvasWidth - Canvas width in pixels
     * @param {number} canvasHeight - Canvas height in pixels
     * @param {HTMLElement} logDiv - Log div element for text output
     */
    constructor(ctx, canvasWidth = 600, canvasHeight = 600, logDiv = null) {
        this.ctx = ctx;
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        this.logDiv = logDiv;
        
        // State tracking
        this.previousMessage = null;
        this.timeElapsed = 0;
        this.count = 0;
        
        // Status tracking
        this.statusTimes = {
            movingStatusTime: 0,
            sittingStatusTime: 0,
            standingStatusTime: 0
        };
        
        // Position tracking
        this.spotPosition = { x: 0, y: 0, z: 0 };
        this.positionHistory = []; // Store previous positions for trail
        
        // Visualization settings
        this.scale = 100; // 1 meter = 100 pixels
        this.dotRadius = 8;
        this.trailLength = 50; // Number of previous positions to keep
        
        // Pan and zoom state
        this.isPanning = false;
        this.panOffset = { x: 0, y: 0 };
        this.zoomLevel = 1;
        this.isMouseDown = false;
        this.lastMousePos = { x: 0, y: 0 };
        
        // Initialize canvas
        this.initializeCanvas();
        
        // Setup mouse event listeners
        this.setupMouseControls();
    }

    /**
     * Log message to the log stream
     * @param {string} message - Message to log
     */
    logToStream(message) {
        if (this.logDiv) {
            const logStream = this.logDiv.querySelector('#logStream');
            if (logStream) {
                const timestamp = new Date().toLocaleTimeString();
                logStream.innerHTML += `[${timestamp}] ${message}<br>`;
                
                // Limit log stream to 1000 lines
                const lines = logStream.innerHTML.split('<br>');
                if (lines.length > 1000) {
                    logStream.innerHTML = lines.slice(-1000).join('<br>');
                }
                
                logStream.scrollTop = logStream.scrollHeight;
            }
        }
    }

    /**
     * Update latest values display
     * @param {string} key - Key for the value
     * @param {string} value - Value to display
     */
    updateLatestValue(key, value) {
        if (this.logDiv) {
            const latestValues = this.logDiv.querySelector('#latestValues');
            if (latestValues) {
                // Find existing key or create new one
                const existingLine = latestValues.querySelector(`[data-key="${key}"]`);
                if (existingLine) {
                    existingLine.innerHTML = `${key}: ${value}`;
                } else {
                    const newLine = document.createElement('div');
                    newLine.setAttribute('data-key', key);
                    newLine.innerHTML = `${key}: ${value}`;
                    latestValues.appendChild(newLine);
                }
            }
        }
    }

    /**
     * Setup mouse controls for pan and zoom
     */
    setupMouseControls() {
        const canvas = this.ctx.canvas;
        
        // Mouse down event
        canvas.addEventListener('mousedown', (e) => {
            this.isMouseDown = true;
            this.lastMousePos = { x: e.clientX, y: e.clientY };
            e.preventDefault();
        });
        
        // Mouse move event (panning)
        canvas.addEventListener('mousemove', (e) => {
            if (this.isMouseDown) {
                const deltaX = e.clientX - this.lastMousePos.x;
                const deltaY = e.clientY - this.lastMousePos.y;
                
                this.panOffset.x += deltaX;
                this.panOffset.y += deltaY;
                
                this.isPanning = true;
                
                this.lastMousePos = { x: e.clientX, y: e.clientY };
                this.redraw();
            }
            e.preventDefault();
        });
        
        // Mouse up event
        canvas.addEventListener('mouseup', (e) => {
            this.isMouseDown = false;
            e.preventDefault();
        });
        
        // Mouse leave event
        canvas.addEventListener('mouseleave', (e) => {
            this.isMouseDown = false;
            e.preventDefault();
        });
        
        // Wheel event (zooming)
        canvas.addEventListener('wheel', (e) => {
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.zoomLevel *= zoomFactor;
            
            // Limit zoom levels
            this.zoomLevel = Math.max(0.1, Math.min(5, this.zoomLevel));
            
            this.redraw();
            e.preventDefault();
        });
        
        // Double click to reset view
        canvas.addEventListener('dblclick', (e) => {
            this.resetView();
            e.preventDefault();
        });
    }

    /**
     * Reset view to default (center on current position, no pan, zoom = 1)
     */
    resetView() {
        this.panOffset = { x: 0, y: 0 };
        this.zoomLevel = 1;
        this.isPanning = false;
        this.redraw();
    }

    /**
     * Initialize the canvas with background
     */
    initializeCanvas() {
        this.clearCanvas();
        this.drawOriginMarker();
    }

    /**
     * Clear the entire canvas
     */
    clearCanvas() {
        this.ctx.clearRect(0, 0, this.canvasWidth, this.canvasHeight);
        
        // Set white background
        this.ctx.fillStyle = 'white';
        this.ctx.fillRect(0, 0, this.canvasWidth, this.canvasHeight);
    }

    /**
     * Draw grid lines on the canvas
     */
    drawGrid() {
        this.ctx.strokeStyle = '#e0e0e0';
        this.ctx.lineWidth = 1;
        
        // Vertical center line
        this.ctx.beginPath();
        this.ctx.moveTo(this.canvasWidth / 2, 0);
        this.ctx.lineTo(this.canvasWidth / 2, this.canvasHeight);
        this.ctx.stroke();
        
        // Horizontal center line
        this.ctx.beginPath();
        this.ctx.moveTo(0, this.canvasHeight / 2);
        this.ctx.lineTo(this.canvasWidth, this.canvasHeight / 2);
        this.ctx.stroke();
    }

    /**
     * Draw origin marker at the center
     */
    drawOriginMarker() {
        this.ctx.fillStyle = '#666';
        this.ctx.font = '12px Arial';
        this.ctx.fillText('Origin (0,0)', this.canvasWidth / 2 + 5, this.canvasHeight / 2 - 5);
    }

    /**
     * Draw Spot's position with pan and zoom support
     */
    drawSpotPosition() {
        // Save the current canvas state
        this.ctx.save();
        
        // Apply zoom and pan transforms
        this.ctx.translate(this.canvasWidth / 2, this.canvasHeight / 2);
        this.ctx.scale(this.zoomLevel, this.zoomLevel);
        
        // Apply pan offset if user is panning, otherwise center on current position
        if (this.isPanning) {
            this.ctx.translate(this.panOffset.x / this.zoomLevel, this.panOffset.y / this.zoomLevel);
        } else {
            const currentX = this.spotPosition.x * this.scale;
            const currentY = -this.spotPosition.y * this.scale; // Invert Y for standard coordinate system
            this.ctx.translate(-currentX, -currentY);
        }
        
        // Draw blue dot for previous position (if it exists)
        if (this.positionHistory.length > 0) {
            const lastPos = this.positionHistory[this.positionHistory.length - 1];
            const lastX = lastPos.x * this.scale;
            const lastY = -lastPos.y * this.scale;
            
            this.ctx.fillStyle = 'blue';
            this.ctx.beginPath();
            this.ctx.arc(lastX, lastY, this.dotRadius, 0, 2 * Math.PI);
            this.ctx.fill();
        }
        
        // Draw current position as a red dot
        this.ctx.fillStyle = 'red';
        this.ctx.beginPath();
        this.ctx.arc(this.spotPosition.x * this.scale, -this.spotPosition.y * this.scale, this.dotRadius, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // Restore the canvas state
        this.ctx.restore();
        
        // Update latest position value
        this.updateLatestValue('Position', `(${this.spotPosition.x.toFixed(3)}, ${this.spotPosition.y.toFixed(3)}, ${this.spotPosition.z.toFixed(3)})`);
        
        // Update pan/zoom status
        this.updateLatestValue('Panning', this.isPanning ? 'Yes' : 'No');
        this.updateLatestValue('Zoom', this.zoomLevel.toFixed(2));
    }



    /**
     * Log status information to div
     * @param {Object} statusData - Status data object
     * @param {number} timestamp - Current timestamp
     */
    logStatusToDiv(statusData, timestamp) {
        // Update latest values
        this.updateLatestValue('Moving', statusData.moving);
        this.updateLatestValue('Sitting', statusData.sitting);
        this.updateLatestValue('Standing', statusData.standing);
        this.updateLatestValue('Timestamp', timestamp);
    }

    /**
     * Update Spot's position from odometry data
     * @param {Object} positionData - Position data with x, y, z coordinates
     */
    updatePosition(positionData) {
        // Validate that position data has the expected properties
        if (typeof positionData.x === 'undefined' || 
            typeof positionData.y === 'undefined' || 
            typeof positionData.z === 'undefined') {
            return;
        }
        
        // Add current position to history before updating
        if (this.spotPosition.x !== 0 || this.spotPosition.y !== 0 || this.spotPosition.z !== 0) {
            this.positionHistory.push({ ...this.spotPosition });
            
            // Limit history length
            if (this.positionHistory.length > this.trailLength) {
                this.positionHistory.shift(); // Remove oldest position
            }
        }
        
        this.spotPosition.x = positionData.x;
        this.spotPosition.y = positionData.y;
        this.spotPosition.z = positionData.z;
        
        this.redraw();
    }

    /**
     * Handle status update messages
     * @param {Object} message - Status message from ROS
     */
    handleStatusUpdate(message) {
        const data = message.data;
        const timestamp = convertROSTimeToMillis(message.timestamp);
        
        // Update status times if we have previous message
        if (this.previousMessage !== null) {
            const prevTimestamp = convertROSTimeToMillis(this.previousMessage.timestamp);
            const delta = timestamp - prevTimestamp;
            this.timeElapsed += delta;

            const prevData = this.previousMessage.data;
            this.statusTimes = updateStatus(data, prevData, delta, this.statusTimes);
        } else {
            this.timeElapsed = 0;
            this.statusTimes = {
                movingStatusTime: 0,
                sittingStatusTime: 0,
                standingStatusTime: 0
            };
        }

        // Update time elapsed in latest values
        this.updateLatestValue('Time Elapsed', toHumanDuration(this.timeElapsed));
        
        this.count++;
        this.updateLatestValue('Message Index', this.count);
        this.previousMessage = message;
        
        this.redraw();
        
        return `moving: ${data.moving}`;
    }

    /**
     * Handle odometry update messages
     * @param {Object} message - Odometry message from ROS
     */
    handleOdometryUpdate(message) {
        try {
            const actualMessage = message.data;
            
            if (!actualMessage.pose) {
                return;
            }
            
            const data = actualMessage.pose.pose.position;
            this.updatePosition(data);
            
        } catch (error) {
            console.error("Error processing odometry data:", error);
        }
    }

    /**
     * Redraw the entire visualization
     */
    redraw() {
        // Clear canvas for clean redraw with pan/zoom
        //this.clearCanvas();
        this.drawSpotPosition();
        
        // Log status information to div if we have status data
        if (this.previousMessage !== null && this.previousMessage.data) {
            const timestamp = convertROSTimeToMillis(this.previousMessage.timestamp);
            this.logStatusToDiv(this.previousMessage.data, timestamp);
        }
    }

    /**
     * Test function to manually move Spot (for debugging)
     * @param {number} x - X coordinate in meters
     * @param {number} y - Y coordinate in meters
     * @param {number} z - Z coordinate in meters
     */
    testMoveSpot(x, y, z) {
        // Add current position to history before updating
        if (this.spotPosition.x !== 0 || this.spotPosition.y !== 0 || this.spotPosition.z !== 0) {
            this.positionHistory.push({ ...this.spotPosition });
            
            // Limit history length
            if (this.positionHistory.length > this.trailLength) {
                this.positionHistory.shift(); // Remove oldest position
            }
        }
        
        this.spotPosition.x = x;
        this.spotPosition.y = y;
        this.spotPosition.z = z;
        this.redraw();
    }

    /**
     * Get current position
     * @returns {Object} - Current position object
     */
    getPosition() {
        return { ...this.spotPosition };
    }

    /**
     * Get current status times
     * @returns {Object} - Current status times object
     */
    getStatusTimes() {
        return { ...this.statusTimes };
    }
} 