/**
 * Spot Robot Visualizer Class
 * Handles all visualization and data processing for Spot robot status and position
 */

import { toHumanDuration } from './utils.js';

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

        // Pan and zoom state
        this.isPanning = false;
        this.panOffset = { x: 0, y: 0 };
        this.zoomLevel = 1;
        this.isMouseDown = false;
        this.lastMousePos = { x: 0, y: 0 };

        // Draw the initial green dot at the center
        this.clearCanvas();
        this.drawReferenceDot();

        // Setup mouse event listeners
        this.setupMouseControls();
    }

    clearCanvas() {
        this.ctx.clearRect(0, 0, this.canvasWidth, this.canvasHeight);
        this.ctx.fillStyle = 'white';
        this.ctx.fillRect(0, 0, this.canvasWidth, this.canvasHeight);
    }

    drawReferenceDot() {
        const centerX = this.canvasWidth / 2;
        const centerY = this.canvasHeight / 2;
        this.ctx.save();
        this.ctx.beginPath();
        this.ctx.arc(centerX, centerY, 10, 0, 2 * Math.PI);
        this.ctx.fillStyle = 'green';
        this.ctx.fill();
        this.ctx.restore();
    }

    setupMouseControls() {
        const canvas = this.ctx.canvas;
        canvas.addEventListener('mousedown', (e) => {
            this.isMouseDown = true;
            this.lastMousePos = { x: e.clientX, y: e.clientY };
            e.preventDefault();
        });
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
        canvas.addEventListener('mouseup', (e) => {
            this.isMouseDown = false;
            e.preventDefault();
        });
        canvas.addEventListener('mouseleave', (e) => {
            this.isMouseDown = false;
            e.preventDefault();
        });
        canvas.addEventListener('wheel', (e) => {
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.zoomLevel *= zoomFactor;
            this.zoomLevel = Math.max(0.1, Math.min(5, this.zoomLevel));
            this.redraw();
            e.preventDefault();
        });
        canvas.addEventListener('dblclick', (e) => {
            this.resetView();
            e.preventDefault();
        });
    }

    resetView() {
        this.panOffset = { x: 0, y: 0 };
        this.zoomLevel = 1;
        this.isPanning = false;
        this.redraw();
    }

    redraw() {
        this.clearCanvas();
        this.ctx.save();
        // Pan and zoom transforms
        this.ctx.translate(this.canvasWidth / 2, this.canvasHeight / 2);
        this.ctx.scale(this.zoomLevel, this.zoomLevel);
        this.ctx.translate(this.panOffset.x / this.zoomLevel, this.panOffset.y / this.zoomLevel);
        // Draw the reference green dot at the original center (0,0 in reference frame)
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 10, 0, 2 * Math.PI);
        this.ctx.fillStyle = 'green';
        this.ctx.fill();
        this.ctx.restore();
    }

    // The following are kept for log/values infrastructure
    logToStream(message) {
        if (this.logDiv) {
            const logStream = this.logDiv.querySelector('#logStream');
            if (logStream) {
                const timestamp = new Date().toLocaleTimeString();
                logStream.innerHTML += `[${timestamp}] ${message}<br>`;
                const lines = logStream.innerHTML.split('<br>');
                if (lines.length > 1000) {
                    logStream.innerHTML = lines.slice(-1000).join('<br>');
                }
                logStream.scrollTop = logStream.scrollHeight;
            }
        }
    }

    /**
     * Draw a red dot at the given position (meters), applying pan/zoom transform.
     * @param {Object} pos - {x, y, z} in meters
     */
    updatePosition(pos) {
        this.ctx.save();
        // Pan and zoom transforms
        this.ctx.translate(this.canvasWidth / 2, this.canvasHeight / 2);
        this.ctx.scale(this.zoomLevel, this.zoomLevel);
        this.ctx.translate(this.panOffset.x / this.zoomLevel, this.panOffset.y / this.zoomLevel);
        // Draw the red dot at the given position (meters to pixels, y inverted)
        const scale = 100; // 1 meter = 100 pixels
        const x = pos.x * scale;
        const y = -pos.y * scale;
        this.ctx.beginPath();
        this.ctx.arc(x, y, 8 / this.zoomLevel, 0, 2 * Math.PI);
        this.ctx.fillStyle = 'red';
        this.ctx.fill();
        this.ctx.restore();
    }
} 