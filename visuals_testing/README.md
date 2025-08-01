# Spot Robot Visualization

A modular, ES6-based visualization system for displaying Spot robot status and position data in real-time.

## Project Structure

```
visuals_testing/
├── main.js              # Main application entry point
├── SpotVisualizer.js    # Core visualization class
├── utils.js             # Utility functions
├── example.html         # HTML interface
└── README.md           # This file
```

## Architecture

### `main.js`
- Application entry point
- Initializes the SpotVisualizer
- Provides legacy function compatibility
- Handles page load events

### `SpotVisualizer.js`
- Core visualization class
- Handles all canvas drawing operations
- Processes ROS messages
- Manages state and position tracking
- Provides clean API for external use

### `utils.js`
- Utility functions for ROS data processing
- Time conversion utilities
- Status tracking helpers
- Debug logging functions

## Features

- **Real-time Position Visualization**: Red dot showing Spot's current position
- **Status Display**: Moving, sitting, and standing status with timestamps
- **Grid System**: Visual grid with origin marker
- **Modular Design**: Clean separation of concerns
- **ES6 Modules**: Modern JavaScript with imports/exports
- **Class-based Architecture**: Object-oriented design with dependency injection

## Usage

### Basic Setup

1. Open `example.html` in a web browser
2. Ensure ROS bridge is running: `roslaunch rosbridge_server rosbridge_websocket.launch`
3. The visualization will automatically connect and display data

### API Usage

```javascript
// Get canvas context
const canvas = document.getElementById('rosCanvas');
const ctx = canvas.getContext('2d');

// Create visualizer instance
const visualizer = new SpotVisualizer(ctx, 600, 600);

// Handle status updates
visualizer.handleStatusUpdate(message);

// Handle odometry updates
visualizer.handleOdometryUpdate(message);

// Test movement (for debugging)
visualizer.testMoveSpot(0.1, 0.05, 0);

// Get current state
const position = visualizer.getPosition();
const statusTimes = visualizer.getStatusTimes();
```

### ROS Topics

- **Status**: `/spot/status/feedback` (spot_msgs/Feedback)
- **Position**: `/spot/odometry` (nav_msgs/Odometry)

## Configuration

### Visualization Settings

The `SpotVisualizer` class accepts configuration options:

```javascript
const visualizer = new SpotVisualizer(ctx, width, height, {
    scale: 100,        // 1 meter = 100 pixels
    dotRadius: 8,      // Red dot radius in pixels
    textSize: 20       // Font size for status text
});
```

### Custom Styling

You can customize the visual appearance by modifying the drawing methods in `SpotVisualizer.js`:

- `drawGrid()` - Grid lines and styling
- `drawSpotPosition()` - Position dot and labels
- `drawStatusText()` - Status information display

## Development

### Adding New Features

1. **New Visualization Elements**: Add methods to `SpotVisualizer` class
2. **New Data Processing**: Add utilities to `utils.js`
3. **New Message Types**: Extend the message handling methods

### Testing

Use the built-in test functions:

```javascript
// Test position movement
window.testMoveSpot(0.1, 0.05, 0);

// Check current state
console.log(spotVisualizer.getPosition());
console.log(spotVisualizer.getStatusTimes());
```

## Dependencies

- **ROSLIB.js**: ROS WebSocket bridge client
- **HTML5 Canvas**: For visualization rendering
- **ES6 Modules**: For code organization

## Browser Compatibility

- Modern browsers with ES6 module support
- Chrome 61+, Firefox 60+, Safari 10.1+, Edge 16+ 