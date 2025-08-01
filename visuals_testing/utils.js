/**
 * Utility functions for ROS data processing
 */

/**
 * Convert ROS timestamp to milliseconds
 * @param {Object} ts - ROS timestamp object with sec and nsec properties
 * @returns {number} - Timestamp in milliseconds
 */
export function convertROSTimeToMillis(ts) {
    return ts.sec * 1000 + ts.nsec / 1e6;
}

/**
 * Update status tracking times
 * @param {Object} data - Current status data
 * @param {Object} prevData - Previous status data
 * @param {number} delta - Time delta in milliseconds
 * @param {Object} statusTimes - Object containing status time tracking
 * @returns {Object} - Updated status times
 */
export function updateStatus(data, prevData, delta, statusTimes) {
    const updatedTimes = { ...statusTimes };

    // Update standing status time
    if (data.standing === prevData.standing) {
        updatedTimes.standingStatusTime += delta;
    } else {
        updatedTimes.standingStatusTime = 0;
    }

    // Update sitting status time
    if (data.sitting === prevData.sitting) {
        updatedTimes.sittingStatusTime += delta;
    } else {
        updatedTimes.sittingStatusTime = 0;
    }

    // Update moving status time
    if (data.moving === prevData.moving) {
        updatedTimes.movingStatusTime += delta;
    } else {
        updatedTimes.movingStatusTime = 0;
    }

    return updatedTimes;
}

/**
 * Convert milliseconds to human-readable duration
 * @param {number} ms - Duration in milliseconds
 * @returns {string} - Human-readable duration string
 */
export function toHumanDuration(ms) {
    if (ms < 1000) {
        return `${Math.round(ms)}ms`;
    }
    
    const seconds = Math.floor(ms / 1000);
    if (seconds < 60) {
        return `${seconds}s`;
    }
    
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    if (minutes < 60) {
        return `${minutes}m ${remainingSeconds}s`;
    }
    
    const hours = Math.floor(minutes / 60);
    const remainingMinutes = minutes % 60;
    if (hours < 24) {
        return `${hours}h ${remainingMinutes}m`;
    }
    
    const days = Math.floor(hours / 24);
    const remainingHours = hours % 24;
    return `${days}d ${remainingHours}h`;
}