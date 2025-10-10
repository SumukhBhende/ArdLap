// The distance to move in millimeters (2cm = 20mm)
const moveDistance = 20;

// G-code commands for each direction
const gcodeCommands = {
    up:    `G91 G0 Y${moveDistance}`,  // Relative move, 20mm on Y-axis
    down:  `G91 G0 Y-${moveDistance}`, // Relative move, -20mm on Y-axis
    right: `G91 G0 X${moveDistance}`,  // Relative move, 20mm on X-axis
    left:  `G91 G0 X-${moveDistance}`  // Relative move, -20mm on X-axis
};

// Function to send a command to the server
async function sendCommand(command) {
    try {
        const response = await fetch('http://127.0.0.1:5000/send-gcode', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command }),
        });
        const result = await response.json();
        console.log('Server response:', result);
    } catch (error) {
        console.error('Error sending command:', error);
        alert('Could not connect to the server. Is it running?');
    }
}

// Add event listeners to buttons
document.getElementById('btn-up').addEventListener('click', () => sendCommand(gcodeCommands.up));
document.getElementById('btn-down').addEventListener('click', () => sendCommand(gcodeCommands.down));
document.getElementById('btn-left').addEventListener('click', () => sendCommand(gcodeCommands.left));
document.getElementById('btn-right').addEventListener('click', () => sendCommand(gcodeCommands.right));