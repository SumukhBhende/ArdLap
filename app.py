import serial
import time
from flask import Flask, request, jsonify
from flask_cors import CORS

# --- Configuration ---
# IMPORTANT: Change this to your Arduino's serial port
# Windows: "COM3", "COM4", etc.
# Linux: "/dev/ttyACM0", "/dev/ttyUSB0", etc.
# macOS: "/dev/cu.usbmodemXXXX", "/dev/cu.usbserial-XXXX", etc.
ARDUINO_PORT = "COM3" 
BAUD_RATE = 115200

# Initialize Flask app
app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# Initialize Serial connection
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    # GRBL boots up and needs a second to initialize
    time.sleep(2) 
    ser.flushInput()
    print(f"Connected to Arduino on {ARDUINO_PORT}")
except serial.SerialException as e:
    print(f"Error: Could not open serial port {ARDUINO_PORT}. {e}")
    ser = None

@app.route('/send-gcode', methods=['POST'])
def send_gcode():
    if not ser:
        return jsonify({"status": "error", "message": "Serial port not available."}), 500

    data = request.get_json()
    command = data.get('command')

    if not command:
        return jsonify({"status": "error", "message": "No command provided."}), 400

    try:
        # Send the G-code command. Note the newline '\n' is critical for GRBL.
        ser.write((command + '\n').encode('utf-8'))
        
        # Wait for GRBL's 'ok' response
        response = ser.readline().decode('utf-8').strip()
        
        print(f"Sent: '{command}', Received: '{response}'")
        return jsonify({"status": "success", "command_sent": command, "grbl_response": response})
        
    except Exception as e:
        print(f"Error during serial communication: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000)