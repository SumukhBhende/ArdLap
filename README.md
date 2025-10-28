### 3. The Python Backend Server (`app.py`)

This Python script sets up the web server and handles communication with the Arduino.

#### Prerequisites

Install the required Python libraries by running:

```bash
pip install Flask opencv-contrib-python pyserial Flask-Cors
```

---

## How to Run Everything

### 1. Connect the Arduino

Plug your Arduino UNO into your computer's USB port.

### 2. Identify the Serial Port

In the Arduino IDE, go to **Tools → Port** and note the port name (e.g., `COM3` on Windows or `/dev/ttyACM0` on Linux).

### 3. Configure the Python Script

Edit `app.py` and set the `ARDUINO_PORT` variable to the port you found above.

### 4. Start the Server

Open a terminal, navigate to your project folder, and run:

```bash
python app.py
```

You should see a message confirming connection to the Arduino and that the server is running at [http://127.0.0.1:5000](http://127.0.0.1:5000).

### 5. Launch the UI

Open `index.html` in your web browser.

### 6. Control the Machine

Use the **Up**, **Down**, **Left**, and **Right** buttons in the UI. The motors should respond, and log messages will appear in the terminal running the server.