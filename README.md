# How to Run Everything

## 1. Connect Arduino
Plug your Arduino UNO into your laptop's USB port.

## 2. Find Serial Port
Open the Arduino IDE, go to **Tools → Port**, and note which port your Arduino is on (e.g., `COM3`, `/dev/ttyACM0`).

## 3. Update Python Script
Open `app.py` and set the `ARDUINO_PORT` variable to the port name you found in the previous step.

## 4. Run the Server
Open a terminal or command prompt, navigate to the folder containing your files, and run:

```bash
python app.py
```

You should see a message indicating it's connected to the Arduino and running at [http://127.0.0.1:5000](http://127.0.0.1:5000).

## 5. Open the UI
Double-click the `index.html` file to open it in your web browser.

## 6. Control Your Machine
Click the **Up**, **Down**, **Left**, and **Right** buttons. Your motors should move, and you'll see log messages in the terminal where the server is running.