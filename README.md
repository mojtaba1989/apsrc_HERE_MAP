# APSRC HERE-ROS Bridge

This project integrates HERE Maps with ROS (Robot Operating System) to facilitate routing and navigation functionalities. It allows users to add waypoints, calculate routes, and send route data to a ROS node. 

## Prerequisites
- Access to the HERE Maps API.
- ROS environment with `rosbridge_server` installed and running.
- [apsrc_msgs](https://github.com/mojtaba1989/apsrc_msgs.git)

## Features

- Display a map centered at APSLAB (47.16984, -88.50701).
- Add destination and waypoints interactively on the map.
- Modify and remove waypoints.
- Calculate routes with multiple alternatives.
- Publish route data to a ROS topic.

## Getting Started

1. **Clone the Repository**: Clone the project to your local machine.
   
2. **HERE Maps API Key**: Ensure you have a valid HERE Maps API key. Replace the placeholder in the script with your actual API key:
   ```javascript
   const platform = new H.service.Platform({
       'apikey': 'your_api_key_here'
   });
   ```

3. **ROS Bridge**: Ensure the `rosbridge_server` is running:
   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

## How to run Demo

### 🛠 Requirements

- Node.js (with npm)
- ROS (with `rosbridge_server`)

---

### 📦 Installation
1. Install Node.js and npm

```bash
sudo apt update
sudo apt install nodejs npm
```

2. Initialize the project (if not already done)
```bash 
npm init -y
```
3. Install required dependencies
```bash
npm install express ws
```

4. Replace the host ip address in ***public/index.html*** with the actual IP address of your machine. This step is crucial for the Node.js server to connect to the ROS bridge from other devices.

5. Run all
```bash
chmod +x run.bash
./run.bash
```

6. Setup a wifi hotspot and connect to it using the provided credentials.

7. Connect to the hotspot and open the web interface at http://<host_ip>:8000

## Usage

### Map Interactions

- **Recenter Map**: Click the "Recenter Map" button to center the map on the current location marker.
- **Add Destination**: Click the "Add Destination" button and tap on the map to place the destination marker.
- **Modify Waypoints**: Click the "Modify Waypoints" button to add or remove waypoints. Click to add, and double-click to remove waypoints. Press `ESC` to exit edit mode.
- **Send to MABx**: Click the "Send to MABx" button to publish the calculated route.

### ROS Integration

- **GPS Listener**: Subscribes to the `/gps/gps` topic to update the current location marker.
- **Route Publisher**: Publishes the route data to the `here/route` topic.

## HERE Map Stand-Alone Application

The stand-alone version of the HERE map application allows users to input origin latitude and longitude coordinates via a dialog box. The **Export** function enables the downloading of route information in XML format.

To generate tables and summaries from the XML data, use the **xml2csv.py** script. Open the XML file(s) using the designated button or by pressing `Ctrl + O`.

### Python Requirements

- pandas
- numpy
- PyQt5

## Action Dictionary

- 0: No Action
- 1: None of the following
- 2: Ramp
- 3: Turn Left
- 4: Turn Right
- 5: Exit
- 6: Roundabout
- 7: U-Turn
- 8: Continue/Keep

## Acknowledgements

- [HERE Maps API](https://developer.here.com/)
- [ROSLIB.js](https://github.com/RobotWebTools/roslibjs)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
