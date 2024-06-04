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

## Usage

### Map Interactions

- **Recenter Map**: Click the "Recenter Map" button to center the map on the current location marker.
- **Add Destination**: Click the "Add Destination" button and tap on the map to place the destination marker.
- **Modify Waypoints**: Click the "Modify Waypoints" button to add or remove waypoints. Click to add, and double-click to remove waypoints. Press `ESC` to exit edit mode.
- **Send to MABx**: Click the "Send to MABx" button to publish the calculated route.

### ROS Integration

- **GPS Listener**: Subscribes to the `/gps/gps` topic to update the current location marker.
- **Route Publisher**: Publishes the route data to the `here/route` topic.


## Acknowledgements

- [HERE Maps API](https://developer.here.com/)
- [ROSLIB.js](https://github.com/RobotWebTools/roslibjs)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
