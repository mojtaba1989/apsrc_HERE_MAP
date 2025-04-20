#!/bin/bash

# Run Node.js server in the background
echo "Starting Node.js server..."
node server.js &

# Save the PID so we can manage it if needed
NODE_PID=$!

# Start rosbridge_server
echo "Starting rosbridge_websocket..."
roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0

# Optional: when rosbridge stops, kill Node.js
echo "Shutting down Node.js server (PID $NODE_PID)..."
kill $NODE_PID