// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;

// Calls added inside the anynonymous function are triggered after the page is loaded
$(document).ready(() => {document.getElementById("log").value = "Default text\n"});

// rosbridge / roslibjs function to connect to ROS
function connectROS()
{
    // Connect to the rosbridge server running on localhost, on port 9090
    // HINT: The rosbridge server SHOULD be closed when disconnecting from ROS
    rosbridgeServer = new ROSLIB.Ros({url : "ws://" + rosMasterIp + ":9090"});

    rosbridgeServer.on("connection", () => {
        console.log("Connected to WebSocket server.");

        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic(
            {ros : rosbridgeServer, name : "/prop_cmd", messageType : "geometry_msgs/Twist"});
    });

    rosbridgeServer.on(
        "error", (error) => { console.log("Error connecting to WebSocket server: ", error); });

    rosbridgeServer.on("close", () => { console.log("Closed connection to WebSocket server."); });
}

// Create a message that conforms to the `Twist` structure defined in ROS.
var twist = new ROSLIB.Message(
    {linear : {x : 0.0, y : 0.0, z : 2.0}, angular : {x : 0.0, y : 0.0, z : 0.0}});

// Add timer callbacks here
setInterval(() => {
    if (velocityCmdTopic != null) {
        // HINT: The `twist` object's values SHOULD be updated BEFORE publishing the message.
        // Publish the message
        velocityCmdTopic.publish(twist);
    }
}, 200);
