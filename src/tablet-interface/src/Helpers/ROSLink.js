var ROSLIB = require('roslib')

var ros = new ROSLIB.ros({
    url: 'ws://localhost:9090'
})

ros.on('connection', function(){
    console.log('Connected to websocket server')
})

ros.on('error', function(error){
    console.log('Error connecting to websocket server: ', error)
})

ros.on('close', function(){
    console.log('Connection to websocket server closed')
})

var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twists'
});

var twist = new ROSLIB.Message({
    linear: {
        x : 0.1,
        y : 0.2,
        z : 0.3
    },
    angular: {
        x : -0.1,
        y : -0.2,
        z : -0.3
    }
});

console.log("Publishing cmd_vel")
cmdVel.publish(twist)

export const publishTwishMessage = () => {
    console.log("publishing cmd_vel")
    cmdVel.publish(twist)
}