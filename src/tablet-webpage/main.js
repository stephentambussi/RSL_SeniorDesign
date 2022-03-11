var app = new Vue({
  el: "#app",
  // storing the state of the page
  data: {
    connected: false,
    ros: null,
    ws_address: "",
    logs: [],
    topic: null,
    message: null
  },
  // helper methods to connect to ROS
  methods: {
    connect: function () {
      console.log("connecting to rosbridge server");
      this.logs.unshift('Connecting to rosbridge server');
      this.ros = new ROSLIB.Ros({
        url: this.ws_address,
      });
      this.ros.on("connection", () => {
        this.connected = true;
        this.logs.unshift('Connected to rosbridge server');
        // console.log("Connected!");
      });
      this.ros.on("error", (error) => {
          this.logs.unshift("Error connecting to rosbridge server");
        // console.log("Error connecting to websocket server: ", error);
      });
      this.ros.on("close", () => {
        this.connected = false;
        this.logs.unshift("Disconnected from rosbridge server");
        // console.log("Connection to websocket server closed.");
      })
    },
    disconnect: function () {
        this.ros.close();
    },
    setTopic: function(){
        this.topic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/cmd_vel',
          messageType: 'geometry_msgs/Twist'
        })
    },
    forward: function(){
        this.message = new ROSLIB.Message({
          linear: { x: 1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0},
        })
        this.setTopic()
        this.topic.publish(this.message)
    },
    stop: function(){
      this.message = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0},
      })
      this.setTopic()
      this.topic.publish(this.message)
    },
    backward: function(){
      this.message = new ROSLIB.Message({
        linear: { x: -1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0},
      })
      this.setTopic()
      this.topic.publish(this.message)
    },
    turnLeft: function(){
      this.message = new ROSLIB.Message({
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0.5},
      })
      this.setTopic()
      this.topic.publish(this.message)
    },
    turnRight: function(){
      this.message = new ROSLIB.Message({
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -0.5},
      })
      this.setTopic()
      this.topic.publish(this.message)
    },
    clearLogs: function () {
        this.logs = [];
        }
  },
});
