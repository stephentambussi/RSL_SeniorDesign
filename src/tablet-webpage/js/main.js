var app = new Vue({
  el: "#app",
  // storing the state of the page
  data: {
    connected: false,
    loading: true,
    ros: null,
    ws_address: "ws://0.0.0.0:9090",
    logs: [],
    topic: null,
    message: null,
    dragging: false,
    x: "no",
    y: "no",
    dragCircleStyle: {
      margin: "0px",
      top: "0px",
      left: "0px",
      display: "none",
      width: "75px",
      height: "75px",
    },
    // joystick valules
    joystick: {
      vertical: 0,
      horizontal: 0,
    },
    //publisher
    pubInterval: null,
    //mode checks
    manualActive: false,
    followActive: false,
    autoActive: false,
    joyActive: false,
  },
  // helper methods to connect to ROS
  methods: {
    connect: function () {
      console.log("connecting to rosbridge server");
      this.logs.unshift("Connecting to rosbridge server");
      this.ros = new ROSLIB.Ros({
        url: this.ws_address,
      });
      this.ros.on("connection", () => {
        this.connected = true;
        this.loading = false;
        this.logs.unshift("Connected to rosbridge server");
        this.pubInterval = setInterval(this.joyPublish, 100);

        this.setCamera()
      });
      this.ros.on("error", (error) => {
        this.logs.unshift("Error connecting to rosbridge server");
      });
      this.ros.on("close", () => {
        this.connected = false;
        this.logs.unshift("Disconnected from rosbridge server");
        document.getElementById('divCamera').innerHTML = ''
        clearInterval(this.pubInterval);
      });
    },
    disconnect: function () {
      this.ros.close();
    },
    // Old code for publishing joy commands before joystick existed
    setTopic: function () {
      this.topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cmd_vel",
        messageType: "geometry_msgs/Twist",
      });
    },
    forward: function () {
      this.message = new ROSLIB.Message({
        linear: { x: 1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      this.setTopic();
      this.topic.publish(this.message);
    },
    stop: function () {
      this.message = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      this.setTopic();
      this.topic.publish(this.message);
    },
    backward: function () {
      this.message = new ROSLIB.Message({
        linear: { x: -1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      this.setTopic();
      this.topic.publish(this.message);
    },
    turnLeft: function () {
      this.message = new ROSLIB.Message({
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0.5 },
      });
      this.setTopic();
      this.topic.publish(this.message);
    },
    turnRight: function () {
      this.message = new ROSLIB.Message({
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -0.5 },
      });
      this.setTopic();
      this.topic.publish(this.message);
    },
    clearLogs: function () {
      this.logs = [];
    },
    //activate manual mode
    manualActivate: function () {
      // document.getElementById('frame').contentWindow.location.reload();
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: 0});
      console.log("publishing manual active");
      this.manualActive = true;
      topic.publish(message);
      console.log(message)
    },
    deactivateManual: function (){
      this.manualActive = false;
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: -1});
      topic.publish(message);
      console.log(message)
    },
    //activate autonomous mode
    autoActivate: function () {
      // document.getElementById('frame').contentWindow.location.reload();
      this.autoActive = true
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: 1});
      console.log("publishing auto active");
      topic.publish(message);
      console.log(message)
    },
    deactivateAuto: function(){
      this.autoActive = false
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: -1});
      topic.publish(message);
      console.log(message)
    },
    //activate follow mode
    followActivate: function () {
      // document.getElementById('frame').contentWindow.location.reload();
      this.followActive = true
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: 2});
      console.log("publishing follow active");
      topic.publish(message);
      console.log(message)
    },
    deactivateFollow: function() {
      this.followActive = false;
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/start_mode",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: -1});
      topic.publish(message);
      console.log(message)
    },
    goForOne: function() {
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/go_to",
        messageType: "std_msgs/Int16",
      });
      let message = new ROSLIB.Message({data: 10});
      topic.publish(message);
      console.log(message)
    },
    joyPublish: function () {
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cmd_vel",
        messageType: "geometry_msgs/Twist",
      });
      let message = new ROSLIB.Message({
        linear: { x: this.joystick.vertical, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: this.joystick.horizontal },
      });
      //console.log("publish joy");
      topic.publish(message);
      //console.log(message)
    },
    startDrag() {
      this.dragging = true;
      this.x = this.y = 0;
    },
    stopDrag() {
      this.dragging = false;
      this.x = this.y = "no";
      this.dragCircleStyle.display = "none";
      this.resetJoystickVals();
    },
    doDrag(event) {
      if (this.dragging) {
        this.x = event.offsetX;
        this.y = event.offsetY;
        let ref = document.getElementById("dragstartzone");
        this.dragCircleStyle.display = "inline-block";

        let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2;
        let maxTop = minTop + 200;
        let top = this.y + minTop;
        this.dragCircleStyle.top = `${top}px`;

        let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2;
        let maxLeft = minLeft + 200;
        let left = this.x + minLeft;
        this.dragCircleStyle.left = `${left}px`;

        this.setJoystickVals();
      }
    },
    setJoystickVals() {
      this.joystick.vertical = -1 * (this.y / 200 - 0.5);
      this.joystick.horizontal = -1 * (this.x / 200 - 0.5);
      this.joyPublish();
    },
    resetJoystickVals() {
      this.joystick.vertical = 0;
      this.joystick.horizontal = 0;
    },
    openTab: function (evt, mode) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tabcontent");
      for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      tablinks = document.getElementsByClassName("tablinks");
      for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      document.getElementById(mode).style.display = "block";
      evt.currentTarget.className += " active";
    },
  },
});
