var app = new Vue({
  el: "#app",
  // storing the state of the page
  data: {
    connected: false,
    ros: null,
    ws_address: "",
    logs: [],
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
    clearLogs: function () {
        this.logs = [];
        }
  },
});
