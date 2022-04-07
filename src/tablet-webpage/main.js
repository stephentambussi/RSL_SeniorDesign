let vueApp = new Vue({
  el: "#vueApp",
  data: {
      // ros connection
      ros: null,
      rosbridge_address: 'wss://i-0734dfc7411934198.robotigniteacademy.com/rosbridge/',
      connected: false,
      // page content
      menu_title: 'Connection',
      // dragging data
      dragging: false,
      x: 'no',
      y: 'no',
      dragCircleStyle: {
          margin: '0px',
          top: '0px',
          left: '0px',
          display: 'none',
          width: '75px',
          height: '75px',
      },
      // joystick valules
      joystick: {
          vertical: 0,
          horizontal: 0,
      },
      // publisher
      pubInterval: null,
  },
  methods: {
      connect: function () {
          // define ROSBridge connection object
          this.ros = new ROSLIB.Ros({
              url: this.rosbridge_address
          })

          // define callbacks
          this.ros.on('connection', () => {
              this.logs.unshift((new Date().toTimeString() + ' - Connected!'))
              this.connected = true
              this.loading = false
              this.setCamera()
              console.log('Connection to ROSBridge established!')
              this.pubInterval = setInterval(this.publish, 100)
          })
          this.ros.on('error', (error) => {
              console.log('Something went wrong when trying to connect')
              console.log(error)
          })
          this.ros.on('close', () => {
            this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
              this.connected = false
              this.loading = false
              document.getElementById('divCamera').innerHTML = ''
              console.log('Connection to ROSBridge was closed!')
              clearInterval(this.pubInterval)
          })
      },
      publish: function () {
          let topic = new ROSLIB.Topic({
              ros: this.ros,
              name: '/cmd_vel',
              messageType: 'geometry_msgs/Twist'
          })
          let message = new ROSLIB.Message({
              linear: { x: this.joystick.vertical, y: 0, z: 0, },
              angular: { x: 0, y: 0, z: this.joystick.horizontal, },
          })
          topic.publish(message)
      },
      disconnect: function () {
          this.ros.close()
      },
      sendCommand: function () {
          let topic = new ROSLIB.Topic({
              ros: this.ros,
              name: '/cmd_vel',
              messageType: 'geometry_msgs/Twist'
          })
          let message = new ROSLIB.Message({
              linear: { x: 1, y: 0, z: 0, },
              angular: { x: 0, y: 0, z: 0.5, },
          })
          topic.publish(message)
      },
      startDrag() {
          this.dragging = true
          this.x = this.y = 0
      },
      stopDrag() {
          this.dragging = false
          this.x = this.y = 'no'
          this.dragCircleStyle.display = 'none'
          this.resetJoystickVals()
      },
      doDrag(event) {
          if (this.dragging) {
              this.x = event.offsetX
              this.y = event.offsetY
              let ref = document.getElementById('dragstartzone')
              this.dragCircleStyle.display = 'inline-block'

              let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
              let maxTop = minTop + 200
              let top = this.y + minTop
              this.dragCircleStyle.top = `${top}px`

              let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
              let maxLeft = minLeft + 200
              let left = this.x + minLeft
              this.dragCircleStyle.left = `${left}px`

              this.setJoystickVals()
          }
      },
      setJoystickVals() {
          this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
          this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
      },
      resetJoystickVals() {
          this.joystick.vertical = 0
          this.joystick.horizontal = 0
      },
      setCamera: function() {
        let without_wss = this.rosbridge_address.split('wss://')[1]
        console.log(without_wss)
        let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
        console.log(domain)
        let host = domain + '/cameras'
        let viewer = new MJPEGCANVAS.Viewer({
            divID: 'divCamera',
            host: host,
            width: 320,
            height: 240,
            topic: '/camera/rgb/image_raw',
            ssl: true,
        })
      }
  },
  mounted() {
      // page is ready
      window.addEventListener('mouseup', this.stopDrag)
  },
})