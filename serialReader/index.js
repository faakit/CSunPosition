var express = require("express"),
  app = express(),
  server = require("http").Server(app),
  io = require("socket.io")(server),
  port = 8888;

//Server start
server.listen(port, () => console.log("on port" + port));

//user server
app.use(express.static(__dirname + "/public"));

io.on("connection", onConnection);

var connectedSocket = null;
function onConnection(socket) {
  connectedSocket = socket;
}

const { SerialPort } = require("serialport");

var arduinoPort = new SerialPort({
  path: "/dev/ttyACM0",
  baudRate: 115200,
});

// mudem isso aqui pra porta e o baudRate que vcs estão usando aí
arduinoPort.write("SerialReader connected", function (err) {
  if (err) {
    return console.log("Error on write: ", err.message);
  }
  console.log("message written");
});

// Read data that is available but keep the stream in "paused mode"
arduinoPort.on("data", function (data) {
  console.log("Data:", data);
  io.emit('serialdata', { data: data });
});
