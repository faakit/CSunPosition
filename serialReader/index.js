var express = require("express"),
  app = express({}),
  server = require("http").Server(app),
  io = require("socket.io")(server, {
    cors: {
      origin: "*",
      methods: ["GET", "POST"]
    }
  }),
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
  autoOpen: false,
});

arduinoPort.open(function (err) {
  if (err) {
    return console.log("Error opening port: ", err.message);
  }

  arduinoPort.write("SerialReader connected", function (err) {
    if (err) {
      return console.log("Error on write: ", err.message);
    }
    console.log("message written");
  });
});

arduinoPort.on("data", function (data) {
  console.log("Data:", data);
  io.emit("serialdata", { data: data });
});
