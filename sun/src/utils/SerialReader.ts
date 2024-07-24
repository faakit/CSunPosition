const { SerialPort } = require("serialport");
import { ReadlineParser as Readline } from "@serialport/parser-readline";

// mudem isso aqui pra porta e o baudRate que vcs estão usando aí
export const port = new SerialPort({
  path: "/dev/ttyACM0",
  baudRate: 115200,
});

port.write("SerialReader connected", function (err) {
  if (err) {
    return console.log("Error on write: ", err.message);
  }
  console.log("message written");
});

// Read data that is available but keep the stream in "paused mode"
port.on('data', function (data) {
  console.log('Data:', data)
})
