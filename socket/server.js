const { createServer } = require("http");
const { Server } = require("socket.io");
const express = require("express");

const app = express();
const httpServer = createServer(app);
const io = new Server(httpServer, { /* options */ });

let sampler = 0;

const PORT = 3000;

function exec () {
  io.on("connection", (socket) => {
    socket.on("ID", id => {
      console.log("CONNECTION: ", id)
    })

    socket.on("gyro", (data)=> {
      let coords = Buffer.from(data).toString().split(",")
  
      coords[2] = coords[2].split("\r")[0];
  
      const [pitch, roll, yaw ] = coords;
      const test = [parseFloat(pitch), parseFloat(roll), parseFloat(yaw)];
  
      const turret = [
        (((test[0]+90)/640)+0.02).toPrecision(5), 
        (((test[1]+90)/640)+0.02).toPrecision(5),
        (test[2]).toPrecision(5)
      ];
  
      // console.log(coords);
      // console.log(test);
      io.emit("gyro-raw", test[0], test[1],test[2]);
      io.emit("gyro-output", turret[0], turret[1], turret[2]);
    })
  
    socket.on("mouse-pos", (pitch, yaw)=> {
      io.emit("gyro-output", 0,pitch, yaw);
    })
  });
  
  require("./router")(app);

  httpServer.listen(PORT, ()=>{
    const ip = require("./helper").getHostIp();
    console.log(`http://${ip}:${PORT}`)
  });
}
// exec();

module.exports = exec;