const {io} = require("socket.io-client");
const { spawnSync, execSync } = require("child_process");

const socket = io("http://192.168.0.14:3000");

let child;

socket.on("connect", ()=> {
    console.log("connected")
})

async ()=>{

    execSync('gcc gyro.cpp -lstdc++ -lwiringPi -lpthread -o exec', {
        cwd: "../driver"
    }, (err, stdout, stderr) => {
        if(!err) {
            // change this, something better
            console.log('subprocess stdout: ', Buffer.from(stdout).toString())
            console.log('subprocess stderr: ', Buffer.from(stderr).toString())
            // resolve()
            
        } else {
            // rejects("Subprocess error: ", err)
        }
    })

    child = spawnSync('./a', [] ,{
        stdio: ['ignore', 'pipe', process.stderr],
        cwd: "../driver"
    })

    child.stdout.on("data", (data)=>{
        socket.emit("gyro", data)
    })
}
