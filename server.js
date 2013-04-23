var net = require('net')
var express = require('express')
var WebSocketServer = require('ws').Server
var app = express()
var wss = new WebSocketServer({port:9322})

var hardwareConnection;

app.configure(function(){
  app.use(express.errorHandler({ showStack: true, dumpExceptions: true }));
  app.use(express.static(__dirname+'/public'));
})
app.listen( 9321 );

wss.on('connection', function(ws) {
  console.log("WebSocket connection")
  ws.on('message', function(message) {
    if (hardwareConnection)
      hardwareConnection.write(message + '\n')
  })

  ws.on('error', function(err) {
    console.log('ERROR', err)
  })

  ws.on('end', function() {
    console.log("WebSocket disconnected")
  })
})


var hardwareServer = net.createServer(function(c) { 
  hardwareConnection = c;
  console.log('hardware connected');
  c.on('end', function() {
    hardwareConnection = null;
    console.log('hardware disconnected');
  });
})
hardwareServer.listen(5204);

