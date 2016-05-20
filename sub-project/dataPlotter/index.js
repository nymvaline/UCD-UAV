var express = require('express');
var app = express();
var server = require('http').Server(app);
var io = require('socket.io')(server);

server.listen(8081);

// Forward files in /static to client
app.use("/", express.static(__dirname + '/static'));

// Send out message in data every 1 s
setInterval(function () {
	io.sockets.emit('data', Math.floor(Math.random() * 10) + 1);
}, 1000);

// Greet client
io.on('connection', function (socket) {
  socket.emit('lol', 'You are connected');
});