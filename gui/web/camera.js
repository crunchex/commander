function initCanvas(wsString) {
	// Show loading notice
	var canvas = document.getElementById('video-canvas');
	var ctx = canvas.getContext('2d');
	ctx.fillStyle = '#444';
	ctx.fillText('Loading...', canvas.width/2-30, canvas.height/3);

	// Setup the WebSocket connection and start the player
	var client = new WebSocket(wsString);
	var player = new jsmpeg(client, {canvas:canvas});
};