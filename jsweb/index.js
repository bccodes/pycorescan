// Connecting to ROS
var ros = new ROSLIB.Ros();
ros.connect('ws://localhost:9090');

// Get the html status object
var statuslabel = document.getElementById("status");

var reconnect_timer = setInterval(reconnect_callback, 3000);
var attempts = 5;

function reconnect_callback() {
	if (ros.isConnected) {
		statuslabel.innerHTML = 'connected'; 
		statuslabel.style.color = 'green';
	} else if (attempts > 0) {
		ros.connect('ws://localhost:9090');
		statuslabel.innerHTML = 'attempting to reconnect...'; 
		statuslabel.style.color = 'orange';
		attempts -= 1;
	} else {
		statuslabel.innerHTML = 'lost connection, please refresh'; 
		statuslabel.style.color = 'red';
		clearInterval(reconnect_timer);
	}
}

var capture_topic = new ROSLIB.Topic({
	ros: ros,
	name: '/capture',  // Replace with your ROS topic name
	messageType: 'pcs_interfaces/CaptureRequest'  // Replace with your message type
});

document.getElementById('publishBtn').addEventListener('click', function() {
	var textInput = document.getElementById('messageInput').value;
	if (textInput.trim() !== "") {
		var message = new ROSLIB.Message({
			segment_id: textInput
		});
		capture_topic.publish(message);
		console.log('Message published: ' + message.segment_id);
	} else {
		console.log('No message entered.');
	}
});

// Update the image frame
var jpegTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});

jpegTopic.subscribe(function(message) {
	var imagedata = "data:image/jpeg;base64," + message.data;
	console.log(message.format);
	document.getElementById('image').setAttribute('src', imagedata);
});

