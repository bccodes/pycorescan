// ROS2 STATUS
var ros = new ROSLIB.Ros();
ros.connect('ws://localhost:9090');

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


// UPDATE SETTINGS
var update_settings_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/update_settings',  // Replace with your new ROS topic name
    messageType: 'pcs_interfaces/UpdateSettings'  // Replace with your new message type
});

// Add event listener to the button
document.getElementById('settingsBtn').addEventListener('click', function() {
    // Get values from the three input fields
    var inputE1 = document.getElementById('inputE1').value;
    var inputE2 = document.getElementById('inputE2').value;
    var inputPrefix = document.getElementById('inputPrefix').value;

	// Create a ROS message with e1, e2, and prefix
	var message = new ROSLIB.Message({
		exposure_ring: inputE1 * 1,
		exposure_uv: inputE2 * 1,
		core_id: inputPrefix
	});

	// Publish the message
	update_settings_topic.publish(message);
	console.log('Message published:', message);
});


// CAPTURE REQUEST
var capture_topic = new ROSLIB.Topic({
	ros: ros,
	name: '/capture',  // replace with your ros topic name
	messagetype: 'pcs_interfaces/CaptureRequest'  // replace with your message type
});

document.getElementById('captureBtn').addEventListener('click', function() {
	var textInput = document.getElementById('captureInput').value;
	var message = new ROSLIB.Message();
	message.segment_id = textInput.trim();
	capture_topic.publish(message);
	console.log('Message published:', message);
});


// IMAGE PREVIEW
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


// FILE BROWSER
// back button
function backBtn() {
	var iframe = document.getElementById('iframe');
	iframe.contentWindow.history.back();
}

// scale images inside the file tree iframe
// Wait for the iframe to load
document.getElementById('iframe').onload = function() {
	var iframe = document.getElementById('iframe');
	var iframeDoc = iframe.contentDocument || iframe.contentWindow.document;

	// Add CSS to scale images inside the iframe
	var style = iframeDoc.createElement('style');
	style.innerHTML = `
		img {
			max-width: 100%;
			height: auto;
			display: block;
			margin: 0 auto;
		}
		body {
			margin: 0;
			padding: 0;
			overflow: hidden;
		}
		h1, hr { 
            display: none; /* Hide the directory header and line */
        }
	`;
	iframeDoc.head.appendChild(style);

	// Get the height of the iframe content
	// var contentHeight = iframeDoc.body.scrollHeight;
	var contentHeight = Math.max(iframeDoc.body.scrollHeight, iframeDoc.documentElement.scrollHeight);

	// Set the iframe height to the content height
	iframe.style.height = contentHeight + 'px';
};

// ROS2 LOGGING
// Define the ROS topic for log messages
var logTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rosout',  // ROS 2 log topic
    messageType: 'rcl_interfaces/Log'  // Update this to the correct message type if necessary
});

// Subscribe to the log topic
logTopic.subscribe(function(message) {
    // Get the message string
    var logMessage = message.msg;  // Adjust according to your message structure

    // Create a new div element for the log message
    var messageDiv = document.createElement('div');
    messageDiv.textContent = logMessage;  // Set the message text

    // Prepend the new message to the log container
    var logContainer = document.getElementById('logContainer');
    logContainer.prepend(messageDiv);  // Prepend so the most recent messages appear at the top

    // Scroll to the top to show the latest message
    logContainer.scrollTop = 0;

	// Optional: Limit the number of messages displayed to avoid overflow
	while (logContainer.childElementCount > 100) {  // Adjust the number as needed
		logContainer.removeChild(logContainer.lastChild);
	}
});
