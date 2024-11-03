// ROS2 STATUS
var ros = new ROSLIB.Ros();
ros.connect('ws://localhost:9090');

var statuslabel = document.getElementById("status");

var reconnect_timer = setInterval(reconnect_callback, 3000);
var attempts = 5;
reconnect_callback();

function reconnect_callback() {
	if (ros.isConnected) {
		statuslabel.innerHTML = 'OK'; 
		statuslabel.style.color = 'green';
	} else if (attempts > 0) {
		ros.connect('ws://localhost:9090');
		statuslabel.innerHTML = 'attempting to connect...'; 
		statuslabel.style.color = 'orange';
		attempts -= 1;
	} else {
		statuslabel.innerHTML = 'lost connection, please refresh'; 
		statuslabel.style.color = 'red';
		clearInterval(reconnect_timer);
	}
}


// IMAGE PREVIEW
var leftTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});
var rightTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});

leftTopic.subscribe(function(message) {
	var imagedata = "data:image/jpeg;base64," + message.data;
	console.log(message.format);
	document.getElementById('left_img').setAttribute('src', imagedata);
});

rightTopic.subscribe(function(message) {
	var imagedata = "data:image/jpeg;base64," + message.data;
	console.log(message.format);
	document.getElementById('right_img').setAttribute('src', imagedata);
});


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


// FILE BROWSER
var iframe = document.getElementById('iframe');
// back button
function backBtn() {
	iframe.contentWindow.history.go(-1);
}
// refresh button
function refreshBtn() {
	iframe.contentWindow.location.reload();
}
function refreshIframe() {
    var iframeDoc = iframe.contentDocument || iframe.contentWindow.document;

	function adjustIframeHeight() {
		var contentHeight = Math.max(iframeDoc.body.scrollHeight, iframeDoc.documentElement.scrollHeight);
		iframe.style.height = contentHeight + 'px';
	}

	// Function to apply styles to images in the iframe
	function applyStyles() {
		var style = iframeDoc.createElement('style');
		style.innerHTML = `
			img {
			width: 100%;
			max-width: 100%;
			height: auto;
			display: block;
			margin: 0 auto;
			}
			body {
			margin: 0;
			padding: 0;
			overflow: hidden;
			font-family: inherit;
			}
			h1, hr {
			display: none; /* Hide the directory header and line */
			}
			i.fas {
			color: grey; /* Set icon color to grey */
			}
			`;
		iframeDoc.head.appendChild(style);
	}

	function applyStylesToList() {
		// Add Bootstrap CSS to the iframe's head if not already present
		if (!iframeDoc.getElementById('bootstrap-link')) {
			var bootstrapLink = iframeDoc.createElement('link');
			bootstrapLink.id = 'bootstrap-link';
			bootstrapLink.rel = 'stylesheet';
			bootstrapLink.href = 'https://maxcdn.bootstrapcdn.com/bootstrap/4.5.2/css/bootstrap.min.css';
			iframeDoc.head.appendChild(bootstrapLink);
		}
		// Add FontAwesome CSS to the iframe's head if not already present
		if (!iframeDoc.getElementById('fontawesome-link')) {
			var fontAwesomeLink = iframeDoc.createElement('link');
			fontAwesomeLink.id = 'fontawesome-link';
			fontAwesomeLink.rel = 'stylesheet';
			fontAwesomeLink.href = 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0-beta3/css/all.min.css';
			iframeDoc.head.appendChild(fontAwesomeLink);
		}

		// Apply icons to each list item based on whether it's a file or a folder
		var listItems = iframeDoc.querySelectorAll('li');
		listItems.forEach(function(item) {
			if (item.textContent.includes('.png') || item.textContent.includes('.jpg')) {// Assume it's an image
				item.innerHTML = `<i class="fas fa-image"></i> ${item.innerHTML}`;
			} else if (item.textContent.includes('.')) {// Assume it's a file
				item.innerHTML = `<i class="fas fa-file"></i> ${item.innerHTML}`;
			} else {// Otherwise, assume it's a folder
				item.innerHTML = `<i class="fas fa-folder"></i> ${item.innerHTML}`;
			}
		});
	}

    // Initial styling application
    applyStyles();
    applyStylesToList();
    
    // Adjust the height initially when the iframe loads
    adjustIframeHeight();
};

window.addEventListener('load', function() {
	refreshIframe(); // Resize the iframe on back navigation
	refreshBtn();
});
// var refreshTimer = setInterval(refreshBtn, 5000);


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
