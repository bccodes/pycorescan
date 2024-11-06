// ROS2 STATUS
var ros = new ROSLIB.Ros();
ros.connect('ws://localhost:9090');

function updateROSStatus() {
    if (ros.isConnected) {
        rosStatusBar.classList.remove('bg-secondary');
        rosStatusBar.classList.remove('bg-danger');
        rosStatusBar.classList.remove('bg-warning');
		rosStatusLabel.innerText = 'ROS2 Connected'
        rosStatusBar.classList.add('bg-success');
    } else if (attempts > 0) {
        rosStatusBar.classList.remove('bg-secondary');
        rosStatusBar.classList.remove('bg-danger');
        rosStatusBar.classList.remove('bg-danger');
		rosStatusLabel.innerText = 'Attempting To Reconnect ROS2...'
        rosStatusBar.classList.add('bg-warning');
		attempts = attempts - 1;
		ros.connect('ws://localhost:9090');
    } else {
        rosStatusBar.classList.remove('bg-secondary');
        rosStatusBar.classList.remove('bg-danger');
        rosStatusBar.classList.remove('bg-warning');
		rosStatusLabel.innerText = 'Lost Connection To ROS2, Please Refresh'
        rosStatusBar.classList.add('bg-danger');
	}
}

var rosStatusBar = document.getElementById('ros-status-bar');
var rosStatusLabel = document.getElementById('ros-status-label');
var attempts = 10;
setInterval(updateROSStatus, 1500);

var relayStatusTopic = new ROSLIB.Topic({
	ros: ros,
	name: '/has_relays',
	messageType: 'std_msgs/Bool'
});
relayStatusTopic.subscribe(function(message) {
	var relayStatusBar = document.getElementById('relay-status-bar');
	if (message.data === false) {
		relayStatusBar.style.display = 'block'; // Show the element
	} else {
		relayStatusBar.style.display = 'none';  // Hide the element
	}
});
var barcodeStatusTopic = new ROSLIB.Topic({
	ros: ros,
	name: '/has_barcode_scanner',
	messageType: 'std_msgs/Bool'
});
barcodeStatusTopic.subscribe(function(message) {
	var statusBar = document.getElementById('barcode-status-bar');
	if (message.data === false) {
		statusBar.style.display = 'block'; // Show the element
	} else {
		statusBar.style.display = 'none';  // Hide the element
	}
});

var usbStatusTopic = new ROSLIB.Topic({
	ros: ros,
	name: '/has_storage',
	messageType: 'std_msgs/Bool'
});
usbStatusTopic.subscribe(function(message) {
	var statusBar = document.getElementById('usb-status-bar');
	if (message.data === false) {
		statusBar.style.display = 'block'; // Show the element
	} else {
		statusBar.style.display = 'none';  // Hide the element
	}
});


// IMAGE PREVIEW
var leftTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/my_camera/camera_left/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});
var rightTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/my_camera/camera_right/image_raw/compressed',
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
var update_prefix_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/set_prefix',  // Replace with your new ROS topic name
    messageType: 'std_msgs/String'  // Replace with your new message type
});
document.getElementById('prefixBtn').addEventListener('click', function() {
    var inputPrefix = document.getElementById('inputPrefix').value;
	var message = new ROSLIB.Message({
		data: inputPrefix.trim(),
	});
	update_prefix_topic.publish(message);
	console.log('Message published:', message);
});
var update_exp1_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/set_exposure_ring',  // Replace with your new ROS topic name
    messageType: 'std_msgs/Float32'  // Replace with your new message type
});
document.getElementById('exp1Btn').addEventListener('click', function() {
    var inputE1 = document.getElementById('inputE1').value;
	var message = new ROSLIB.Message({
		data: inputE1 * 1,
	});
	update_exp1_topic.publish(message);
	console.log('Message published:', message);
});
var update_exp2_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/set_exposure_uv',  // Replace with your new ROS topic name
    messageType: 'std_msgs/Float32'  // Replace with your new message type
});
document.getElementById('exp2Btn').addEventListener('click', function() {
    var inputE2 = document.getElementById('inputE2').value;
	var message = new ROSLIB.Message({
		data: inputE2 * 1.0,
	});
	update_exp2_topic.publish(message);
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
function styleIframe() {
    var iframeDoc = iframe.contentDocument || iframe.contentWindow.document;
//
// 	function adjustIframeHeight() {
// 		var contentHeight = document.getElementById('preview_frame').scrollHeight;
// 		// var contentHeight = Math.max(iframeDoc.body.scrollHeight, iframeDoc.documentElement.scrollHeight);
// 		iframe.style.height = contentHeight + 'px';
// 	}
//
// 	// Function to apply styles to images in the iframe
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
			<!-- overflow: hidden; -->
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
//
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
			var str = item.textContent
			if (str.includes('.png') || str.includes('.jpg')) {
				item.innerHTML = `<i class="fas fa-image"></i> ${item.innerHTML}`;
			} else if (str.charAt(str.length - 1) == '/') {
				item.innerHTML = `<i class="fas fa-folder"></i> ${item.innerHTML}`;
			} else {
				item.innerHTML = `<i class="fas fa-file"></i> ${item.innerHTML}`;
			}
		});
	}	
//
//     // Initial styling application
    applyStyles();
    applyStylesToList();
//
//     // Adjust the height initially when the iframe loads
//     adjustIframeHeight();
};

// window.addEventListener('load', function() {
// 	styleIframe(); // Resize the iframe on back navigation
// // 	refreshBtn();
// });
// var refreshTimer = setInterval(refreshBtn, 1000);


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
