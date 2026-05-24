/**
 * =============================================================================
 * File: app.js
 * Role: The Brain of the UI (Frontend Logic & ROS 2 Bridge)
 *
 * Description:
 * This script acts as the master control layer for the web interface. It 
 * manages the WebSocket connection to the robot and translates human inputs 
 * (clicks, touches, slider drags) into raw ROS 2 messages.
 *
 * What it does:
 * 1. Nervous System: Establishes a live `roslibjs` WebSocket connection to 
 * the robot and manages connection states and video feed rendering.
 * 2. Teleoperation & Safety: Handles continuous D-Pad inputs for manual 
 * movement and manages the E-Stop logic to instantly cancel hardware 
 * velocities and autonomous navigation.
 * 3. Live Tuning: Listens to the Engineering Drawer sliders and publishes 
 * live kinematic adjustments (speed, step duration, stride pivot) to the 
 * Python backend.
 * 4. RVIZ-Lite (Mapping): Uses `ros2djs` to render the live SLAM map, plot 
 * the robot's real-time odometry, and handle mouse panning/zooming on 
 * the canvas.
 * 5. Autonomous Dispatch: Converts screen pixel clicks on the map into exact 
 * ROS world coordinates, deploying Nav2 goal poses to drive the robot.
 * =============================================================================
 */

// --- STATE MANAGEMENT ---
let robotMode = 'walk'; 
let driveState = 'stop'; 

// ==========================================
// ROS2 CONNECTION (THE NERVOUS SYSTEM)
// ==========================================
const IP_ADDRESS = window.location.hostname;

const videoUrl = `http://${IP_ADDRESS}:8080/stream?topic=/camera/image_raw&type=mjpeg`;

const ros = new ROSLIB.Ros({ url : `ws://${IP_ADDRESS}:9091` });


// ROS Status DOM Elements
const statusDot = document.getElementById('ros-status-dot');
const statusText = document.getElementById('ros-status-text');
const videoStream = document.getElementById('video-stream');

ros.on('connection', () => {
    console.log('[SYS] Connected to ROS 2 websocket server.');
    statusDot.classList.replace('offline', 'online');
    statusText.innerText = 'ONLINE';
    videoStream.src = videoUrl; // Start pulling the camera feed
});

ros.on('error', (error) => {
    console.error('[SYS] Error connecting to websocket server: ', error);
    statusDot.classList.replace('online', 'offline');
    statusText.innerText = 'ERROR';
});

ros.on('close', () => {
    console.log('[SYS] Connection to websocket server closed.');
    statusDot.classList.replace('online', 'offline');
    statusText.innerText = 'DISCONNECTED';
    videoStream.src = ""; // Cut the video feed
});

// --- DEFINE ROS PUBLISHERS ---
const cmdPublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/otto_command',
    messageType : 'std_msgs/String'
});

// Helper function to send commands
function sendOttoCommand(commandString) {
    const msg = new ROSLIB.Message({ data: commandString });
    cmdPublisher.publish(msg);
    console.log(`[TX] /otto_command -> '${commandString}'`);
}

// --- DEFINE NAV2 ACTION CLIENT ---
const navClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose'
});

// ==========================================
// DOM ELEMENTS & UI LOGIC
// ==========================================
const masterToggle = document.getElementById('master-toggle');
const smallCameraFeed = document.getElementById('small-camera-feed');
const cameraFeed = document.getElementById('camera-feed');
const smallVideoStream = document.getElementById('small-video-stream');
const mapFeed = document.getElementById('map-feed');

const btnSettings = document.getElementById('btn-settings');
const drawer = document.getElementById('settings-drawer');
const rollSettings = document.getElementById('roll-settings');
const walkSettings = document.getElementById('walk-settings');
const dBtns = document.querySelectorAll('.d-btn');
const btnStop = document.getElementById('btn-stop');

// --- MASTER TOGGLE (WALK / ROLL) ---
masterToggle.addEventListener('change', (e) => {
    robotMode = e.target.checked ? 'roll' : 'walk';
    
    // SEND MODE SWITCH TO PYTHON SCRIPT
    sendOttoCommand(`switch_${robotMode}`);

    if (robotMode === 'walk') {
        // --- WALK MODE ---
        cameraFeed.classList.remove('hidden');
        mapFeed.classList.add('hidden');
        smallCameraFeed.classList.add('hidden');
        
        rollSettings.classList.add('hidden');
        walkSettings.classList.remove('hidden');

        // Route video to the BIG screen, kill the small one
        videoStream.src = videoUrl;
        smallVideoStream.src = ""; 
    } else {
        // --- ROLL MODE ---
        cameraFeed.classList.add('hidden');
        mapFeed.classList.remove('hidden');
        smallCameraFeed.classList.remove('hidden');
        
        walkSettings.classList.add('hidden');
        rollSettings.classList.remove('hidden');

        // Route video to the SMALL screen, kill the big one
        smallVideoStream.src = videoUrl;
        videoStream.src = ""; 
    }
});

// --- ENGINEERING DRAWER (SPEED TUNING) ---
btnSettings.addEventListener('click', drawer_logic);

function drawer_logic() {
    if (drawer.classList.contains('hidden')) {
        drawer.classList.remove('hidden');
    } else {
        drawer.classList.add('hidden');
    }
}

// When sliders change, send the new speed data as a JSON string
document.querySelectorAll('input[type=range]').forEach(slider => {
    slider.addEventListener('change', (e) => { // 'change' fires only when thumb is released
        const id = e.target.id;
        const val = parseFloat(e.target.value).toFixed(2);
        
        // Update UI
        const displaySpan = document.getElementById(`val-${id.split('-')[1]}`);
        if(displaySpan) displaySpan.innerText = val;

        // Create a JSON string to parse easily in python
        const tuneMsg = `tune_${id.split('-')[1]}_${val}`;
        sendOttoCommand(tuneMsg); // e.g., "tune_linear_2.5"
    });
});

// Update text live while dragging (without spamming ROS)
document.querySelectorAll('input[type=range]').forEach(slider => {
    slider.addEventListener('input', (e) => {
        const displaySpan = document.getElementById(`val-${e.target.id.split('-')[1]}`);
        if(displaySpan) displaySpan.innerText = parseFloat(e.target.value).toFixed(2);
    });
});

// --- DEAD-MAN'S D-PAD LOGIC ---
dBtns.forEach(btn => {
    btn.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        btn.classList.add('active');
        
        const direction = btn.getAttribute('data-dir');
        driveState = direction;
        
        if (navigator.vibrate) navigator.vibrate(40);
        
        // "roll_forward" or "walk_left"
        sendOttoCommand(`${robotMode}_${direction}`);
    });

    const releaseThumb = (e) => {
        e.preventDefault();
        if(btn.classList.contains('active')) {
            btn.classList.remove('active');
            driveState = 'stop';
            sendOttoCommand('stop');
        }
    };

    btn.addEventListener('pointerup', releaseThumb);
    btn.addEventListener('pointerleave', releaseThumb);
    btn.addEventListener('pointercancel', releaseThumb);
});

// --- E-STOP ---
btnStop.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (navigator.vibrate) navigator.vibrate([100, 50, 100]); // Aggressive haptic pattern
    
    console.warn('[E-STOP] ACTIVATED. Halting all subsystems.');

    // Kill Manual Physics (Walking & Teleop)
    sendOttoCommand('stop');

    // Kill Autonomous Navigation (Nav2)
    // Calling cancel() without a specific goal ID automatically aborts everything
    navClient.cancel();

    // Clear the UI Visuals
    // Hide the red destination marker so the operator knows the goal is wiped
    if (typeof goalMarker !== 'undefined') {
        goalMarker.visible = false;
    }

    // Brutalist UI Flash
    // Flash the button inverted colors for 200ms to confirm the strike
    btnStop.style.backgroundColor = '#000000';
    btnStop.style.color = '#ff3333';
    setTimeout(() => {
        btnStop.style.backgroundColor = ''; 
        btnStop.style.color = '';
    }, 200);
});


// ==========================================
//  AUTONOMOUS MAPPING & NAVIGATION
// ==========================================

// Setup the Map Viewer (The Canvas)
const mapViewer = new ROS2D.Viewer({
    divID: 'map-feed',
    width: window.innerWidth * 0.6,  
    height: window.innerHeight * 0.7,
    background: '#111111'            
});

// Setup the Map Client (Pulls data from /map topic)
const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: mapViewer.scene,
    continuous: true // True = Live updates (SLAM). False = Static map (AMCL)
});

// Center the map once it loads (USING YOUR EXACT WORKING MATH)
gridClient.on('change', () => {
    mapViewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    mapViewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
});
// ==========================================
// RVIZ-LITE: LIVE ROBOT TRACKING
// ==========================================
// The scene is scaled in meters! 
// Use raw createjs Shape for a circle
const robotMarker = new createjs.Shape();
const diameter = 0.2; // maintaining the original diameter in meters
const radius = diameter / 2; // radius in meters

robotMarker.graphics.setStrokeStyle(0.03); // maintaining stroke width in meters
robotMarker.graphics.beginStroke(createjs.Graphics.getRGB(0, 0, 0, 1)); // keep black stroke
robotMarker.graphics.beginFill(createjs.Graphics.getRGB(0, 0, 255, 1)); // blue fill, for the blue cercle
robotMarker.graphics.drawCircle(0, 0, radius); // draw circle with correct radius in meters
mapViewer.scene.addChild(robotMarker);

// ==========================================
// THE GOAL MARKER
// ==========================================
const goalMarker = new createjs.Shape();
// Draw a bright red circle, 0.2 meters in radius
goalMarker.graphics.beginFill(createjs.Graphics.getRGB(255, 50, 50, 0.8)).drawCircle(0, 0, 0.05);
goalMarker.visible = false; // Hidden until you click
mapViewer.scene.addChild(goalMarker);

// A simple loop to check if OTTO reached the goal
setInterval(() => {
    if (!goalMarker.visible) return;
    
    // Calculate distance between OTTO and the Goal using Pythagorean theorem
    const dx = robotMarker.x - goalMarker.x;
    const dy = robotMarker.y - goalMarker.y;
    const distance = Math.sqrt(dx*dx + dy*dy);
    
    if (distance < 0.25) { // If he is within 25 centimeters
        goalMarker.visible = false;
        console.log("[NAV] Goal Reached! Marker cleared.");
    }
}, 500); // Check twice a second

const odomListener = new ROSLIB.Topic({
    ros: ros,
    name: '/diff_drive_controller/odom',
    messageType: 'nav_msgs/msg/Odometry'
});

odomListener.subscribe((msg) => {
    // Because the map scales to meters, we just use raw ROS coordinates!
    robotMarker.x = msg.pose.pose.position.x;
    robotMarker.y = -msg.pose.pose.position.y; 

    // Convert Quaternion to Euler Yaw
    const q = msg.pose.pose.orientation;
    const yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    robotMarker.rotation = -yaw * (180 / Math.PI);
});
// ==========================================
// RVIZ-LITE: PAN, ZOOM & GOAL DISPATCH
// ==========================================
const goalPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/msg/PoseStamped'
});

// Delay grabbing the canvas slightly to ensure ros2djs has injected it
setTimeout(() => {
    const canvas = document.querySelector('#map-feed canvas');
    if (!canvas) return;

    // --- MOUSE WHEEL ZOOM ---
    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const zoom = e.deltaY < 0 ? 1.1 : 0.9;
        mapViewer.scene.scaleX *= zoom;
        mapViewer.scene.scaleY *= zoom;
    });

    // --- PAN VS CLICK LOGIC ---
    let isDragging = false;
    let startX, startY;     // Used for Canvas panning
    let clickStartX, clickStartY; // Used for tap-distance measurement

    canvas.addEventListener('pointerdown', (e) => {
        isDragging = true;
        
        // Record starting positions
        startX = e.clientX - mapViewer.scene.x;
        startY = e.clientY - mapViewer.scene.y;
        
        clickStartX = e.clientX;
        clickStartY = e.clientY;
    });

    canvas.addEventListener('pointermove', (e) => {
        if (isDragging) {
            // Pan the map
            mapViewer.scene.x = e.clientX - startX;
            mapViewer.scene.y = e.clientY - startY;
        }
    });

    canvas.addEventListener('pointerup', (e) => {
        isDragging = false;

        // Calculate how many pixels the mouse/finger moved
        const distanceMoved = Math.hypot(e.clientX - clickStartX, e.clientY - clickStartY);

        // If it moved less than 10 pixels, it was a TAP, not a DRAG!
        if (distanceMoved < 10) {
            if (robotMode === 'walk') {
                console.warn("[NAV] Cannot dispatch goals while in Walk Mode.");
                return; 
            }

            // Convert pixel click to ROS meter coordinates (YOUR WORKING MATH)
            // Note: We use stageX and stageY from the EaselJS event, which we can get 
            // by converting the raw clientX/Y coordinates.
            const rect = canvas.getBoundingClientRect();
            const stageX = e.clientX - rect.left;
            const stageY = e.clientY - rect.top;

            const displayPos = mapViewer.scene.globalToRos(stageX, stageY);
            
            console.log(`[AUTONOMY] Target Set: X=${displayPos.x.toFixed(2)}, Y=${displayPos.y.toFixed(2)}`);

            // Display target waypoint
            goalMarker.x = displayPos.x;
            goalMarker.y = displayPos.y;
            goalMarker.visible = true;

            const goalMessage = new ROSLIB.Message({
                header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
                pose: {
                    position: { x: displayPos.x, y: displayPos.y, z: 0.0 },
                    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } 
                }
            });

            goalPublisher.publish(goalMessage);
            if (navigator.vibrate) navigator.vibrate([50]); // Haptic feedback
        }
    });

    canvas.addEventListener('pointerleave', () => {
        isDragging = false;
    });

}, 1000); // 1-second delay ensures Canvas exists


