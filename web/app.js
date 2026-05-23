// --- STATE MANAGEMENT ---
let robotMode = 'walk'; 
let driveState = 'stop'; 

// ==========================================
// ROS 2 CONNECTION (THE NERVOUS SYSTEM)
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


// ==========================================
// DOM ELEMENTS & UI LOGIC
// ==========================================
const masterToggle = document.getElementById('master-toggle');
const viewportToggleWrapper = document.getElementById('viewport-toggle-wrapper');
const viewportToggle = document.getElementById('viewport-toggle');
const cameraFeed = document.getElementById('camera-feed');
const mapFeed = document.getElementById('map-feed');

const btnSettings = document.getElementById('btn-settings');
const drawer = document.getElementById('settings-drawer');
const rollSettings = document.getElementById('roll-settings');
const walkSettings = document.getElementById('walk-settings');
const dBtns = document.querySelectorAll('.d-btn');
const btnStop = document.getElementById('btn-stop');

// --- 1. MASTER TOGGLE (WALK / ROLL) ---
masterToggle.addEventListener('change', (e) => {
    robotMode = e.target.checked ? 'roll' : 'walk';
    
    // SEND MODE SWITCH TO PYTHON SCRIPT
    sendOttoCommand(`switch_${robotMode}`);

    if (robotMode === 'walk') {
        viewportToggleWrapper.classList.add('hidden');
        cameraFeed.classList.remove('hidden');
        mapFeed.classList.add('hidden');
        rollSettings.classList.add('hidden');
        walkSettings.classList.remove('hidden');
    } else {
        viewportToggleWrapper.classList.remove('hidden');
        walkSettings.classList.add('hidden');
        rollSettings.classList.remove('hidden');
        viewportToggle.dispatchEvent(new Event('change')); 
    }
});

// --- 2. VIEWPORT TOGGLE (VISION / MAP) ---
viewportToggle.addEventListener('change', (e) => {
    if (robotMode === 'walk') return; 
    const isMap = e.target.checked;
    if (isMap) {
        cameraFeed.classList.add('hidden');
        mapFeed.classList.remove('hidden');
    } else {
        mapFeed.classList.add('hidden');
        cameraFeed.classList.remove('hidden');
    }
});

// --- 3. ENGINEERING DRAWER (SPEED TUNING) ---
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

// --- 4. DEAD-MAN'S D-PAD LOGIC ---
dBtns.forEach(btn => {
    btn.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        btn.classList.add('active');
        
        const direction = btn.getAttribute('data-dir');
        driveState = direction;
        
        if (navigator.vibrate) navigator.vibrate(40);
        
        // e.g., "roll_forward" or "walk_left"
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

// --- 5. E-STOP ---
btnStop.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (navigator.vibrate) navigator.vibrate([100, 50, 100]); 
    sendOttoCommand('stop');
});


// ==========================================
// PHASE 3: AUTONOMOUS MAPPING & NAVIGATION
// ==========================================

// 1. Setup the Map Viewer (The Canvas)
const mapViewer = new ROS2D.Viewer({
    divID: 'map-feed',
    width: window.innerWidth * 0.6,
    height: window.innerHeight * 0.7,
    background: '#111111'
});

// 2. Setup the Map Client (Pulls data from /map topic)
const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: mapViewer.scene,
    continuous: true
});

gridClient.on('change', () => {
    mapViewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    mapViewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
});

// ==========================================
// NEW: LIVE ROBOT TRACKING
// ==========================================
// Create a stark, high-contrast tracking arrow for OTTO
const robotMarker = new ROS2D.NavigationArrow({
    size: 0.4,           // Size in meters on the map
    strokeSize: 0.05,
    fillColor: createjs.Graphics.getRGB(0, 255, 0, 1), // Bright green
    pulse: false
});
mapViewer.scene.addChild(robotMarker);

// Listen to OTTO's live odometry to move the marker
const odomListener = new ROSLIB.Topic({
    ros: ros,
    name: '/odom', // Uses diff_drive_controller odometry for smooth tracking
    messageType: 'nav_msgs/msg/Odometry'
});

odomListener.subscribe((msg) => {
    // 1. Update Position (Note: ros2djs inverts the Y axis for HTML canvas rendering)
    robotMarker.x = msg.pose.pose.position.x;
    robotMarker.y = -msg.pose.pose.position.y; 

    // 2. Update Rotation (Convert Quaternion to Euler Yaw)
    const q = msg.pose.pose.orientation;
    const yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    
    // Convert radians to degrees and invert for canvas rendering
    robotMarker.rotation = -yaw * (180 / Math.PI);
});


// ==========================================
// NEW: PAN, ZOOM, AND CLICK-TO-DRIVE
// ==========================================
const goalPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/msg/PoseStamped'
});

// --- ZOOM LOGIC (Mouse Wheel) ---
const canvasElement = document.querySelector('#map-feed canvas');
canvasElement.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1; // Scroll down = out, up = in
    mapViewer.scene.scaleX *= zoomFactor;
    mapViewer.scene.scaleY *= zoomFactor;
});

// --- PAN VS CLICK LOGIC (Touch & Mouse) ---
let isDraggingMap = false;
let startStageX, startStageY;
let startSceneX, startSceneY;

// When touch/mouse goes down, record the starting coordinates
mapViewer.scene.addEventListener('stagemousedown', (e) => {
    isDraggingMap = true;
    startStageX = e.stageX;
    startStageY = e.stageY;
    startSceneX = mapViewer.scene.x;
    startSceneY = mapViewer.scene.y;
});

// When dragging, move the entire scene relative to the start point
mapViewer.scene.addEventListener('stagemousemove', (e) => {
    if (isDraggingMap) {
        const dx = e.stageX - startStageX;
        const dy = e.stageY - startStageY;
        mapViewer.scene.x = startSceneX + dx;
        mapViewer.scene.y = startSceneY + dy;
    }
});

// When releasing touch/mouse, calculate if it was a Pan or a Tap
mapViewer.scene.addEventListener('stagemouseup', (e) => {
    isDraggingMap = false;
    
    // Calculate total pixels moved during the interaction
    const moveDistance = Math.hypot(e.stageX - startStageX, e.stageY - startStageY);
    
    // If we moved less than 10 pixels, it was an intentional Click/Tap!
    if (moveDistance < 10) {
        if (robotMode === 'walk') return; // Safety check

        const displayPos = mapViewer.scene.globalToRos(e.stageX, e.stageY);
        console.log(`[AUTONOMY] Target: X=${displayPos.x.toFixed(2)}, Y=${displayPos.y.toFixed(2)}`);

        const goalMessage = new ROSLIB.Message({
            header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
            pose: {
                position: { x: displayPos.x, y: displayPos.y, z: 0.0 },
                orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } 
            }
        });

        goalPublisher.publish(goalMessage);
        if (navigator.vibrate) navigator.vibrate(50);
    }
});