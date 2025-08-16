    // 3D Trajectory Viewer Setup
    var odom3dCanvas = document.getElementById('odom3d');
    var odom3dRenderer, odom3dScene, odom3dCamera, odom3dLine, odom3dGeometry, odom3dMaterial;
    var odom3dPositions = [];
    var odom3dOrigin = null;
    if (odom3dCanvas && window.THREE) {
        const THREE = window.THREE;
        console.log('Initializing 3D plot...');
        odom3dRenderer = new THREE.WebGLRenderer({ canvas: odom3dCanvas, antialias: true });
        odom3dRenderer.setSize(odom3dCanvas.width, odom3dCanvas.height, false);
        odom3dScene = new THREE.Scene();
        odom3dScene.background = new THREE.Color(0x0d1324);
        odom3dCamera = new THREE.PerspectiveCamera(60, odom3dCanvas.width / odom3dCanvas.height, 0.01, 1000);
        // Camera angled view, looking at the origin
        odom3dCamera.position.set(10, -10, 10);   // Back & above at an angle
        odom3dCamera.up.set(0, 0, 1);             // Z axis is "up" (ROS convention)
        odom3dCamera.lookAt(10, 4, 0);             // Focus on world origin

        // Axes helper
        var axesHelper = new THREE.AxesHelper(2);
        odom3dScene.add(axesHelper);

        odom3dGeometry = new THREE.BufferGeometry();
        odom3dMaterial = new THREE.LineBasicMaterial({ color: 0x0072bd, linewidth: 15 });
        odom3dLine = new THREE.Line(odom3dGeometry, odom3dMaterial);
        odom3dScene.add(odom3dLine);

        function animate3d() {
            // Debug log for animation loop
            // console.log('Rendering 3D scene...');
            odom3dRenderer.render(odom3dScene, odom3dCamera);
            requestAnimationFrame(animate3d);
        }
        animate3d();
    }
window.onload = function() {

    // Loading bar logic
    var loadingOverlay = document.getElementById('loadingOverlay');
    var loadingBar = document.getElementById('loadingBar');
    var loadingDone = false;
    var loadingInterval = null;
    function showLoading() {
        if (loadingOverlay) {
            loadingOverlay.style.display = 'flex';
            loadingBar.style.width = '0%';
            loadingDone = false;
            var loadingProgress = 0;
            loadingInterval = setInterval(function() {
                if (loadingDone) { clearInterval(loadingInterval); return; }
                loadingProgress = Math.min(loadingProgress + Math.random() * 10, 90);
                if (loadingBar) loadingBar.style.width = loadingProgress + '%';
            }, 400);
        }
    }
    function hideLoading() {
        if (!loadingDone && loadingOverlay) {
            loadingDone = true;
            loadingBar.style.width = '100%';
            setTimeout(function() {
                loadingOverlay.style.display = 'none';
            }, 500);
            if (loadingInterval) clearInterval(loadingInterval);
        }
    }

    // Connect to ROS
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    var connBadge = document.getElementById('connBadge');
    var latestPred = document.getElementById('latestPred');
    var logDiv = document.getElementById('log');

    // Individual 2D plots: t vs x, t vs y, t vs z
    var canvasX = document.getElementById('plotX');
    var canvasY = document.getElementById('plotY');
    var canvasZ = document.getElementById('plotZ');
    var chartX = new Chart(canvasX, {
        type: 'line',
        data: {
            labels: [], // t
            datasets: [{
                label: 'Time vs x-Position',
                data: [], // x
                borderColor: '#0072bd', // Matlab blue
                backgroundColor: 'rgba(0,114,189,0.08)',
                pointRadius: 1,
                pointBackgroundColor: '#0072bd',
                borderWidth: 3,
                fill: false,
                tension: 0 // straight lines
            }]
        },
        options: {
            responsive: false,
            animation: false,
            plugins: {
                title: {
                    display: true,
                    text: 'Time vs X Position',
                    font: { size: 15, weight: 'bold' },
                    color: '#e6e8ec',
                    padding: { top: 0, bottom: 0 }
                },
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    title: { display: true, text: 'Time (s)', font: { size: 14, weight: 'bold' }, color: '#e6e8ec' },
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 },
                        callback: function(val, idx, values) { return Math.floor(val); }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                },
                y: {
                    title: { display: true, text: 'X Position', font: { size: 16, weight: 'bold' }, color: '#e6e8ec' },
                    min: -2,
                    max: 2,
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                }
            }
        }
    });
    var chartY = new Chart(canvasY, {
        type: 'line',
        data: {
            labels: [], // t
            datasets: [{
                label: 'Time vs y-Position',
                data: [], // y
                borderColor: '#d95319', // Matlab orange
                backgroundColor: 'rgba(217,83,25,0.08)',
                pointRadius: 1,
                pointBackgroundColor: '#d95319',
                borderWidth: 3,
                fill: false,
                tension: 0
            }]
        },
        options: {
            responsive: false,
            animation: false,
            plugins: {
                title: {
                    display: true,
                    text: 'Time vs Y Position',
                    font: { size: 15, weight: 'bold' },
                    color: '#e6e8ec',
                    padding: { top: 0, bottom: 0 }
                },
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    title: { display: true, text: 'Time (s)', font: { size: 14, weight: 'bold' }, color: '#e6e8ec' },
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 },
                        callback: function(val, idx, values) { return Math.floor(val); }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                },
                y: {
                    title: { display: true, text: 'Y Position', font: { size: 16, weight: 'bold' }, color: '#e6e8ec' },
                    min: -4,
                    max: 4,
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                }
            }
        }
    });
    var chartZ = new Chart(canvasZ, {
        type: 'line',
        data: {
            labels: [], // t
            datasets: [{
                label: 'Time vs z-Position',
                data: [], // z
                borderColor: '#edb120', // Matlab yellow
                backgroundColor: 'rgba(237,177,32,0.08)',
                pointRadius: 1,
                pointBackgroundColor: '#edb120',
                borderWidth: 3,
                fill: false,
                tension: 0
            }]
        },
        options: {
            responsive: false,
            animation: false,
            plugins: {
                title: {
                    display: true,
                    text: 'Time vs Z Position',
                    font: { size: 15, weight: 'bold' },
                    color: '#e6e8ec',
                    padding: { top: 0, bottom: 0 }
                },
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    title: { display: true, text: 'Time (s)', font: { size: 14, weight: 'bold' }, color: '#e6e8ec' },
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 },
                        callback: function(val, idx, values) { return Math.floor(val); }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                },
                y: {
                    title: { display: true, text: 'Z Position', font: { size: 16, weight: 'bold' }, color: '#e6e8ec' },
                    min: -3,
                    max: 3,
                    ticks: {
                        color: '#e6e8ec',
                        font: { size: 14 }
                    },
                    grid: {
                        color: '#2a355b',
                        lineWidth: 1
                    },
                    border: { color: '#e6e8ec', width: 2 }
                }
            }
        }
    });
    var startTime = null;
    var yOrigin = null;
    var zOrigin = null;

    ros.on('connection', function() { 
        console.log('Connected to rosbridge websocket'); 
        if (connBadge) connBadge.innerText = "Connected ✅"; 
    });

    ros.on('error', function(error) { 
        console.log('Error connecting:', error); 
        if (connBadge) connBadge.innerText = "Error ❌";
    });

    ros.on('close', function() { 
        console.log('Connection closed'); 
        if (connBadge) connBadge.innerText = "Closed ⚠️";
    });

    // Subscribe to /activity_prediction
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/activity_prediction',
        messageType: 'std_msgs/String'
    });

    listener.subscribe(function(message) {
        console.log('Prediction:', message.data);
        if (latestPred) latestPred.innerText = message.data;
        if (logDiv) {
            logDiv.innerText += message.data + "\n";
            logDiv.scrollTop = logDiv.scrollHeight; // auto scroll
        }
        hideLoading(); // Hide loading bar on first prediction
    });

    // Subscribe to /vins_estimator/odometry for t vs x plot
    var odomListener = new ROSLIB.Topic({
        ros: ros,
        name: '/vins_estimator/odometry',
        messageType: 'nav_msgs/Odometry'
    });
    console.log('Subscribing to /vins_estimator/odometry as nav_msgs/Odometry');

    var xOrigin = null;
    odomListener.subscribe(function(msg) {
    var pos = msg.pose.pose.position;
    var t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9;
    if (!startTime) startTime = t;
    var relT = t - startTime;
    hideLoading(); // Hide loading bar on first plot
        // X plot
        var labelsX = chartX.data.labels;
        var dataX = chartX.data.datasets[0].data;
        if (xOrigin === null) xOrigin = pos.x;
        labelsX.push(relT);
        dataX.push(pos.x - xOrigin);
        if (labelsX.length > 500) { labelsX.shift(); dataX.shift(); }
        chartX.update('none');
        // Y plot
        var labelsY = chartY.data.labels;
        var dataY = chartY.data.datasets[0].data;
        if (yOrigin === null) yOrigin = pos.y;
        labelsY.push(relT);
        dataY.push(pos.y - yOrigin);
        if (labelsY.length > 500) { labelsY.shift(); dataY.shift(); }
        chartY.update('none');
        // Z plot
        var labelsZ = chartZ.data.labels;
        var dataZ = chartZ.data.datasets[0].data;
        if (zOrigin === null) zOrigin = pos.z;
        labelsZ.push(relT);
        dataZ.push(pos.z - zOrigin);
        if (labelsZ.length > 500) { labelsZ.shift(); dataZ.shift(); }
        chartZ.update('none');

        // 3D trajectory
        if (odom3dCanvas && window.THREE) {
            const THREE = window.THREE;
            if (odom3dOrigin === null) {
                odom3dOrigin = { x: pos.x, y: pos.y, z: pos.z };
                console.log('3D plot origin set:', odom3dOrigin);
            }
            // Always plot relative to origin (centered)
            odom3dPositions.push(new THREE.Vector3(pos.x - odom3dOrigin.x, pos.y - odom3dOrigin.y, pos.z - odom3dOrigin.z));
            if (odom3dPositions.length > 2000) odom3dPositions.shift();
            var positionsArray = new Float32Array(odom3dPositions.length * 3);
            for (var i = 0; i < odom3dPositions.length; i++) {
                positionsArray[i * 3] = odom3dPositions[i].x;
                positionsArray[i * 3 + 1] = odom3dPositions[i].y;
                positionsArray[i * 3 + 2] = odom3dPositions[i].z;
            }
            odom3dGeometry.setAttribute('position', new THREE.BufferAttribute(positionsArray, 3));
            odom3dGeometry.setDrawRange(0, odom3dPositions.length);
            odom3dGeometry.computeBoundingSphere();
            console.log('3D plot updated, points:', odom3dPositions.length);
        }
    }, function(error) {
        console.error('Odometry subscription error:', error);
    });

    // Run button
    var runBtn = document.getElementById('runBtn');
    if (runBtn) {
        runBtn.onclick = function() {
            // Make Run button green when pressed
            runBtn.classList.add('btn-green');
            runBtn.style.background = '#00d084';
            runBtn.style.color = '#121829';
            runBtn.style.borderColor = '#00d084';

            var bagInput = document.getElementById('bag');
            if (!bagInput || bagInput.value.trim() === "") {
                alert("Please enter a bag file path.");
                return;
            }
            var bagPath = bagInput.value.trim();

            showLoading(); // Show loading overlay when Run is pressed

            var startService = new ROSLIB.Service({
                ros: ros,
                name: '/viohar/start',
                serviceType: 'viohar_prediction/StartPipeline'
            });

            var request = new ROSLIB.ServiceRequest({ bag_file: bagPath });

            startService.callService(request, function(result) {
                alert(result.message);
            });
        };
    }

    // Stop button
    var stopBtn = document.getElementById('stopBtn');
    if (stopBtn) {
        stopBtn.onclick = function() {
            var stopService = new ROSLIB.Service({
                ros: ros,
                name: '/viohar/stop',
                serviceType: 'viohar_prediction/StopPipeline'
            });

            var request = new ROSLIB.ServiceRequest({});
            stopService.callService(request, function(result) {
                alert(result.message);
            });
            // Remove green color from Run button when Stop is pressed
            if (runBtn) {
                runBtn.classList.remove('btn-green');
                runBtn.style.background = '';
                runBtn.style.color = '';
                runBtn.style.borderColor = '';
            }
        };
    }

    // Clear button
    var clearBtn = document.getElementById('clearBtn');
    if (clearBtn) {
        clearBtn.onclick = function() {
            if (logDiv) logDiv.innerText = "";
        };
    }

};