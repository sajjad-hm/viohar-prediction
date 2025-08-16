// odom_plot.js
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.153.0/build/three.module.js';

export class OdomPlot {
    constructor(canvas) {
        this.canvas = canvas;
        this.renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        this.renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0d1324);
        this.camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.01, 1000);
        this.camera.position.set(0, -5, 5);
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt(0, 0, 0);

        // Add grid and axes
        const gridHelper = new THREE.GridHelper(10, 20);
        this.scene.add(gridHelper);
        const axesHelper = new THREE.AxesHelper(1.5);
        this.scene.add(axesHelper);

        this.odomPositions = [];
        this.odomGeometry = new THREE.BufferGeometry();
        this.odomMaterial = new THREE.LineBasicMaterial({ color: 0x00d084 });
        this.odomLine = new THREE.Line(this.odomGeometry, this.odomMaterial);
        this.scene.add(this.odomLine);

        // Add a default marker (sphere) at origin for visual confirmation
        const markerGeometry = new THREE.SphereGeometry(0.1, 16, 16);
        const markerMaterial = new THREE.MeshBasicMaterial({ color: 0xff6b6b });
        const marker = new THREE.Mesh(markerGeometry, markerMaterial);
        marker.position.set(0, 0, 0);
        this.scene.add(marker);

        this.animate = this.animate.bind(this);
        this.animate();
    }

    addPosition(x, y, z) {
        this.odomPositions.push(new THREE.Vector3(x, y, z));
        if (this.odomPositions.length > 2000) this.odomPositions.shift();
        const positionsArray = new Float32Array(this.odomPositions.length * 3);
        for (let i = 0; i < this.odomPositions.length; i++) {
            positionsArray[i * 3] = this.odomPositions[i].x;
            positionsArray[i * 3 + 1] = this.odomPositions[i].y;
            positionsArray[i * 3 + 2] = this.odomPositions[i].z;
        }
        this.odomGeometry.setAttribute('position', new THREE.BufferAttribute(positionsArray, 3));
        this.odomGeometry.setDrawRange(0, this.odomPositions.length);
        this.odomGeometry.computeBoundingSphere();
    }

    animate() {
        this.renderer.render(this.scene, this.camera);
        requestAnimationFrame(this.animate);
    }
}
