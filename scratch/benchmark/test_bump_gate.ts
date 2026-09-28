import { EkfCore } from '../../src/core/EkfCore.js';
import { Matrix } from 'ml-matrix';

function testBumpGate() {
  console.log("Running Bump Gate Unit Test...");
  const ekf = new EkfCore();
  
  // Enable the feature
  ekf.enableBumpGate = true;
  
  // Normal IMU sample (1g Z, no rotation)
  const dt = 0.1;
  const normalAccel = [0, 0, 9.81];
  const normalGyro = [0, 0, 0];
  
  // Run normal predict
  ekf.predict(dt, normalAccel, normalGyro);
  let qBefore = (ekf as any).lastVelQ;
  let bumpsBefore = ekf.bumpGateCount;
  
  console.log(`Normal step: Vel Q = ${qBefore.toFixed(5)}, Bumps = ${bumpsBefore}`);
  
  // Synthetic pothole spike (e.g. 15 m/s^2 on Z)
  const bumpAccel = [0, 0, 15.0];
  const bumpGyro = [0.5, 0.5, 0];
  
  ekf.predict(dt, bumpAccel, bumpGyro);
  let qAfter = (ekf as any).lastVelQ;
  let bumpsAfter = ekf.bumpGateCount;
  
  console.log(`Bump step: Vel Q = ${qAfter.toFixed(5)}, Bumps = ${bumpsAfter}`);
  
  if (bumpsAfter > bumpsBefore && qAfter > qBefore) {
    console.log("PASS: Bump detected and Q inflamed.");
  } else {
    console.log("FAIL: Bump not detected or Q not inflated.");
  }
}

testBumpGate();
