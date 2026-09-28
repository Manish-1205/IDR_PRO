import { fusionRuntime, ImuSource } from '../../src/core/FusionRuntime.js';

async function testPhase5() {
  console.log("Running Phase 5: 200 Hz External IMU & Latency Test...");
  
  const imuRate = 200; // Hz
  const dtMs = 1000 / imuRate;
  
  let currentTs = 10000;
  
  // Feed GNSS to initialize
  (fusionRuntime as any).handleGnssUpdate({
      gnss: {
          latitude: 17.0,
          longitude: 75.0,
          accuracy: 5,
          timestamp: currentTs,
          speed: 0,
          heading: 0
      }
  });

  console.log(`Initialized GNSS at ts=${currentTs}`);

  // Feed 1 second of IMU (200 samples)
  for (let i = 0; i < imuRate; i++) {
    currentTs += dtMs;
    const imu: ImuSource = {
        timestamp: currentTs,
        accel: { x: 0, y: 0, z: 9.81 },
        gyro: { x: 0, y: 0, z: 0 }
    };
    fusionRuntime.feedExternalImu(imu);
  }

  // GNSS loss happens. Need to wait > 2000 ms to trigger loss state
  console.log("Simulating GNSS loss (2500ms of IMU without GNSS)...");
  
  for (let i = 0; i < (2500 / dtMs); i++) {
    currentTs += dtMs;
    const imu: ImuSource = {
        timestamp: currentTs,
        accel: { x: 0, y: 0, z: 9.81 },
        gyro: { x: 0, y: 0, z: 0 }
    };
    fusionRuntime.feedExternalImu(imu);
  }
  
  console.log("Loss Latency (GNSS Loss to first AI-aided output):", (fusionRuntime as any).latencyMetrics.lossLatencyMs, "ms");

  // GNSS Returns
  console.log("Simulating GNSS return...");
  currentTs += dtMs;
  (fusionRuntime as any).handleGnssUpdate({
      gnss: {
          latitude: 17.0001,
          longitude: 75.0001,
          accuracy: 5,
          timestamp: currentTs,
          speed: 0,
          heading: 0
      }
  });
  
  // Feed one IMU sample to trigger fused output
  currentTs += dtMs;
  fusionRuntime.feedExternalImu({
      timestamp: currentTs,
      accel: { x: 0, y: 0, z: 9.81 },
      gyro: { x: 0, y: 0, z: 0 }
  });

  console.log("Return Latency (GNSS Return to first fused output):", (fusionRuntime as any).latencyMetrics.returnLatencyMs, "ms");
  
  if ((fusionRuntime as any).latencyMetrics.lossLatencyMs > 0 && (fusionRuntime as any).latencyMetrics.returnLatencyMs > 0) {
      console.log("PASS: 200 Hz integration and latency tracking successful.");
  } else {
      console.log("FAIL: Latency metrics not properly captured.");
  }
}

testPhase5().catch(console.error);
