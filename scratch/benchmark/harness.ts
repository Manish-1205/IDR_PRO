import * as fs from 'fs';
import * as path from 'path';

// Mock Date.now() before importing anything else
let mockTime = 0;
const originalDateNow = Date.now;
Date.now = () => mockTime;

import { useSensorStore } from '../../src/store/useSensorStore.js';
import { fusionRuntime, FusedState } from '../../src/core/FusionRuntime.js';

import { EkfCore } from '../../src/core/EkfCore.js';
import { InsMechanization } from '../../src/core/InsMechanization.js';
import { OutputStabilizer } from '../../src/core/OutputStabilizer.js';

function haversine(lat1: number, lon1: number, lat2: number, lon2: number) {
    const R = 6378137;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
              Math.sin(dLon/2) * Math.sin(dLon/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    return R * c;
}

async function runScenario(csvPath: string, blackoutStartSec: number, blackoutEndSec: number, aiEnabled: boolean, outCsv: string) {
    const content = fs.readFileSync(csvPath, 'utf-8');
    const lines = content.split('\n').map(l => l.trim()).filter(l => l.length > 0);
    
    // Reset state
    useSensorStore.setState({
        accel: { x: 0, y: 0, z: 0, timestamp: 0 },
        gyro: { x: 0, y: 0, z: 0, timestamp: 0 },
        mag: { x: 0, y: 0, z: 0, timestamp: 0 },
        gnss: { latitude: null, longitude: null, altitude: null, accuracy: null, timestamp: 0, speed: null, heading: null },
        isAcquiring: false
    });

    // Re-instantiate fusion runtime internally to avoid state leak
    (fusionRuntime as any).ekf = new EkfCore();
    (fusionRuntime as any).pureIns = new InsMechanization();
    (fusionRuntime as any).outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    (fusionRuntime as any).imuWindow = [];
    (fusionRuntime as any).lastImuTimestamp = 0;
    (fusionRuntime as any).lastGnssTimestamp = 0;
    (fusionRuntime as any).initialLat = null;
    (fusionRuntime as any).initialLon = null;
    (fusionRuntime as any).isAttitudeInitialized = false;
    for (let i = 0; i < 20; i++) {
        (fusionRuntime as any).imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
    
    // Disable AI if requested
    if (!aiEnabled) {
        (fusionRuntime as any).ekf.getAiModel().isModelLoaded = false;
    } else {
        (fusionRuntime as any).ekf.getAiModel().isModelLoaded = true;
        if (!(fusionRuntime as any).ekf.getAiModel().model) {
            (fusionRuntime as any).ekf.getAiModel().model = {
                run: () => { return [new Float32Array([0, 0]).buffer]; }
            };
        }
    }

    let currentFusedState: FusedState | null = null;
    fusionRuntime.setOnFusedDataCallback((state) => {
        currentFusedState = state as FusedState;
    });

    await fusionRuntime.start();

    const outputLines = ['Timestamp,TruthLat,TruthLon,EstLat,EstLon,Error'];
    let firstTimestamp = 0;
    
    let maxError = 0;
    let sumError = 0;
    let distTravelled = 0;
    let lastTruthLat: number | null = null;
    let lastTruthLon: number | null = null;
    
    let lastGnssLat: number | null = null;

    let validUpdates = 0;

    for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',');
        if (cols.length < 18) continue;
        
        const ts = parseInt(cols[0], 10);
        if (firstTimestamp === 0) firstTimestamp = ts;
        mockTime = ts;

        const relSec = (ts - firstTimestamp) / 1000.0;

        const accelX = parseFloat(cols[1]);
        const accelY = parseFloat(cols[2]);
        const accelZ = parseFloat(cols[3]);
        const gyroX = parseFloat(cols[4]);
        const gyroY = parseFloat(cols[5]);
        const gyroZ = parseFloat(cols[6]);
        
        const gnssLat = parseFloat(cols[7]);
        const gnssLon = parseFloat(cols[8]);
        const gnssAcc = parseFloat(cols[9]);

        if (lastTruthLat !== null && lastTruthLon !== null) {
            distTravelled += haversine(lastTruthLat, lastTruthLon, gnssLat, gnssLon);
        }
        lastTruthLat = gnssLat;
        lastTruthLon = gnssLon;

        // Apply GNSS if it changed and outside blackout
        let isBlackout = relSec >= blackoutStartSec && relSec <= blackoutEndSec;
        
        if (gnssLat !== lastGnssLat && !isNaN(gnssLat)) {
            lastGnssLat = gnssLat;
            if (!isBlackout) {
                useSensorStore.getState().setGNSS({
                    latitude: gnssLat,
                    longitude: gnssLon,
                    altitude: null,
                    accuracy: gnssAcc,
                    timestamp: ts,
                    speed: null,
                    heading: null
                });
            }
        }

        // Apply IMU
        useSensorStore.getState().setAccel({ x: accelX, y: accelY, z: accelZ, timestamp: ts });
        useSensorStore.getState().setGyro({ x: gyroX, y: gyroY, z: gyroZ, timestamp: ts });

        if (currentFusedState && (currentFusedState as any).latitude !== null && (currentFusedState as any).longitude !== null) {
            const err = haversine(gnssLat, gnssLon, (currentFusedState as any).latitude, (currentFusedState as any).longitude);
            if (isBlackout) {
                if (err > maxError) maxError = err;
            }
            outputLines.push(ts + "," + gnssLat + "," + gnssLon + "," + (currentFusedState as any).latitude + "," + (currentFusedState as any).longitude + "," + err.toFixed(3));
            
            if (isBlackout) {
                sumError += err;
                validUpdates++;
            }
        }
    }

    fusionRuntime.stop();
    fs.writeFileSync(outCsv, outputLines.join('\n'));
    
    const meanErr = validUpdates > 0 ? (sumError / validUpdates) : 0;
    
    return {
        maxError,
        meanError: meanErr,
        distTravelled,
    };
}

async function main() {
    console.log("Starting Benchmark Harness...");
    const logFile = path.resolve(__dirname, '../../../test2.csv');
    if (!fs.existsSync(logFile)) {
        console.error("Log file not found:", logFile);
        return;
    }
    
    console.log("\n--- SCENARIO: 60s Blackout (Baseline, AI ON) ---");
    (fusionRuntime as any).enableMapMatching = false;
    let resOn = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on.csv'));
    console.log("Max Error:", resOn.maxError.toFixed(2), "m");

    console.log("\n--- SCENARIO: 60s Blackout (Phase 3 MapMatch, AI ON) ---");
    // Load mock map data
    try {
        const roads = JSON.parse(fs.readFileSync(path.resolve(__dirname, 'roads.json'), 'utf-8'));
        (fusionRuntime as any).mapMatcher.loadSegments(roads);
        (fusionRuntime as any).enableMapMatching = true;
    } catch (e) {
        console.error("No roads.json found, skipping map matching.");
    }
    let resMap = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_map.csv'));
    console.log("Max Error:", resMap.maxError.toFixed(2), "m");
    (fusionRuntime as any).enableMapMatching = false;

    console.log("\n--- SCENARIO: 60s Blackout (Phase 2 NHC+Yaw, AI ON) ---");
    (fusionRuntime as any).ekf.enableYawAlignment = true;
    (fusionRuntime as any).ekf.enableNHC = true;
    let resNhc = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_nhc.csv'));
    console.log("Max Error:", resNhc.maxError.toFixed(2), "m");

    console.log("\n--- SCENARIO: 60s Blackout (Phase 4 BumpGate, AI ON) ---");
    (fusionRuntime as any).ekf.enableYawAlignment = false;
    (fusionRuntime as any).ekf.enableNHC = false;
    (fusionRuntime as any).enableMapMatching = false;
    (fusionRuntime as any).ekf.enableBumpGate = true;
    (fusionRuntime as any).ekf.bumpGateCount = 0;
    
    let resBump = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_bump.csv'));
    console.log("Max Error:", resBump.maxError.toFixed(2), "m");
    console.log("Total Bumps Detected:", (fusionRuntime as any).ekf.bumpGateCount);
    
    (fusionRuntime as any).ekf.enableBumpGate = false;

    console.log("\n--- SCENARIO: 50m Blackout (Baseline, AI ON) ---");
    let resOn50 = await runScenario(logFile, 40, 45, true, path.resolve(__dirname, 'scen2_ai_on.csv'));
    console.log("Max Error:", resOn50.maxError.toFixed(2), "m");

    console.log("\n--- SCENARIO: 50m Blackout (Phase 2 NHC+Yaw, AI ON) ---");
    (fusionRuntime as any).ekf.enableYawAlignment = true;
    (fusionRuntime as any).ekf.enableNHC = true;
    let resNhc50 = await runScenario(logFile, 40, 45, true, path.resolve(__dirname, 'scen2_ai_on_nhc.csv'));
    console.log("Max Error:", resNhc50.maxError.toFixed(2), "m");
    
    console.log("Done.");
}

main().catch(console.error);
