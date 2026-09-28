"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
const fs = __importStar(require("fs"));
const path = __importStar(require("path"));
// Mock Date.now() before importing anything else
let mockTime = 0;
const originalDateNow = Date.now;
Date.now = () => mockTime;
const useSensorStore_js_1 = require("../../src/store/useSensorStore.js");
const FusionRuntime_js_1 = require("../../src/core/FusionRuntime.js");
const EkfCore_js_1 = require("../../src/core/EkfCore.js");
const InsMechanization_js_1 = require("../../src/core/InsMechanization.js");
const OutputStabilizer_js_1 = require("../../src/core/OutputStabilizer.js");
function haversine(lat1, lon1, lat2, lon2) {
    const R = 6378137;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}
async function runScenario(csvPath, blackoutStartSec, blackoutEndSec, aiEnabled, outCsv) {
    const content = fs.readFileSync(csvPath, 'utf-8');
    const lines = content.split('\n').map(l => l.trim()).filter(l => l.length > 0);
    // Reset state
    useSensorStore_js_1.useSensorStore.setState({
        accel: { x: 0, y: 0, z: 0, timestamp: 0 },
        gyro: { x: 0, y: 0, z: 0, timestamp: 0 },
        mag: { x: 0, y: 0, z: 0, timestamp: 0 },
        gnss: { latitude: null, longitude: null, altitude: null, accuracy: null, timestamp: 0, speed: null, heading: null },
        isAcquiring: false
    });
    // Re-instantiate fusion runtime internally to avoid state leak
    FusionRuntime_js_1.fusionRuntime.ekf = new EkfCore_js_1.EkfCore();
    FusionRuntime_js_1.fusionRuntime.pureIns = new InsMechanization_js_1.InsMechanization();
    FusionRuntime_js_1.fusionRuntime.outputStabilizer = new OutputStabilizer_js_1.OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    FusionRuntime_js_1.fusionRuntime.imuWindow = [];
    FusionRuntime_js_1.fusionRuntime.lastImuTimestamp = 0;
    FusionRuntime_js_1.fusionRuntime.lastGnssTimestamp = 0;
    FusionRuntime_js_1.fusionRuntime.initialLat = null;
    FusionRuntime_js_1.fusionRuntime.initialLon = null;
    FusionRuntime_js_1.fusionRuntime.isAttitudeInitialized = false;
    for (let i = 0; i < 20; i++) {
        FusionRuntime_js_1.fusionRuntime.imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
    // Disable AI if requested
    if (!aiEnabled) {
        FusionRuntime_js_1.fusionRuntime.ekf.getAiModel().isModelLoaded = false;
    }
    else {
        FusionRuntime_js_1.fusionRuntime.ekf.getAiModel().isModelLoaded = true;
        if (!FusionRuntime_js_1.fusionRuntime.ekf.getAiModel().model) {
            FusionRuntime_js_1.fusionRuntime.ekf.getAiModel().model = {
                run: () => { return [new Float32Array([0, 0]).buffer]; }
            };
        }
    }
    let currentFusedState = null;
    FusionRuntime_js_1.fusionRuntime.setOnFusedDataCallback((state) => {
        currentFusedState = state;
    });
    await FusionRuntime_js_1.fusionRuntime.start();
    const outputLines = ['Timestamp,TruthLat,TruthLon,EstLat,EstLon,Error'];
    let firstTimestamp = 0;
    let maxError = 0;
    let sumError = 0;
    let distTravelled = 0;
    let lastTruthLat = null;
    let lastTruthLon = null;
    let lastGnssLat = null;
    let validUpdates = 0;
    for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',');
        if (cols.length < 18)
            continue;
        const ts = parseInt(cols[0], 10);
        if (firstTimestamp === 0)
            firstTimestamp = ts;
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
                useSensorStore_js_1.useSensorStore.getState().setGNSS({
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
        useSensorStore_js_1.useSensorStore.getState().setAccel({ x: accelX, y: accelY, z: accelZ, timestamp: ts });
        useSensorStore_js_1.useSensorStore.getState().setGyro({ x: gyroX, y: gyroY, z: gyroZ, timestamp: ts });
        if (currentFusedState && currentFusedState.latitude !== null && currentFusedState.longitude !== null) {
            const err = haversine(gnssLat, gnssLon, currentFusedState.latitude, currentFusedState.longitude);
            if (isBlackout) {
                if (err > maxError)
                    maxError = err;
            }
            outputLines.push(ts + "," + gnssLat + "," + gnssLon + "," + currentFusedState.latitude + "," + currentFusedState.longitude + "," + err.toFixed(3));
            if (isBlackout) {
                sumError += err;
                validUpdates++;
            }
        }
    }
    FusionRuntime_js_1.fusionRuntime.stop();
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
    FusionRuntime_js_1.fusionRuntime.enableMapMatching = false;
    let resOn = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on.csv'));
    console.log("Max Error:", resOn.maxError.toFixed(2), "m");
    console.log("\n--- SCENARIO: 60s Blackout (Phase 3 MapMatch, AI ON) ---");
    // Load mock map data
    try {
        const roads = JSON.parse(fs.readFileSync(path.resolve(__dirname, 'roads.json'), 'utf-8'));
        FusionRuntime_js_1.fusionRuntime.mapMatcher.loadSegments(roads);
        FusionRuntime_js_1.fusionRuntime.enableMapMatching = true;
    }
    catch (e) {
        console.error("No roads.json found, skipping map matching.");
    }
    let resMap = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_map.csv'));
    console.log("Max Error:", resMap.maxError.toFixed(2), "m");
    FusionRuntime_js_1.fusionRuntime.enableMapMatching = false;
    console.log("\n--- SCENARIO: 60s Blackout (Phase 2 NHC+Yaw, AI ON) ---");
    FusionRuntime_js_1.fusionRuntime.ekf.enableYawAlignment = true;
    FusionRuntime_js_1.fusionRuntime.ekf.enableNHC = true;
    let resNhc = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_nhc.csv'));
    console.log("Max Error:", resNhc.maxError.toFixed(2), "m");
    console.log("\n--- SCENARIO: 60s Blackout (Phase 4 BumpGate, AI ON) ---");
    FusionRuntime_js_1.fusionRuntime.ekf.enableYawAlignment = false;
    FusionRuntime_js_1.fusionRuntime.ekf.enableNHC = false;
    FusionRuntime_js_1.fusionRuntime.enableMapMatching = false;
    FusionRuntime_js_1.fusionRuntime.ekf.enableBumpGate = true;
    FusionRuntime_js_1.fusionRuntime.ekf.bumpGateCount = 0;
    let resBump = await runScenario(logFile, 30, 90, true, path.resolve(__dirname, 'scen1_ai_on_bump.csv'));
    console.log("Max Error:", resBump.maxError.toFixed(2), "m");
    console.log("Total Bumps Detected:", FusionRuntime_js_1.fusionRuntime.ekf.bumpGateCount);
    FusionRuntime_js_1.fusionRuntime.ekf.enableBumpGate = false;
    console.log("\n--- SCENARIO: 50m Blackout (Baseline, AI ON) ---");
    let resOn50 = await runScenario(logFile, 40, 45, true, path.resolve(__dirname, 'scen2_ai_on.csv'));
    console.log("Max Error:", resOn50.maxError.toFixed(2), "m");
    console.log("\n--- SCENARIO: 50m Blackout (Phase 2 NHC+Yaw, AI ON) ---");
    FusionRuntime_js_1.fusionRuntime.ekf.enableYawAlignment = true;
    FusionRuntime_js_1.fusionRuntime.ekf.enableNHC = true;
    let resNhc50 = await runScenario(logFile, 40, 45, true, path.resolve(__dirname, 'scen2_ai_on_nhc.csv'));
    console.log("Max Error:", resNhc50.maxError.toFixed(2), "m");
    console.log("Done.");
}
main().catch(console.error);
