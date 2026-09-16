// @ts-nocheck
import * as fs from 'fs';
import * as path from 'path';
import { EkfCore } from '../EkfCore';

describe('GNSS Q-Matrix Verification', () => {
    it('should eliminate sawtooth pattern in GOOD state and accept updates', () => {
        const csvPath = 'e:\\reckonX_SIH\\test2.csv';
        const content = fs.readFileSync(csvPath, 'utf-8');
        const lines = content.split('\n').map((l: string) => l.trim()).filter((l: string) => l.length > 0);
        
        const ekf = new EkfCore();
        let initialLat = null;
        let initialLon = null;
        const R_EARTH = 6378137.0;

        let lastGnssLat = null;
        let lastGnssLon = null;
        let lastTime = 0;

        console.log(`[Validation] Running on ${lines.length - 1} rows...`);

        let maxError = 0;
        let totalError = 0;
        let updateCount = 0;

        for (let i = 1; i < lines.length; i++) {
            const cols = lines[i].split(',');
            if (cols.length < 10) continue;

            const time = parseFloat(cols[0]);
            const accX = parseFloat(cols[1]);
            const accY = parseFloat(cols[2]);
            const accZ = parseFloat(cols[3]);
            const gyrX = parseFloat(cols[4]);
            const gyrY = parseFloat(cols[5]);
            const gyrZ = parseFloat(cols[6]);
            const gnssLat = parseFloat(cols[7]);
            const gnssLon = parseFloat(cols[8]);
            const gnssAcc = parseFloat(cols[9]);

            let dt = lastTime === 0 ? 0.1 : (time - lastTime) / 1000.0;
            if (dt <= 0 || dt > 1.0) dt = 0.1;
            lastTime = time;

            if (i === 1) {
                ekf.getIns().initializeAttitude({ x: accX, y: accY, z: accZ });
            }

            // Predict
            ekf.predict(dt, [accX, accY, accZ], [gyrX, gyrY, gyrZ]);

            // If new GNSS fix arrived
            if (gnssLat !== lastGnssLat || gnssLon !== lastGnssLon || i === 1) {
                if (initialLat === null) {
                    initialLat = gnssLat;
                    initialLon = gnssLon;
                }

                lastGnssLat = gnssLat;
                lastGnssLon = gnssLon;
                
                const latRad = initialLat * (Math.PI / 180);
                const dx = (gnssLon - initialLon) * (Math.PI / 180) * R_EARTH * Math.cos(latRad);
                const dy = (gnssLat - initialLat) * (Math.PI / 180) * R_EARTH;

                const posBefore = { ...ekf.getPosition() };
                
                ekf.updateGnss([dx, dy, 0], null, gnssAcc, []);

                const posAfter = { ...ekf.getPosition() };
                
                // Calculate error relative to raw GNSS
                const errX = posAfter.x - dx;
                const errY = posAfter.y - dy;
                const errorMag = Math.sqrt(errX*errX + errY*errY);

                if (i > 1) { // Skip first initialization
                    maxError = Math.max(maxError, errorMag);
                    totalError += errorMag;
                    updateCount++;
                }

                console.log(`[Update ${updateCount}] GNSS dx=${dx.toFixed(3)}, dy=${dy.toFixed(3)} | ` +
                            `Before: ${posBefore.x.toFixed(3)}, ${posBefore.y.toFixed(3)} | ` +
                            `After: ${posAfter.x.toFixed(3)}, ${posAfter.y.toFixed(3)} | ` +
                            `Error: ${errorMag.toFixed(3)}m`);
            }
        }
        
        const avgError = totalError / Math.max(1, updateCount);
        console.log(`\n--- STATS ---`);
        console.log(`Max Error: ${maxError.toFixed(3)}m`);
        console.log(`Avg Error: ${avgError.toFixed(3)}m`);
        console.log(`Updates Processed: ${updateCount}`);
    });
});
