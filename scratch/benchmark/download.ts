import { OsmConverter } from '../../src/mapmatch/OsmConverter.js';
import * as fs from 'fs';

async function main() {
    // Solapur bounding box covering test2.csv area
    // test2.csv has TruthLat ~ 17.6464, TruthLon ~ 75.9517
    const minLat = 17.64;
    const minLon = 75.94;
    const maxLat = 17.65;
    const maxLon = 75.96;
    
    console.log("Downloading OSM data...");
    const segments = await OsmConverter.fetchAndConvert(minLat, minLon, maxLat, maxLon);
    console.log(`Downloaded ${segments.length} segments.`);
    
    fs.writeFileSync('scratch/benchmark/roads.json', JSON.stringify(segments, null, 2));
    console.log("Saved to scratch/benchmark/roads.json");
}

main().catch(console.error);
