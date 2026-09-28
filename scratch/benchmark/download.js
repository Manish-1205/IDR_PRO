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
const OsmConverter_js_1 = require("../../src/mapmatch/OsmConverter.js");
const fs = __importStar(require("fs"));
async function main() {
    // Solapur bounding box covering test2.csv area
    // test2.csv has TruthLat ~ 17.6464, TruthLon ~ 75.9517
    const minLat = 17.64;
    const minLon = 75.94;
    const maxLat = 17.65;
    const maxLon = 75.96;
    console.log("Downloading OSM data...");
    const segments = await OsmConverter_js_1.OsmConverter.fetchAndConvert(minLat, minLon, maxLat, maxLon);
    console.log(`Downloaded ${segments.length} segments.`);
    fs.writeFileSync('scratch/benchmark/roads.json', JSON.stringify(segments, null, 2));
    console.log("Saved to scratch/benchmark/roads.json");
}
main().catch(console.error);
