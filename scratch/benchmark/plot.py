import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137.0
    dlat = np.radians(lat2 - lat1)
    dlon = np.radians(lon2 - lon1)
    a = np.sin(dlat / 2)**2 + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dlon / 2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c

def analyze_and_plot(csv_file, title_prefix, out_prefix):
    if not os.path.exists(csv_file):
        print(f"Skipping {csv_file}: Not found.")
        return
        
    df = pd.read_csv(csv_file)
    if len(df) == 0:
        return
        
    # Errors
    final_error = df['Error'].iloc[-1]
    max_error = df['Error'].max()
    
    # Distance travelled
    truth_lat = df['TruthLat'].values
    truth_lon = df['TruthLon'].values
    
    dist_travelled = 0.0
    for i in range(1, len(truth_lat)):
        dist_travelled += haversine(truth_lat[i-1], truth_lon[i-1], truth_lat[i], truth_lon[i])
        
    drift_pct = (final_error / dist_travelled) * 100 if dist_travelled > 0 else 0
    
    print(f"{title_prefix}")
    print(f"  Final Position Error: {final_error:.2f} m")
    print(f"  Max Error during blackout: {max_error:.2f} m")
    print(f"  Distance Travelled: {dist_travelled:.2f} m")
    print(f"  Drift: {drift_pct:.2f}% of distance travelled")
    print()
    
    # Plot Trajectory
    plt.figure(figsize=(8,6))
    plt.plot(df['TruthLon'], df['TruthLat'], label='Truth', color='blue')
    plt.plot(df['EstLon'], df['EstLat'], label='Estimate', color='red', linestyle='dashed')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(f"{title_prefix} - Trajectory")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{out_prefix}_trajectory.png")
    plt.close()
    
    # Plot Error vs Time
    plt.figure(figsize=(8,4))
    # Assuming Timestamp is ms, convert to relative seconds
    t = (df['Timestamp'] - df['Timestamp'].iloc[0]) / 1000.0
    plt.plot(t, df['Error'], color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title(f"{title_prefix} - Error over Time")
    plt.grid(True)
    plt.savefig(f"{out_prefix}_error.png")
    plt.close()

def main():
    print("Generating Benchmark Reports & Plots...\n")
    analyze_and_plot('scen1_ai_on.csv', 'Scenario: 60s Blackout (AI ON)', 'scen1_on')
    analyze_and_plot('scen1_ai_off.csv', 'Scenario: 60s Blackout (AI OFF)', 'scen1_off')
    analyze_and_plot('scen2_ai_on.csv', 'Scenario: 50m Blackout (AI ON)', 'scen2_on')

if __name__ == '__main__':
    main()
