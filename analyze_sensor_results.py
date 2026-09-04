import csv
import numpy as np

# Load sensor layer results
with open('batch_test_results_sensor_layer.csv', 'r') as f:
    sensor_rows = list(csv.DictReader(f))

print(f"Loaded {len(sensor_rows)} sensor layer rows.")
sensor_outcomes = {}
for r in sensor_rows:
    sensor_outcomes[r['outcome']] = sensor_outcomes.get(r['outcome'], 0) + 1
print("Sensor layer outcome counts:", sensor_outcomes)

# Times to goal
sensor_success_times = [float(r['time_to_goal']) for r in sensor_rows if r['outcome'] == 'SUCCESS' and r['time_to_goal']]
print(f"Sensor layer success times (N={len(sensor_success_times)}):")
print(f"  Min   : {np.min(sensor_success_times):.2f}s")
print(f"  Mean  : {np.mean(sensor_success_times):.2f}s")
print(f"  Median: {np.median(sensor_success_times):.2f}s")
print(f"  Max   : {np.max(sensor_success_times):.2f}s")
print(f"  StdDev: {np.std(sensor_success_times):.2f}s")

# Innovations and dropouts
innov_means = [float(r['innov_mean']) for r in sensor_rows if r['innov_mean']]
innov_maxes = [float(r['innov_max']) for r in sensor_rows if r['innov_max']]
dropouts = [int(r['total_dropouts']) for r in sensor_rows if r['total_dropouts']]
print("\nSensor layer telemetry statistics:")
print(f"  Mean innovation across runs: {np.mean(innov_means):.4f}m")
print(f"  Max innovation across runs : {np.max(innov_maxes):.4f}m")
print(f"  Mean dropouts per run      : {np.mean(dropouts):.2f}")
print(f"  Total dropouts across 1000 : {np.sum(dropouts)}")
