import csv

# Load sensor layer 1000 trials
with open('batch_test_results_sensor_layer.csv', 'r') as f:
    sensor_data = {int(r['seed']): r for r in csv.DictReader(f)}

# Known baseline collisions from task-133.log
# First 15 collisions in baseline: seeds 6, 8, 9, 13, 15, 17, 23, 24, 25, 29, 30, 36, 37, 38, 39
baseline_first_collisions = {
    6: 0.14,
    8: 0.91,
    9: 0.97,
    13: -0.03,
    15: 0.19,
    17: 0.82,
    23: 0.86,
    24: 0.19,
    25: 0.17,
    29: 0.15,
    30: -0.00,
    36: 0.09,
    37: -0.20,
    38: 0.89,
    39: -0.03
}

print("=== Comparison on first 40 seeds ===")
for seed in range(1, 41):
    s_res = sensor_data.get(seed)
    s_outcome = s_res['outcome']
    s_time = s_res['time_to_goal']
    s_clr = s_res['min_clearance_achieved']
    s_drop = s_res['total_dropouts']
    s_im = s_res['innov_mean']
    
    b_is_col = seed in baseline_first_collisions
    b_outcome = "COLLISION" if b_is_col else ("SUCCESS" if seed <= 39 else "UNKNOWN")
    
    diff_flag = ""
    if b_outcome != "UNKNOWN" and b_outcome != s_outcome:
        diff_flag = f" <--- CHANGED ({b_outcome} -> {s_outcome})"
    
    print(f"Seed {seed:2d}: Baseline={b_outcome:9s} | Sensor={s_outcome:9s} (t={s_time:5s}, clr={s_clr:6s}, drops={s_drop:2s}, innov_m={s_im:6s}){diff_flag}")
