# Indian Road Pathfinder — Reproducibility & Validation Package
**Team Oorum Blood · Indian Institute of Technology Kharagpur**
**SIH Problem Statement 26037 (MathWorks)**

## 1. Single Reproduction Command
Run the standalone forensic audit reproducing the exact Seed 438 comparison:
```bash
./run_demo.sh
# or on Windows:
run_demo.bat
```

## 2. Hardware Specification
- **Processor**: AMD Ryzen 5 7530U with Radeon Graphics (6 Cores, 12 Threads)
- **Base Frequency**: 2.00 GHz (Max Boost: 4.50 GHz)
- **Memory**: 16.0 GB DDR4
- **OS**: Windows 11 Home 64-bit
- **Environment**: Headless MATLAB R2026a (Zero-Toolbox Dependency)

## 3. Key Constraints & Safety Parameters
- **Minimum Turning Radius**: $R_{\min} = 3.8\text{ m}$ (Ackermann Kinodynamic constraint)
- **Maximum Steering Slew Rate**: $25^\circ/\text{s}$
- **Comfort Longitudinal Jerk Band**: $\le 0.80\text{ m/s}^3$ (nominal cruise)
- **Emergency Braking Reflex Slew**: $8.00\text{ m/s}^3$ (active in $<0.21\%$ of nominal runtime)
- **Audited Narrow Corridor Threshold**: $W_{\text{hero}} = 2.55\text{ m}$ (Virtual Stop Line deployed $3.5\text{ m}$ upstream)
- **Mean Replanning Latency**: $3.65\text{ ms}$ (P99: $42.85\text{ ms}$, real-time budget $30.0\text{ ms}$)

## 4. Deliverables Index
- `hardware_spec.txt`: Detailed processor and execution environment configuration.
- `monte_carlo_1000_summary.json`: 1,000-trial Monte Carlo outcome and latency distribution.
- `per_stage_timing.csv`: Measured latency profile across all 7 architecture stages.
- `seed_438_collision_trace.json`: Per-tick log of uniform slew rate-choking crash.
- `centerpiece_yield_trace.json`: 30-second continuous 10 Hz telemetry trace for narrow corridor yield.
