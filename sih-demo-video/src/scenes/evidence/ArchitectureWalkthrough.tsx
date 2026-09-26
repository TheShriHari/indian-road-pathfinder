import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";

interface StageInfo {
  id: number;
  title: string;
  sourceFile: string;
  latencyMs: number;
  mathFormula: string;
  telemetryCode: string;
  summary: string;
}

const STAGES: StageInfo[] = [
  {
    id: 1,
    title: "1. Sensor Layer & Range Gating",
    sourceFile: "matlab/simulate_sensor_detection.m",
    latencyMs: 0.38,
    mathFormula: "r_max = 35.0m, \\theta \\in [-70^\\circ, +70^\\circ]",
    telemetryCode: "sensor_raw = simulate_sensor_detection(ego_pose, world_agents, potholes);\nrange_filtered = sensor_raw.dist <= 35.0 & abs(sensor_raw.azimuth) <= deg2rad(70);",
    summary: "Radial 35m cutoff & 140° azimuth boundary filters unpaved roadside noise.",
  },
  {
    id: 2,
    title: "2. EKF Estimation & Slip Rejection",
    sourceFile: "matlab/test_ekf_convergence.m",
    latencyMs: 0.32,
    mathFormula: "d_M = \\sqrt{(z - \\hat{z})^T S^{-1} (z - \\hat{z})} < 3.0\\sigma",
    telemetryCode: "residual = z_meas - H * x_pred;\nS = H * P_pred * H' + R_noise;\nd_M = sqrt(residual' / S * residual); % Mahalanobis gate rejects gravel wheel slip",
    summary: "Mahalanobis distance gating filters loose gravel wheel slip from corrupting state estimate.",
  },
  {
    id: 3,
    title: "3. Dynamic Obstacle Covariance Predictor",
    sourceFile: "matlab/dynamic_obstacle_predictor.m",
    latencyMs: 0.58,
    mathFormula: "\\Sigma_{k+1} = F \\Sigma_k F^T + Q_{agent}, \\quad \\chi^2_{0.95} = 5.991",
    telemetryCode: "pred = dynamic_obstacle_predictor(tracks, dt, 3.0); % 3.0s time-horizon\n% Cattle: isotropic circular covariance (1.6m) | Rickshaw: oriented elongated (2.4m x 0.9m)",
    summary: "Generates 95% confidence covariance envelopes tailored to bimodal cattle vs auto-rickshaws.",
  },
  {
    id: 4,
    title: "4. 60x30m Cellular Rolling Costmap",
    sourceFile: "matlab/local_occupancy_grid_builder.m",
    latencyMs: 0.85,
    mathFormula: "C(d) = 100 \\cdot \\exp(-\\alpha \\cdot (d - d_{safe})), \\quad \\Delta = 0.2m",
    telemetryCode: "[cmap, gmeta] = local_occupancy_grid_builder(ego_pose, obstacles, predictions);\n% 150x300 matrix, 0.2m resolution, continuous exponential decay inflation",
    summary: "Zero-toolbox 150x300 matrix with continuous exponential decay away from eroded edges.",
  },
  {
    id: 5,
    title: "5. Universal Bottleneck Decider (Hero Metric)",
    sourceFile: "matlab/universal_bottleneck_decider.m",
    latencyMs: 0.44,
    mathFormula: "W_{free} = d_{left} + d_{right}. \\quad \\text{If } W_{free} < 2.55m \\implies \\text{VSL deployed at } s - 3.5m",
    telemetryCode: "[vstop, stop_pose, info] = universal_bottleneck_decider(ego_state, path, cmap, gmeta, dyn_actors);\n% Hero 2.55m clearance trigger: holds before pinch, clears after oncoming pass",
    summary: "Audits available drivable width. If corridor < 2.55m with oncoming agent, deploys upstream VSL.",
  },
  {
    id: 6,
    title: "6. Lattice / Hybrid A* Planner & Spline",
    sourceFile: "matlab/adaptive_path_planner.m",
    latencyMs: 1.12,
    mathFormula: "R_{min} = 3.8m, \\quad \\kappa_{max} = 0.263 m^{-1}, \\quad C^2 \\text{ continuity}",
    telemetryCode: "path = adaptive_path_planner(ego_pose, goal_pose, cmap, gmeta, vstop_active);\n% Kinodynamically feasible trajectory satisfying Ackermann steering curvature bounds",
    summary: "Samples candidate paths respecting 3.8m Ackermann turning limit with zero high-frequency curvature jerk.",
  },
  {
    id: 7,
    title: "7. Deterministic FSM & Asymmetric Jerk Controller",
    sourceFile: "matlab/behavior_state_machine.m",
    latencyMs: 0.21,
    mathFormula: "j_{comfort} \\le 0.80 m/s^3, \\quad j_{emergency} = 8.00 m/s^3, \\quad \\dot{\\delta} \\le 25^\\circ/s",
    telemetryCode: "[bsm_state, v_ref] = behavior_state_machine(bsm_state, ego_state, predictions, dt, bsm_params);\n[steer, accel] = pure_pursuit_controller(ego_state, path, v_ref, dt);",
    summary: "Asymmetric slew limiter maintains 0.80 m/s³ comfort, unlocking 8.0 m/s³ emergency reflex with anti-chatter latch.",
  },
];

export const ArchitectureWalkthrough: React.FC = () => {
  const frame = useCurrentFrame();

  // 1200 frames total (40s). ~170 frames per stage.
  const stageDuration = 1200 / STAGES.length;
  const activeIndex = Math.min(STAGES.length - 1, Math.floor(frame / stageDuration));
  const currentStage = STAGES[activeIndex];

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
      {/* Grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
      </svg>

      {/* Top Header */}
      <div style={{ position: "absolute", top: 70, left: 100, display: "flex", justifyContent: "space-between", width: 1720 }}>
        <div>
          <div style={{ fontFamily: "'Courier New', monospace", fontSize: 15, color: COLORS.cyan, letterSpacing: 4, fontWeight: 700 }}>
            PIPELINE ARCHITECTURE (7 REPRODUCIBLE MODULES)
          </div>
          <div style={{ fontFamily: "'Courier New', monospace", fontSize: 28, color: COLORS.text, fontWeight: 700, letterSpacing: 1.5, marginTop: 4 }}>
            First-Principles Stack · Zero External Toolboxes
          </div>
        </div>
        <div style={{ textAlign: "right", fontFamily: "'Courier New', monospace" }}>
          <div style={{ fontSize: 14, color: COLORS.textMuted }}>TOTAL PIPELINE MEDIAN LATENCY</div>
          <div style={{ fontSize: 26, color: COLORS.green, fontWeight: 700 }}>3.65 ms <span style={{ fontSize: 16, color: COLORS.textMuted }}>(BUDGET: 30.0 ms)</span></div>
        </div>
      </div>

      {/* Left Stage Selector Sidebar */}
      <div
        style={{
          position: "absolute",
          top: 175,
          left: 100,
          width: 500,
          display: "flex",
          flexDirection: "column",
          gap: 12,
        }}
      >
        {STAGES.map((s, idx) => {
          const isActive = idx === activeIndex;
          return (
            <div
              key={s.id}
              style={{
                padding: "14px 20px",
                borderRadius: 10,
                border: `1.5px solid ${isActive ? COLORS.cyan : COLORS.curb}`,
                backgroundColor: isActive ? `${COLORS.cyan}18` : `${COLORS.bg}EE`,
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                fontFamily: "'Courier New', monospace",
                transition: "all 0.15s ease",
              }}
            >
              <span style={{ fontSize: 16, fontWeight: isActive ? 700 : 500, color: isActive ? COLORS.text : COLORS.textMuted }}>
                {s.title}
              </span>
              <span style={{ fontSize: 13, color: isActive ? COLORS.cyan : COLORS.textMuted, fontWeight: 600 }}>
                {s.latencyMs.toFixed(2)} ms
              </span>
            </div>
          );
        })}
      </div>

      {/* Right Stage Technical Deep-Dive Panel */}
      <div
        style={{
          position: "absolute",
          top: 175,
          left: 640,
          width: 1180,
          height: 800,
          padding: "36px 44px",
          borderRadius: 16,
          border: `2px solid ${COLORS.cyan}60`,
          backgroundColor: `${COLORS.bg}F8`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 20,
          boxSizing: "border-box",
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 16 }}>
          <div>
            <div style={{ fontSize: 14, color: COLORS.cyan, letterSpacing: 3, fontWeight: 700 }}>
              MODULE {currentStage.id} OF 7 · PROFILING & MATH VERIFICATION
            </div>
            <div style={{ fontSize: 26, fontWeight: 700, color: COLORS.text, marginTop: 4 }}>
              {currentStage.title}
            </div>
          </div>
          <div style={{ textAlign: "right" }}>
            <div style={{ fontSize: 13, color: COLORS.textMuted }}>MODULE SOURCE FILE</div>
            <div style={{ fontSize: 15, color: COLORS.amber, fontWeight: 700 }}>{currentStage.sourceFile}</div>
          </div>
        </div>

        {/* Mathematical Formulation Card */}
        <div
          style={{
            padding: "16px 24px",
            borderRadius: 10,
            border: `1px solid ${COLORS.curb}`,
            backgroundColor: `${COLORS.road}CC`,
            display: "flex",
            flexDirection: "column",
            gap: 6,
          }}
        >
          <span style={{ fontSize: 13, color: COLORS.textMuted, letterSpacing: 2 }}>MATHEMATICAL FORMULATION / GATING CRITERIA:</span>
          <span style={{ fontSize: 18, color: COLORS.green, fontWeight: 700, letterSpacing: 1 }}>
            {currentStage.mathFormula}
          </span>
        </div>

        {/* Live Code / Telemetry Snippet */}
        <div
          style={{
            padding: "20px 24px",
            borderRadius: 10,
            border: `1px solid ${COLORS.curb}`,
            backgroundColor: "#070C15",
            display: "flex",
            flexDirection: "column",
            gap: 10,
            flex: 1,
          }}
        >
          <div style={{ display: "flex", justifyContent: "space-between" }}>
            <span style={{ fontSize: 13, color: COLORS.textMuted, letterSpacing: 2 }}>CORE MATLAB / C++20 LOGIC SNIPPET:</span>
            <span style={{ fontSize: 13, color: COLORS.cyan, fontWeight: 600 }}>MEDIAN LATENCY: {currentStage.latencyMs} ms</span>
          </div>
          <pre
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 15,
              color: "#E2E8F0",
              lineHeight: 1.6,
              margin: 0,
              whiteSpace: "pre-wrap",
            }}
          >
            {currentStage.telemetryCode}
          </pre>
          <div style={{ marginTop: "auto", borderTop: `1px solid ${COLORS.curb}`, paddingTop: 14, fontSize: 15, color: COLORS.textMuted }}>
            {currentStage.summary}
          </div>
        </div>

        {/* Bottom Hardware Execution Bar */}
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", fontSize: 13, color: COLORS.textMuted }}>
          <span>PROFILED ON: AMD Ryzen 5 7530U (6 Cores, 12 Threads)</span>
          <span style={{ color: COLORS.green, fontWeight: 700 }}>✓ DETERMINISTIC REAL-TIME CLOSURE</span>
        </div>
      </div>
    </AbsoluteFill>
  );
};
