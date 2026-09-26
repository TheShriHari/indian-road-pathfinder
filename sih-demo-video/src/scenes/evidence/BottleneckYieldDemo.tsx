import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS, HERO_CORRIDOR_WIDTH_M, TARGET_HARDWARE } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import yieldData from "../../data/centerpiece_yield_trace.json";

export const BottleneckYieldDemo: React.FC = () => {
  const frame = useCurrentFrame();

  // 900 frames total = 30.0s (1:10 to 1:40)
  // 300 ticks in yieldData @ 10 Hz (1 tick every 3 frames)
  const ticks = yieldData.data;
  const tickIndex = Math.min(ticks.length - 1, Math.floor(frame / 3));
  const cur = ticks[tickIndex];

  // Visual layout coordinates for top-down corridor
  const ROAD_LEFT = 680;
  const ROAD_RIGHT = 1240;
  const ROAD_WIDTH_PX = ROAD_RIGHT - ROAD_LEFT; // 560px (~3.5m, scale ~160px/m)

  // Ego vehicle vertical translation on screen (moves forward from y=860 to y=640 where it yields, then resumes to y=200)
  const egoY = interpolate(
    cur.ego_x,
    [2.0, 32.0, 65.0, 78.0],
    [860, 720, 640, 240],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Rickshaw oncoming motion (starts far ahead, squeezes past vehicle, clears corridor)
  const rickshawY = interpolate(
    cur.time,
    [0, 10, 18, 24, 30],
    [100, 320, 480, 720, 1050],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );
  const rickshawX = interpolate(
    cur.time,
    [0, 8, 16, 24, 30],
    [1060, 1040, 1030, 1080, 1140],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Cattle on left shoulder (stationary with small lateral shift)
  const cattleX = 760;
  const cattleY = 460;

  // Calipers pinch measurement at (cattleX, rickshawY)
  const isPinchActive = cur.corridor_width_m < 3.0;
  const isBreached = cur.corridor_width_m < HERO_CORRIDOR_WIDTH_M;

  // Virtual Stop Line position
  const vslY = 560; // 3.5m upstream of pinch point

  // State color
  const stateColor =
    cur.state === "CRUISE"
      ? COLORS.cyan
      : cur.state === "NUDGE"
      ? "#38BDF8"
      : cur.state === "YIELD_DECEL"
      ? COLORS.amber
      : cur.state === "YIELD_WAIT"
      ? COLORS.red
      : COLORS.green;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
      {/* ── Oscilloscope Background Grid ───────────────────────────────── */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}

        {/* Unmarked Road Surface */}
        <rect x={ROAD_LEFT} y={0} width={ROAD_WIDTH_PX} height={1080} fill={COLORS.road} />
        {/* Soft eroded shoulders */}
        <rect x={ROAD_LEFT} y={0} width={6} height={1080} fill={COLORS.curb} strokeDasharray="14 10" />
        <rect x={ROAD_RIGHT - 6} y={0} width={6} height={1080} fill={COLORS.curb} strokeDasharray="14 10" />

        {/* Planned Path Spline */}
        <path
          d={`M 960 880 L 960 700 Q 940 560 970 420 T 960 100`}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={3}
          strokeDasharray="8 6"
          opacity={0.7}
        />

        {/* Cattle with 95% circular covariance ellipse */}
        <ellipse
          cx={cattleX}
          cy={cattleY}
          rx={54}
          ry={38}
          fill="none"
          stroke={COLORS.amber}
          strokeWidth={1.5}
          strokeDasharray="4 3"
          opacity={0.8}
        />
        <ellipse cx={cattleX} cy={cattleY} rx={36} ry={22} fill={`${COLORS.amber}25`} stroke={COLORS.amber} strokeWidth={2} />
        <text x={cattleX} y={cattleY + 5} fill={COLORS.amber} fontSize={13} fontFamily="'Courier New', monospace" fontWeight="bold" textAnchor="middle">
          CATTLE (σ=1.6m)
        </text>

        {/* Oncoming Rickshaw with 95% elongated covariance ellipse */}
        <g transform={`translate(${rickshawX}, ${rickshawY})`}>
          <ellipse
            cx={0}
            cy={0}
            rx={32}
            ry={52}
            fill="none"
            stroke={COLORS.orange}
            strokeWidth={1.5}
            strokeDasharray="4 3"
            opacity={0.8}
          />
          <rect x={-20} y={-32} width={40} height={64} rx={6} fill={`${COLORS.orange}25`} stroke={COLORS.orange} strokeWidth={2} />
          <text x={0} y={4} fill={COLORS.orange} fontSize={12} fontFamily="'Courier New', monospace" fontWeight="bold" textAnchor="middle">
            RICKSHAW
          </text>
          <text x={0} y={20} fill={COLORS.textMuted} fontSize={10} fontFamily="'Courier New', monospace" textAnchor="middle">
            -3.0 m/s
          </text>
        </g>

        {/* Calipers Corridor Measurement */}
        {isPinchActive && (
          <g>
            <line x1={cattleX + 36} y1={480} x2={rickshawX - 20} y2={480} stroke={isBreached ? COLORS.red : COLORS.amber} strokeWidth={2.5} />
            <line x1={cattleX + 36} y1={465} x2={cattleX + 36} y2={495} stroke={isBreached ? COLORS.red : COLORS.amber} strokeWidth={3} />
            <line x1={rickshawX - 20} y1={465} x2={rickshawX - 20} y2={495} stroke={isBreached ? COLORS.red : COLORS.amber} strokeWidth={3} />
            <rect
              x={(cattleX + 36 + rickshawX - 20) / 2 - 80}
              y={455}
              width={160}
              height={26}
              rx={6}
              fill={`${COLORS.bg}EE`}
              stroke={isBreached ? COLORS.red : COLORS.amber}
              strokeWidth={1}
            />
            <text
              x={(cattleX + 36 + rickshawX - 20) / 2}
              y={473}
              fill={isBreached ? COLORS.red : COLORS.amber}
              fontSize={14}
              fontFamily="'Courier New', monospace"
              fontWeight="bold"
              textAnchor="middle"
            >
              WIDTH: {cur.corridor_width_m.toFixed(2)}m
            </text>
          </g>
        )}

        {/* Virtual Stop Line (VSL) 3.5m upstream */}
        {cur.vsl_active && (
          <g>
            <line x1={ROAD_LEFT + 20} y1={vslY} x2={ROAD_RIGHT - 20} y2={vslY} stroke={COLORS.red} strokeWidth={4} strokeDasharray="14 10" />
            <rect x={960 - 150} y={vslY - 28} width={300} height={24} rx={6} fill={`${COLORS.bg}EE`} stroke={COLORS.red} strokeWidth={1} />
            <text x={960} y={vslY - 12} fill={COLORS.red} fontSize={13} fontFamily="'Courier New', monospace" fontWeight="bold" textAnchor="middle">
              VIRTUAL STOP LINE ENGAGED (s=16.5m)
            </text>
          </g>
        )}
      </svg>

      {/* Ego Vehicle (Live steering and position) */}
      <VehicleSprite
        x={960}
        y={egoY}
        color={COLORS.cyan}
        wheelAngle={cur.steer_deg}
      />

      {/* ── 1. Top-Left HUD: Ego Vehicle State (Live Numbers, No Spring Bounce) ─ */}
      <div
        style={{
          position: "absolute",
          top: 60,
          left: 50,
          width: 440,
          padding: "20px 24px",
          borderRadius: 12,
          border: `1.5px solid ${COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 10,
        }}
      >
        <div style={{ fontSize: 13, color: COLORS.cyan, fontWeight: 700, letterSpacing: 2, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 6 }}>
          EGO STATE TELEMETRY (10 Hz RAW)
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>SPEED:</span>
          <span style={{ color: COLORS.cyan, fontWeight: 700, fontSize: 18 }}>
            {cur.speed_kmh.toFixed(1)} km/h <span style={{ fontSize: 13, color: COLORS.textMuted }}>({cur.speed_mps.toFixed(2)} m/s)</span>
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>STEERING ANGLE:</span>
          <span style={{ color: COLORS.text, fontWeight: 700 }}>
            {cur.steer_deg.toFixed(1)}° <span style={{ fontSize: 12, color: COLORS.textMuted }}>({cur.steer_rate_deg_s.toFixed(1)}°/s ≤ 25°/s)</span>
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>ACCELERATION:</span>
          <span style={{ color: cur.accel < 0 ? COLORS.amber : COLORS.text, fontWeight: 700 }}>
            {cur.accel.toFixed(2)} m/s²
          </span>
        </div>
        {/* Jerk Slew Meter with Comfort vs Emergency Bands */}
        <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
          <div style={{ display: "flex", justifyContent: "space-between", fontSize: 12 }}>
            <span style={{ color: COLORS.textMuted }}>JERK SLEW:</span>
            <span style={{ color: cur.jerk > 0.8 ? COLORS.amber : COLORS.green, fontWeight: 700 }}>
              {cur.jerk.toFixed(2)} m/s³ {cur.jerk <= 0.8 ? "(COMFORT)" : "(ASYM REFLEX)"}
            </span>
          </div>
          <div style={{ width: "100%", height: 8, backgroundColor: COLORS.road, borderRadius: 4, overflow: "hidden", display: "flex" }}>
            <div
              style={{
                width: `${Math.min(100, (cur.jerk / 8.0) * 100)}%`,
                backgroundColor: cur.jerk > 0.8 ? COLORS.amber : COLORS.green,
                height: "100%",
              }}
            />
          </div>
          <div style={{ display: "flex", justifyContent: "space-between", fontSize: 10, color: COLORS.textMuted }}>
            <span>0.0 m/s³</span>
            <span>COMFORT 0.80</span>
            <span>EMERGENCY 8.00</span>
          </div>
        </div>
      </div>

      {/* ── 2. Top-Right HUD: Hero Bottleneck & FSM Decision ─────────────── */}
      <div
        style={{
          position: "absolute",
          top: 60,
          right: 50,
          width: 480,
          padding: "20px 24px",
          borderRadius: 12,
          border: `1.5px solid ${isBreached ? COLORS.red : COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 12,
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 6 }}>
          <span style={{ fontSize: 13, color: isBreached ? COLORS.red : COLORS.cyan, fontWeight: 700, letterSpacing: 2 }}>
            UNIVERSAL BOTTLENECK DECIDER
          </span>
          <span style={{ fontSize: 12, color: COLORS.amber, fontWeight: 600 }}>THRESHOLD: 2.55m</span>
        </div>

        {/* HERO METRIC DISPLAY: DRIVABLE WIDTH */}
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div>
            <div style={{ fontSize: 12, color: COLORS.textMuted }}>MEASURED CORRIDOR WIDTH</div>
            <div style={{ fontSize: 28, fontWeight: 700, color: isBreached ? COLORS.red : COLORS.green, marginTop: 2 }}>
              {cur.corridor_width_m.toFixed(2)} m
            </div>
          </div>
          <div
            style={{
              padding: "8px 16px",
              borderRadius: 8,
              border: `1px solid ${cur.vsl_active ? COLORS.red : COLORS.curb}`,
              backgroundColor: cur.vsl_active ? `${COLORS.red}20` : `${COLORS.road}`,
              color: cur.vsl_active ? COLORS.red : COLORS.textMuted,
              fontSize: 12,
              fontWeight: 700,
              letterSpacing: 2,
            }}
          >
            {cur.vsl_active ? "VSL: ACTIVE (s-3.5m)" : "VSL: INACTIVE"}
          </div>
        </div>

        {/* FSM STATE BADGE */}
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", backgroundColor: `${COLORS.road}CC`, padding: "10px 14px", borderRadius: 8 }}>
          <span style={{ fontSize: 13, color: COLORS.textMuted }}>BEHAVIOR FSM:</span>
          <span style={{ fontSize: 18, fontWeight: 700, color: stateColor, letterSpacing: 2 }}>
            {cur.state}
          </span>
        </div>

        {/* Transition Log */}
        <div style={{ fontSize: 11, color: COLORS.textMuted, lineHeight: 1.4 }}>
          {cur.time < 6.0 && "T+00.0s · Nominal cruise along unmarked rural lane"}
          {cur.time >= 6.0 && cur.time < 12.0 && "T+06.0s · Oncoming rickshaw detected · Corridor constricted < 2.55m"}
          {cur.time >= 12.0 && cur.time < 18.0 && "T+12.0s · VSL engaged upstream · Controlled yield deceleration"}
          {cur.time >= 18.0 && cur.time < 24.0 && "T+18.0s · Holding at stop line (0.0 km/h) · Awaiting clearance"}
          {cur.time >= 24.0 && "T+24.0s · Corridor clear (>2.55m) · Accelerating resume with Ackermann arc"}
        </div>
      </div>

      {/* ── 3. Bottom-Left HUD: Perception & Safety Clearance ─────────────── */}
      <div
        style={{
          position: "absolute",
          bottom: 50,
          left: 50,
          width: 440,
          padding: "18px 24px",
          borderRadius: 12,
          border: `1.5px solid ${COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 8,
          fontSize: 13,
        }}
      >
        <div style={{ color: COLORS.cyan, fontWeight: 700, letterSpacing: 2, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 4 }}>
          PERCEPTION & COLLISION ENVELOPES
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>NEAREST HAZARD:</span>
          <span style={{ color: COLORS.text, fontWeight: 700 }}>{cur.nearest_hazard_dist_m.toFixed(1)} m</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>TIME-TO-COLLISION:</span>
          <span style={{ color: cur.ttc_s < 3.0 ? COLORS.amber : COLORS.green, fontWeight: 700 }}>
            {cur.ttc_s >= 90 ? "CLEAR (>10s)" : `${cur.ttc_s.toFixed(1)} s`}
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>BRAKE PRESSURE:</span>
          <span style={{ color: cur.brake_pct > 0 ? COLORS.red : COLORS.textMuted, fontWeight: 700 }}>
            {cur.brake_pct.toFixed(0)} %
          </span>
        </div>
      </div>

      {/* ── 4. Bottom-Right HUD: Real-Time Compute & Profiling ───────────── */}
      <div
        style={{
          position: "absolute",
          bottom: 50,
          right: 50,
          width: 480,
          padding: "18px 24px",
          borderRadius: 12,
          border: `1.5px solid ${COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 8,
          fontSize: 13,
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 4 }}>
          <span style={{ color: COLORS.cyan, fontWeight: 700, letterSpacing: 2 }}>COMPUTE PROFILE (REAL-TIME)</span>
          <span style={{ color: COLORS.green, fontWeight: 700 }}>50 Hz CLOSED LOOP</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>CYCLE PLANNING LATENCY:</span>
          <span style={{ color: COLORS.green, fontWeight: 700 }}>
            {cur.plan_latency_ms.toFixed(1)} ms <span style={{ fontSize: 11, color: COLORS.textMuted }}>(BUDGET: 20.0 ms)</span>
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>HARDWARE BENCHMARK:</span>
          <span style={{ color: COLORS.text, fontWeight: 600 }}>{TARGET_HARDWARE}</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>PROPRIETARY TOOLBOXES:</span>
          <span style={{ color: COLORS.cyan, fontWeight: 700 }}>ZERO (FIRST-PRINCIPLES C++20)</span>
        </div>
      </div>

      {/* Centerpiece Proof Banner */}
      <div
        style={{
          position: "absolute",
          bottom: 15,
          left: "50%",
          transform: "translateX(-50%)",
          fontFamily: "'Courier New', monospace",
          fontSize: 12,
          color: COLORS.textMuted,
          letterSpacing: 2,
        }}
      >
        SOURCE: validation/centerpiece_yield_trace.json · CONTINUOUS 10 Hz TELEMETRY LOG
      </div>
    </AbsoluteFill>
  );
};
