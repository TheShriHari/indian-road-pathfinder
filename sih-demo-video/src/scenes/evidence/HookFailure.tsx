import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import seed438Data from "../../data/seed_438_collision_trace.json";

export const HookFailure: React.FC = () => {
  const frame = useCurrentFrame();

  // 450 frames total = 15.0s
  // Simulation runs from frame 0 to frame 300 (10s), then hold on impact diagnostic (frames 300-450)
  const ticks = seed438Data.data;

  // Map frame (0..300) to tick index (0..ticks.length - 1)
  const tickIndex = Math.min(
    ticks.length - 1,
    Math.floor(interpolate(frame, [0, 270], [0, ticks.length - 1], { extrapolateRight: "clamp" }))
  );
  const currentTick = ticks[tickIndex];

  // Ego position moves from y=850 up to impact point y=380
  const egoY = interpolate(frame, [0, 270], [820, 375], { extrapolateRight: "clamp" });

  const isCrashed = frame >= 270;

  // Staggered title and provenance badges
  const hookTextOpacity = interpolate(frame, [20, 50], [0, 1], { extrapolateRight: "clamp" });
  const hookSubTextOpacity = interpolate(frame, [50, 80], [0, 1], { extrapolateRight: "clamp" });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
      {/* Oscilloscope Grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line
            key={`h-${i}`}
            x1={0}
            y1={i * 52}
            x2={1920}
            y2={i * 52}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.35}
          />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line
            key={`v-${i}`}
            x1={i * 52}
            y1={0}
            x2={i * 52}
            y2={1080}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.35}
          />
        ))}

        {/* Unmarked Road Surface (No Lane Lines) */}
        <rect x={700} y={0} width={520} height={1080} fill={COLORS.road} />
        {/* Eroded dirt shoulders */}
        <rect x={700} y={0} width={6} height={1080} fill={COLORS.curb} strokeDasharray="18 12" />
        <rect x={1214} y={0} width={6} height={1080} fill={COLORS.curb} strokeDasharray="18 12" />

        {/* Stray Cattle Obstacle at (960, 310) */}
        <ellipse
          cx={960}
          cy={310}
          rx={48}
          ry={28}
          fill={`${COLORS.amber}22`}
          stroke={COLORS.amber}
          strokeWidth={2}
        />
        <text
          x={960}
          y={316}
          fill={COLORS.amber}
          fontSize={15}
          fontFamily="'Courier New', monospace"
          fontWeight="bold"
          textAnchor="middle"
        >
          STRAY CATTLE (STATIONARY)
        </text>

        {/* Range line to obstacle */}
        {!isCrashed && (
          <line
            x1={960}
            y1={egoY - 56}
            x2={960}
            y2={338}
            stroke={COLORS.red}
            strokeWidth={1.5}
            strokeDasharray="4 4"
          />
        )}

        {/* Crash Impact Shockwave */}
        {isCrashed && (
          <circle
            cx={960}
            cy={340}
            r={interpolate(frame, [270, 340], [10, 110], { extrapolateRight: "clamp" })}
            fill="none"
            stroke={COLORS.red}
            strokeWidth={3}
            opacity={interpolate(frame, [270, 340], [1, 0], { extrapolateRight: "clamp" })}
          />
        )}
      </svg>

      {/* Ego Vehicle under baseline uniform 0.95 m/s^3 controller */}
      <VehicleSprite
        x={960}
        y={egoY}
        color={isCrashed ? COLORS.red : COLORS.cyan}
        wheelAngle={0}
      />

      {/* Top Center: The Mandatory Hook Statement */}
      <div
        style={{
          position: "absolute",
          top: 70,
          left: "50%",
          transform: "translateX(-50%)",
          textAlign: "center",
          width: 1400,
        }}
      >
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 32,
            fontWeight: 700,
            color: COLORS.text,
            letterSpacing: 2,
            opacity: hookTextOpacity,
            textTransform: "uppercase",
          }}
        >
          Indian roads don't have lanes. Most planners assume they do.
        </div>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 18,
            color: COLORS.red,
            letterSpacing: 3,
            marginTop: 10,
            opacity: hookSubTextOpacity,
            fontWeight: 600,
          }}
        >
          PRE-FIX FORENSIC AUDIT · SEED 438 · RATE-CHOKED JERK LIMITER COLLISION
        </div>
      </div>

      {/* Left HUD Panel: Live Telemetry Oscilloscope (Strictly Real Data from seed_438_collision_trace.json) */}
      <div
        style={{
          position: "absolute",
          top: 220,
          left: 60,
          width: 440,
          padding: "24px 28px",
          borderRadius: 14,
          border: `1.5px solid ${isCrashed ? COLORS.red : COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 12,
        }}
      >
        <div style={{ fontSize: 14, color: COLORS.textMuted, letterSpacing: 2, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 6 }}>
          MOCK RUNNER · TELEMETRY TRACE
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>SIM TIME (t):</span>
          <span style={{ color: COLORS.text, fontWeight: 700 }}>{currentTick.time.toFixed(2)} s</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>EGO SPEED:</span>
          <span style={{ color: isCrashed ? COLORS.red : COLORS.cyan, fontWeight: 700 }}>
            {currentTick.speed_kmh.toFixed(1)} km/h ({currentTick.speed_mps.toFixed(2)} m/s)
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>OBSTACLE DIST:</span>
          <span style={{ color: currentTick.obs_dist_m < 2.0 ? COLORS.red : COLORS.amber, fontWeight: 700 }}>
            {currentTick.obs_dist_m.toFixed(3)} m
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>TIME-TO-COLLISION:</span>
          <span style={{ color: currentTick.ttc_s < 1.0 ? COLORS.red : COLORS.amber, fontWeight: 700 }}>
            {currentTick.ttc_s.toFixed(2)} s
          </span>
        </div>
        <div style={{ height: 1, backgroundColor: COLORS.curb, margin: "4px 0" }} />
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>DEMANDED BRAKE:</span>
          <span style={{ color: COLORS.red, fontWeight: 700 }}>{currentTick.cmd_a.toFixed(2)} m/s²</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>ACTUAL BRAKE:</span>
          <span style={{ color: COLORS.amber, fontWeight: 700 }}>{currentTick.act_a.toFixed(2)} m/s²</span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ color: COLORS.textMuted }}>JERK SLEW CLAMP:</span>
          <span style={{ color: COLORS.red, fontWeight: 700 }}>0.950 m/s³ (UNIFORM)</span>
        </div>
      </div>

      {/* Right HUD Panel: Engineering Diagnosis */}
      <div
        style={{
          position: "absolute",
          top: 220,
          right: 60,
          width: 480,
          padding: "24px 28px",
          borderRadius: 14,
          border: `1.5px solid ${isCrashed ? COLORS.red : COLORS.curb}`,
          backgroundColor: `${COLORS.bg}F0`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 12,
        }}
      >
        <div style={{ fontSize: 14, color: COLORS.red, fontWeight: 700, letterSpacing: 2, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 6 }}>
          ROOT CAUSE FORENSIC DIAGNOSIS
        </div>
        <div style={{ fontSize: 14, color: COLORS.textMuted, lineHeight: 1.5 }}>
          Planner demanded <span style={{ color: COLORS.red }}>-8.95 m/s²</span> emergency braking to avoid collision.
        </div>
        <div style={{ fontSize: 14, color: COLORS.textMuted, lineHeight: 1.5 }}>
          However, legacy controls enforced a uniform <span style={{ color: COLORS.red }}>0.95 m/s³</span> comfort clamp, restricting braking slew to only <span style={{ color: COLORS.amber }}>0.095 m/s² per 100ms</span>.
        </div>
        <div style={{ fontSize: 14, color: COLORS.textMuted, lineHeight: 1.5 }}>
          Actual deceleration was choked at <span style={{ color: COLORS.red }}>-1.67 m/s²</span>, making impact mathematically inevitable.
        </div>

        {isCrashed && (
          <div
            style={{
              marginTop: 10,
              padding: "10px 14px",
              borderRadius: 8,
              backgroundColor: `${COLORS.red}20`,
              border: `1px solid ${COLORS.red}`,
              color: COLORS.red,
              fontWeight: 700,
              fontSize: 15,
              textAlign: "center",
              letterSpacing: 2,
            }}
          >
            COLLISION IMPACT AT 23.2 km/h (6.45 m/s)
          </div>
        )}
      </div>

      {/* Bottom Proof Watermark */}
      <div
        style={{
          position: "absolute",
          bottom: 40,
          left: 60,
          fontFamily: "'Courier New', monospace",
          fontSize: 13,
          color: COLORS.textMuted,
          letterSpacing: 2,
        }}
      >
        SOURCE: matlab/audit_seed_438_forensics.m · MOCK KINEMATICS ENGINE · HEADLESS MATLAB R2026a
      </div>
    </AbsoluteFill>
  );
};
