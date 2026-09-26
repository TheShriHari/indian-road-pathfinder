import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";

export const ClosingBranding: React.FC = () => {
  const frame = useCurrentFrame();

  const titleOpacity = interpolate(frame, [15, 45], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const nextStepsOpacity = interpolate(frame, [60, 90], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const lockupOpacity = interpolate(frame, [160, 200], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Vehicle rolls forward along road centerline
  const egoY = interpolate(frame, [0, 540], [800, 300], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
      {/* Background Grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080} stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}

        {/* Road Surface */}
        <rect x={760} y={0} width={400} height={1080} fill={COLORS.road} />
        <rect x={760} y={0} width={6} height={1080} fill={COLORS.curb} />
        <rect x={1154} y={0} width={6} height={1080} fill={COLORS.curb} />
      </svg>

      {/* Vehicle driving forward */}
      <VehicleSprite x={960} y={egoY} color={COLORS.cyan} wheelAngle={0} />

      {/* Top Center: What's Next & Hardware Scaling */}
      <div
        style={{
          position: "absolute",
          top: 90,
          left: "50%",
          transform: "translateX(-50%)",
          textAlign: "center",
          width: 1400,
          opacity: nextStepsOpacity,
          fontFamily: "'Courier New', monospace",
        }}
      >
        <div style={{ fontSize: 16, color: COLORS.cyan, letterSpacing: 4, fontWeight: 700 }}>
          ROADMAP & PHYSICAL DEPLOYMENT
        </div>
        <div style={{ fontSize: 28, color: COLORS.text, fontWeight: 700, letterSpacing: 2, marginTop: 8 }}>
          Phase 6: CAN-Bus Hardware-in-the-Loop (HIL) Testbed
        </div>
        <div style={{ fontSize: 16, color: COLORS.textMuted, marginTop: 8, lineHeight: 1.5 }}>
          Porting vectorized MATLAB algorithms to standalone C++20 ROS2 nodes for campus test-track evaluation.
        </div>
      </div>

      {/* Center / Team Lockup Card */}
      <div
        style={{
          position: "absolute",
          top: "48%",
          left: "50%",
          transform: "translate(-50%, -50%)",
          textAlign: "center",
          opacity: lockupOpacity,
          width: 1200,
          padding: "36px 48px",
          borderRadius: 16,
          border: `1.5px solid ${COLORS.cyan}60`,
          backgroundColor: `${COLORS.bg}EE`,
          fontFamily: "'Courier New', monospace",
          display: "flex",
          flexDirection: "column",
          gap: 12,
        }}
      >
        <div style={{ fontSize: 14, color: COLORS.textMuted, letterSpacing: 5 }}>
          SMART INDIA HACKATHON 2026 · PROBLEM STATEMENT 26037 (MATHWORKS)
        </div>
        <div style={{ fontSize: 44, fontWeight: 700, color: COLORS.text, letterSpacing: 4 }}>
          OORUM BLOOD
        </div>
        <div style={{ fontSize: 22, color: COLORS.cyan, fontWeight: 600, letterSpacing: 3 }}>
          Indian Institute of Technology Kharagpur
        </div>
        <div style={{ height: 1, backgroundColor: `${COLORS.cyan}40`, margin: "8px 0" }} />
        <div style={{ fontSize: 20, color: COLORS.green, fontWeight: 700, letterSpacing: 2 }}>
          Autonomous Mobility Built for the Roads India Actually Has.
        </div>
      </div>

      {/* Bottom Rigor Verification Stamp */}
      <div
        style={{
          position: "absolute",
          bottom: 40,
          left: "50%",
          transform: "translateX(-50%)",
          fontFamily: "'Courier New', monospace",
          fontSize: 13,
          color: COLORS.textMuted,
          letterSpacing: 2,
          opacity: titleOpacity,
        }}
      >
        CODEBASE: github.com/TheShriHari/sih_demo_video · ALL DATA TRACEABLE TO VERIFIED MAT/CSV LOGS
      </div>
    </AbsoluteFill>
  );
};
