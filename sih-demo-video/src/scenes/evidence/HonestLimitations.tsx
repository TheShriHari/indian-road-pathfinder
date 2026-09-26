import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";

export const HonestLimitations: React.FC = () => {
  const frame = useCurrentFrame();

  const opacity = interpolate(frame, [15, 35], [0, 1], {
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
      </svg>

      {/* Header */}
      <div style={{ position: "absolute", top: 80, left: 160, display: "flex", flexDirection: "column", gap: 8 }}>
        <div style={{ fontFamily: "'Courier New', monospace", fontSize: 16, color: COLORS.amber, letterSpacing: 4, fontWeight: 700 }}>
          HONEST ENGINEERING LIMITATIONS & FAIL-SAFE DEFENSE
        </div>
        <div style={{ fontFamily: "'Courier New', monospace", fontSize: 32, color: COLORS.text, fontWeight: 700, letterSpacing: 2 }}>
          Known Operational Boundaries & Deterministic Mitigations
        </div>
      </div>

      {/* Main Container: 2 Comparative Panels */}
      <div
        style={{
          position: "absolute",
          top: 190,
          left: 160,
          width: 1600,
          display: "flex",
          gap: 40,
          opacity,
        }}
      >
        {/* Left Panel: Stated Limitation */}
        <div
          style={{
            flex: 1,
            padding: "36px 40px",
            borderRadius: 16,
            border: `2px solid ${COLORS.amber}80`,
            backgroundColor: `${COLORS.bg}F8`,
            fontFamily: "'Courier New', monospace",
            display: "flex",
            flexDirection: "column",
            gap: 16,
          }}
        >
          <div style={{ fontSize: 14, color: COLORS.amber, fontWeight: 700, letterSpacing: 3, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 10 }}>
            LIMITATION 01 · SUSTAINED SENSOR OCCLUSION & ODOMETRY SLIP
          </div>
          <div style={{ fontSize: 22, fontWeight: 700, color: COLORS.text, lineHeight: 1.3 }}>
            Prolonged Vision Dropout Beyond 1.2s on Unpaved Gravel
          </div>
          <div style={{ fontSize: 16, color: COLORS.textMuted, lineHeight: 1.6 }}>
            Under severe monsoon conditions or dense dust clouds where camera/lidar confidence drops below 40% for &gt;1.2 seconds, dead-reckoning uncertainty grows non-linearly on low-friction loose gravel ($\mu &lt; 0.35$).
          </div>
          <div style={{ marginTop: "auto", padding: "12px 18px", borderRadius: 8, backgroundColor: `${COLORS.amber}18`, border: `1px solid ${COLORS.amber}50`, color: COLORS.amber, fontSize: 14, fontWeight: 600 }}>
            ⚠ BOUNDARY: We do NOT claim Level 5 autonomy under total sensor blackout.
          </div>
        </div>

        {/* Right Panel: Deterministic Architectural Mitigation */}
        <div
          style={{
            flex: 1,
            padding: "36px 40px",
            borderRadius: 16,
            border: `2px solid ${COLORS.green}80`,
            backgroundColor: `${COLORS.bg}F8`,
            fontFamily: "'Courier New', monospace",
            display: "flex",
            flexDirection: "column",
            gap: 16,
          }}
        >
          <div style={{ fontSize: 14, color: COLORS.green, fontWeight: 700, letterSpacing: 3, borderBottom: `1px solid ${COLORS.curb}`, paddingBottom: 10 }}>
            ARCHITECTURAL MITIGATION · DETERMINISTIC SAFE STOP LATCH
          </div>
          <div style={{ fontSize: 22, fontWeight: 700, color: COLORS.text, lineHeight: 1.3 }}>
            Anti-Chatter Deceleration Clamp to In-Lane Standstill
          </div>
          <div style={{ fontSize: 16, color: COLORS.textMuted, lineHeight: 1.6 }}>
            The system explicitly refuses blind heuristic detours. Upon state covariance violation (d_M &gt; 3.0σ), the Behavior FSM latches a controlled safe stop (a = -3.5 m/s²) within the confirmed corridor clearance, holding pose until sensor trust is restored.
          </div>
          <div style={{ marginTop: "auto", padding: "12px 18px", borderRadius: 8, backgroundColor: `${COLORS.green}18`, border: `1px solid ${COLORS.green}50`, color: COLORS.green, fontSize: 14, fontWeight: 600 }}>
            ✓ DEFENSE: 470 safe stops in 1,000 trials prove fail-safe survivability over blind risk.
          </div>
        </div>
      </div>

      {/* Bottom Rigor Note */}
      <div
        style={{
          position: "absolute",
          bottom: 60,
          left: 160,
          fontFamily: "'Courier New', monospace",
          fontSize: 14,
          color: COLORS.textMuted,
          letterSpacing: 2,
        }}
      >
        FAIL-SAFE VERIFICATION: matlab/behavior_state_machine.m · anti_chatter_brake_latch
      </div>
    </AbsoluteFill>
  );
};
