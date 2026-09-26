import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";

interface BulletCardProps {
  id: string;
  title: string;
  subtitle: string;
  metric: string;
  y: number;
  frame: number;
  enterFrame: number;
}

const BulletCard: React.FC<BulletCardProps> = ({
  id,
  title,
  subtitle,
  metric,
  y,
  frame,
  enterFrame,
}) => {
  const opacity = interpolate(frame, [enterFrame, enterFrame + 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const translateY = interpolate(frame, [enterFrame, enterFrame + 20], [15, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <div
      style={{
        position: "absolute",
        left: "50%",
        top: y,
        transform: `translateX(-50%) translateY(${translateY}px)`,
        opacity,
        width: 1400,
        padding: "24px 36px",
        borderRadius: 14,
        border: `1.5px solid ${COLORS.curb}`,
        backgroundColor: `${COLORS.bg}F0`,
        fontFamily: "'Courier New', monospace",
        display: "flex",
        justifyContent: "space-between",
        alignItems: "center",
      }}
    >
      <div style={{ display: "flex", flexDirection: "column", gap: 6, maxWidth: 960 }}>
        <div style={{ display: "flex", alignItems: "center", gap: 16 }}>
          <span style={{ color: COLORS.cyan, fontWeight: 700, fontSize: 18 }}>{id}</span>
          <span style={{ color: COLORS.text, fontWeight: 700, fontSize: 24, letterSpacing: 1.5 }}>
            {title}
          </span>
        </div>
        <div style={{ color: COLORS.textMuted, fontSize: 16, lineHeight: 1.4, paddingLeft: 46 }}>
          {subtitle}
        </div>
      </div>
      <div
        style={{
          padding: "10px 20px",
          borderRadius: 8,
          backgroundColor: `${COLORS.road}`,
          border: `1px solid ${COLORS.curb}`,
          color: COLORS.amber,
          fontSize: 14,
          fontWeight: 700,
          letterSpacing: 2,
          textAlign: "right",
        }}
      >
        {metric}
      </div>
    </div>
  );
};

export const ProblemFraming: React.FC = () => {
  const frame = useCurrentFrame();

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
      {/* Background Grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52} stroke={COLORS.grid} strokeWidth={1} opacity={0.35} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080} stroke={COLORS.grid} strokeWidth={1} opacity={0.35} />
        ))}
      </svg>

      {/* Header */}
      <div style={{ position: "absolute", top: 80, left: 160, display: "flex", flexDirection: "column", gap: 8 }}>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 16,
            color: COLORS.cyan,
            letterSpacing: 4,
            fontWeight: 700,
          }}
        >
          OPERATIONAL DESIGN DOMAIN (ODD) CHALLENGE
        </div>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 32,
            color: COLORS.text,
            fontWeight: 700,
            letterSpacing: 2,
          }}
        >
          3 Architectural Failure Points in Standard Autonomy Stacks
        </div>
      </div>

      {/* 3 Technical Bullet Cards */}
      <BulletCard
        id="01"
        title="Zero Lane Markings & Eroded Soft Shoulders"
        subtitle="Standard lane-detection CNNs and HD vector priors collapse on unstructured rural roads with variable asphalt widths (3.2m–5.5m) and crumbling dirt edges."
        metric="NO LANE PRIORS"
        y={240}
        frame={frame}
        enterFrame={20}
      />
      <BulletCard
        id="02"
        title="Unstructured, Heterogeneous Dynamic Agents"
        subtitle="Stray cattle exhibit non-directional bimodal pause-and-dart behaviors; oncoming auto-rickshaws frequently squeeze through single-lane pinch corridors."
        metric="HIGH COVARIANCE AGENTS"
        y={430}
        frame={frame}
        enterFrame={110}
      />
      <BulletCard
        id="03"
        title="Non-Holonomic Kinodynamic & Braking Jerk Paradox"
        subtitle="Tight Ackermann turning bounds (R_min = 3.8m) combined with nominal passenger comfort limits (0.80 m/s³) cause standard controllers to rate-choke during sudden obstructions."
        metric="R_min: 3.8m | JERK: 0.8 m/s³"
        y={620}
        frame={frame}
        enterFrame={200}
      />

      {/* Bottom Architectural Thesis Banner */}
      <div
        style={{
          position: "absolute",
          bottom: 80,
          left: "50%",
          transform: "translateX(-50%)",
          width: 1400,
          padding: "16px 28px",
          borderRadius: 12,
          border: `1px solid ${COLORS.cyan}60`,
          backgroundColor: `${COLORS.cyan}10`,
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          fontFamily: "'Courier New', monospace",
          opacity: interpolate(frame, [290, 320], [0, 1], { extrapolateRight: "clamp" }),
        }}
      >
        <span style={{ color: COLORS.cyan, fontWeight: 700, fontSize: 16, letterSpacing: 2 }}>
          SOLUTION: CLOSED-LOOP VECTORIZED NAVIGATION WITH HERO NARROW CORRIDOR DECIDER
        </span>
        <span style={{ color: COLORS.green, fontWeight: 700, fontSize: 15, letterSpacing: 2 }}>
          ZERO EXTERNAL TOOLBOXES
        </span>
      </div>
    </AbsoluteFill>
  );
};
