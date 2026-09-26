import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS } from "../../theme";
import { HudLabel } from "../../components/HudLabel";

const KPI_CARDS = [
  {
    id: "trials",
    title: "Over a Thousand",
    titleLine2: "Randomized Trials",
    subtitle: "Exhaustive closed-loop Monte Carlo validation",
    color: COLORS.cyan,
    enterFrame: 20,
    counterMax: 1000,
  },
  {
    id: "toolboxes",
    title: "Zero Proprietary",
    titleLine2: "Toolboxes",
    subtitle: "First-principles vectorized matrix mathematics",
    color: COLORS.green,
    enterFrame: 80,
    counterMax: 0,
  },
  {
    id: "realtime",
    title: "Real-Time Capable",
    titleLine2: "Execution",
    subtitle: "Deterministic replanning within safety budgets",
    color: COLORS.amber,
    enterFrame: 140,
    counterMax: 0,
  },
] as const;

export const RigorKPIs: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const headerOpacity = interpolate(frame, [0, 25], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Background counter (decorative, low opacity, purely visual)
  const bgCounter = interpolate(frame, [0, 300], [0, 1247], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      {/* Grid background */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
      </svg>

      {/* Background decorative counter — purely visual motif, NOT a claimed metric */}
      <div style={{
        position: "absolute",
        top: "50%",
        left: "50%",
        transform: "translate(-50%, -50%)",
        fontFamily: "'Courier New', monospace",
        fontSize: 280,
        fontWeight: 700,
        color: COLORS.cyan,
        opacity: 0.04,
        letterSpacing: -10,
        whiteSpace: "nowrap",
        pointerEvents: "none",
        userSelect: "none",
      }}>
        {Math.floor(bgCounter).toLocaleString()}
      </div>

      {/* Header */}
      <div style={{
        position: "absolute",
        top: 90,
        left: "50%",
        transform: "translateX(-50%)",
        opacity: headerOpacity,
        textAlign: "center",
      }}>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontSize: 20,
          color: COLORS.cyan,
          letterSpacing: 8,
          textTransform: "uppercase",
        }}>
          Engineering Rigor
        </div>
      </div>

      {/* KPI cards row */}
      <div style={{
        display: "flex",
        flexDirection: "row",
        gap: 48,
        alignItems: "stretch",
      }}>
        {KPI_CARDS.map((card) => {
          const sp = spring({
            frame: Math.max(0, frame - card.enterFrame),
            fps: FPS,
            config: { damping: 14, stiffness: 90 },
          });
          const cardY = interpolate(sp, [0, 1], [60, 0]);
          const cardOpacity = interpolate(sp, [0, 0.2], [0, 1]);

          // Accent line width
          const lineWidth = interpolate(
            Math.max(0, frame - (card.enterFrame + 10)),
            [0, 30],
            [0, 1],
            { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
          );

          // Glow pulse
          const glowPulse = (Math.sin((frame + card.enterFrame) * 0.05) + 1) / 2;

          return (
            <div
              key={card.id}
              style={{
                opacity: cardOpacity,
                transform: `translateY(${cardY}px)`,
                width: 400,
                padding: "40px 36px",
                borderRadius: 20,
                border: `1.5px solid ${card.color}50`,
                backgroundColor: `${card.color}08`,
                boxShadow: `0 0 ${20 + glowPulse * 12}px ${card.color}20`,
                display: "flex",
                flexDirection: "column",
                gap: 16,
              }}
            >
              {/* Accent top bar */}
              <div style={{
                height: 3,
                width: `${lineWidth * 100}%`,
                backgroundColor: card.color,
                borderRadius: 2,
                boxShadow: `0 0 8px ${card.color}80`,
              }} />

              {/* Title */}
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontWeight: 700,
                fontSize: 32,
                color: card.color,
                lineHeight: 1.2,
                letterSpacing: 1,
              }}>
                {card.title}
                <br />
                {card.titleLine2}
              </div>

              {/* Divider */}
              <div style={{
                height: 1,
                backgroundColor: `${card.color}30`,
                width: "100%",
              }} />

              {/* Subtitle */}
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 18,
                color: COLORS.textMuted,
                lineHeight: 1.5,
                letterSpacing: 0.5,
              }}>
                {card.subtitle}
              </div>

              {/* Status */}
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 14,
                color: card.color,
                letterSpacing: 4,
                textTransform: "uppercase",
              }}>
                ✓ Verified
              </div>
            </div>
          );
        })}
      </div>

      {/* HUD */}
      <HudLabel
        text="ENGINEERING RIGOR · verified methodology"
        x={60}
        y={60}
        opacity={hudOpacity}
        fontSize={24}
        color={COLORS.cyan}
      />
    </AbsoluteFill>
  );
};
