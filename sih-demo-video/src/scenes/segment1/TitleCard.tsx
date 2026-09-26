import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS } from "../../theme";

export const TitleCard: React.FC = () => {
  const frame = useCurrentFrame();

  // Grid fades in
  const gridOpacity = interpolate(frame, [0, 20], [0, 0.4], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Title springs in
  const titleScale = spring({
    frame,
    fps: FPS,
    config: { damping: 12, stiffness: 80 },
  });
  const titleScaleMapped = interpolate(titleScale, [0, 1], [0.88, 1.0]);

  const titleOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Subtitle fades in after title
  const subtitleOpacity = interpolate(frame, [15, 35], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Badge fades in
  const badgeOpacity = interpolate(frame, [35, 55], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Cyan accent line under title
  const lineWidth = interpolate(frame, [10, 45], [0, 560], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Glow pulse on the badge
  const glowPulse = Math.sin(frame * 0.08) * 0.5 + 0.5;

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        overflow: "hidden",
      }}
    >
      {/* Grid overlay */}
      <svg
        width={1920}
        height={1080}
        style={{ position: "absolute", top: 0, left: 0 }}
      >
        {Array.from({ length: 22 }).map((_, i) => (
          <line
            key={`h-${i}`}
            x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={gridOpacity}
          />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line
            key={`v-${i}`}
            x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={gridOpacity}
          />
        ))}

        {/* Corner decorations */}
        <g opacity={gridOpacity}>
          <line x1={80} y1={80} x2={180} y2={80} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={80} y1={80} x2={80} y2={180} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={1840} y1={80} x2={1740} y2={80} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={1840} y1={80} x2={1840} y2={180} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={80} y1={1000} x2={180} y2={1000} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={80} y1={1000} x2={80} y2={900} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={1840} y1={1000} x2={1740} y2={1000} stroke={COLORS.cyan} strokeWidth={2} />
          <line x1={1840} y1={1000} x2={1840} y2={900} stroke={COLORS.cyan} strokeWidth={2} />
        </g>

        {/* Accent underline */}
        <rect
          x={(1920 - lineWidth) / 2}
          y={530}
          width={lineWidth}
          height={3}
          fill={COLORS.cyan}
          rx={1.5}
          opacity={0.9}
          style={{ filter: `drop-shadow(0 0 6px ${COLORS.cyan})` }}
        />
      </svg>

      {/* System tag */}
      <div
        style={{
          position: "absolute",
          top: 160,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: subtitleOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 8,
          textTransform: "uppercase",
        }}
      >
        PS-26037 · MathWorks SIH 2026
      </div>

      {/* Main title */}
      <div
        style={{
          opacity: titleOpacity,
          transform: `scale(${titleScaleMapped})`,
          textAlign: "center",
          position: "absolute",
          top: 300,
          left: "50%",
          width: 1400,
          marginLeft: -700,
        }}
      >
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontWeight: 700,
            fontSize: 88,
            color: COLORS.cyan,
            letterSpacing: 8,
            textTransform: "uppercase",
            lineHeight: 1.1,
            textShadow: `0 0 40px ${COLORS.cyan}60`,
          }}
        >
          ADAPTIVE PATH PLANNING
        </div>
      </div>

      {/* Subtitle */}
      <div
        style={{
          position: "absolute",
          top: 560,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: subtitleOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 32,
          color: COLORS.textMuted,
          letterSpacing: 3,
          textAlign: "center",
          width: 1200,
        }}
      >
        Map-Free Closed-Loop Navigation for Unstructured Environments
      </div>

      {/* Badge */}
      <div
        style={{
          position: "absolute",
          bottom: 180,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: badgeOpacity,
          padding: "14px 40px",
          border: `1.5px solid ${COLORS.cyan}`,
          borderRadius: 8,
          backgroundColor: `${COLORS.cyan}12`,
          fontFamily: "'Courier New', monospace",
          fontSize: 22,
          color: COLORS.text,
          letterSpacing: 4,
          textTransform: "uppercase",
          whiteSpace: "nowrap",
          boxShadow: `0 0 ${16 + glowPulse * 8}px ${COLORS.cyan}40`,
        }}
      >
        Team Oorum Blood · IIT Kharagpur · PS-26037
      </div>
    </AbsoluteFill>
  );
};
