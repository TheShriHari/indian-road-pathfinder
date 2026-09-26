import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, SPRING_PRESETS } from "../../theme";

export const TitleCard: React.FC = () => {
  const frame = useCurrentFrame();

  // Grid fades in
  const gridOpacity = interpolate(frame, [0, 25], [0, 0.35], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Title spring with overshoot (Section 1.5)
  const titleSpring = spring({
    frame,
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });
  const titleScale = interpolate(titleSpring, [0, 1], [0.85, 1.0]);

  const titleOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const subtitleOpacity = interpolate(frame, [25, 45], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const badgeOpacity = interpolate(frame, [45, 65], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Underline expansion
  const lineWidth = interpolate(frame, [15, 60], [0, 680], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Camera push-through transition on exit (frames 135 to 180)
  // Instead of a hard cut, the camera pushes directly THROUGH the title text into the road environment
  const pushZoom = interpolate(frame, [135, 180], [1.0, 2.6], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const pushFade = interpolate(frame, [140, 180], [1.0, 0.0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Emerging road preview behind the title (match cut handoff into Beat 2)
  const roadFadeIn = interpolate(frame, [135, 180], [0.0, 1.0], {
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
        overflow: "hidden",
      }}
    >
      {/* Background road fading in on exit to hand off into Beat 2 */}
      {roadFadeIn > 0 && (
        <svg
          width={1920}
          height={1080}
          style={{ position: "absolute", top: 0, left: 0, opacity: roadFadeIn }}
        >
          <rect x={540} y={0} width={840} height={1080} fill={COLORS.road} />
          <rect x={530} y={0} width={10} height={1080} fill={COLORS.curb} />
          <rect x={1380} y={0} width={10} height={1080} fill={COLORS.curb} />
        </svg>
      )}

      {/* Grid overlay */}
      <svg
        width={1920}
        height={1080}
        style={{ position: "absolute", top: 0, left: 0 }}
      >
        {Array.from({ length: 22 }).map((_, i) => (
          <line
            key={`h-${i}`}
            x1={0}
            y1={i * 52}
            x2={1920}
            y2={i * 52}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={gridOpacity}
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
            opacity={gridOpacity}
          />
        ))}

        {/* Decorative tactical corners */}
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
      </svg>

      {/* Main Title Center Container (subject to camera push-through) */}
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          transform: `scale(${titleScale * pushZoom})`,
          opacity: pushFade,
          transformOrigin: "center center",
        }}
      >
        {/* System Tag */}
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 20,
            color: COLORS.cyan,
            letterSpacing: 8,
            textTransform: "uppercase",
            marginBottom: 24,
            opacity: subtitleOpacity,
          }}
        >
          PS-26037 · MathWorks Smart India Hackathon
        </div>

        {/* Main Title */}
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontWeight: 700,
            fontSize: 84,
            color: COLORS.cyan,
            letterSpacing: 8,
            textTransform: "uppercase",
            textAlign: "center",
            lineHeight: 1.1,
            textShadow: `0 0 40px ${COLORS.cyan}60`,
            opacity: titleOpacity,
          }}
        >
          INDIAN ROAD PATHFINDER
        </div>

        {/* Underline */}
        <div
          style={{
            height: 4,
            width: lineWidth,
            backgroundColor: COLORS.cyan,
            boxShadow: `0 0 12px ${COLORS.cyan}`,
            borderRadius: 2,
            marginTop: 20,
            marginBottom: 24,
          }}
        />

        {/* Subtitle */}
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 30,
            color: COLORS.textMuted,
            letterSpacing: 3,
            textAlign: "center",
            maxWidth: 1300,
            lineHeight: 1.4,
            opacity: subtitleOpacity,
          }}
        >
          Map-Free Closed-Loop Navigation for Unstructured Environments
        </div>
      </div>

      {/* Team Badge */}
      <div
        style={{
          position: "absolute",
          bottom: 160,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: badgeOpacity * pushFade,
          padding: "14px 44px",
          border: `2px solid ${COLORS.cyan}`,
          borderRadius: 16,
          backgroundColor: `${COLORS.bg}EE`,
          boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 15px ${COLORS.cyan}20`,
          fontFamily: "'Courier New', monospace",
          fontSize: 22,
          color: COLORS.text,
          letterSpacing: 4,
          textTransform: "uppercase",
        }}
      >
        Team Oorum Blood · IIT Kharagpur
      </div>
    </AbsoluteFill>
  );
};
