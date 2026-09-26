import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS, ASSUMPTION_TIMING } from "../../theme";

interface AssumptionCardProps {
  text: string;
  y: number;
  enterFrame: number;
  strikeStart: number;
  strikeEnd: number;
  frame: number;
}

const AssumptionCard: React.FC<AssumptionCardProps> = ({
  text,
  y,
  enterFrame,
  strikeStart,
  strikeEnd,
  frame,
}) => {
  const cardOpacity = interpolate(frame, [enterFrame, enterFrame + 12], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const cardY = interpolate(frame, [enterFrame, enterFrame + 12], [24, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const strikeWidth = interpolate(frame, [strikeStart, strikeEnd], [0, 100], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const textColor = frame > strikeStart ? COLORS.red : COLORS.text;
  const textOpacity = frame > strikeEnd ? 0.5 : 1;

  return (
    <div
      style={{
        position: "absolute",
        left: "50%",
        top: y,
        transform: `translateX(-50%) translateY(${cardY}px)`,
        opacity: cardOpacity,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        width: 900,
      }}
    >
      {/* Card background */}
      <div
        style={{
          padding: "20px 48px",
          borderRadius: 12,
          border: `1px solid ${frame > strikeStart ? COLORS.red + "60" : COLORS.grid}`,
          backgroundColor: `${frame > strikeStart ? COLORS.red : COLORS.grid}18`,
          position: "relative",
          width: "100%",
          boxSizing: "border-box",
        }}
      >
        <span
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 32,
            color: textColor,
            opacity: textOpacity,
            letterSpacing: 2,
            display: "block",
          }}
        >
          {text}
        </span>

        {/* Strikethrough bar */}
        <div
          style={{
            position: "absolute",
            top: "50%",
            left: 48,
            height: 3,
            width: `${strikeWidth}%`,
            backgroundColor: COLORS.red,
            transform: "translateY(-50%)",
            transformOrigin: "left",
            boxShadow: `0 0 8px ${COLORS.red}`,
          }}
        />
      </div>
    </div>
  );
};

export const AssumptionBreakdown: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Header
  const headerOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        opacity: sceneOpacity,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      {/* Grid lines background */}
      <svg
        width={1920}
        height={1080}
        style={{ position: "absolute", top: 0, left: 0 }}
      >
        {Array.from({ length: 20 }).map((_, i) => (
          <line
            key={`h-${i}`}
            x1={0}
            y1={i * 54}
            x2={1920}
            y2={i * 54}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.4}
          />
        ))}
        {Array.from({ length: 36 }).map((_, i) => (
          <line
            key={`v-${i}`}
            x1={i * 54}
            y1={0}
            x2={i * 54}
            y2={1080}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.4}
          />
        ))}
      </svg>

      {/* Header */}
      <div
        style={{
          position: "absolute",
          top: 180,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: headerOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 20,
          color: COLORS.textMuted,
          letterSpacing: 6,
          textTransform: "uppercase",
        }}
      >
        Existing autonomous systems assume:
      </div>

      {/* Assumption cards */}
      <AssumptionCard
        text="HD Maps · Centimetre-accurate positioning"
        y={280}
        enterFrame={ASSUMPTION_TIMING.card1Enter}
        strikeStart={ASSUMPTION_TIMING.card1Strike.start}
        strikeEnd={ASSUMPTION_TIMING.card1Strike.end}
        frame={frame}
      />
      <AssumptionCard
        text="Painted lanes · Structured road markings"
        y={410}
        enterFrame={ASSUMPTION_TIMING.card2Enter}
        strikeStart={ASSUMPTION_TIMING.card2Strike.start}
        strikeEnd={ASSUMPTION_TIMING.card2Strike.end}
        frame={frame}
      />
      <AssumptionCard
        text="Predictable, rule-following traffic agents"
        y={540}
        enterFrame={ASSUMPTION_TIMING.card3Enter}
        strikeStart={ASSUMPTION_TIMING.card3Strike.start}
        strikeEnd={ASSUMPTION_TIMING.card3Strike.end}
        frame={frame}
      />

      {/* Bottom note */}
      <div
        style={{
          position: "absolute",
          bottom: 160,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: interpolate(frame, [96, 115], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
          fontFamily: "'Courier New', monospace",
          fontSize: 26,
          color: COLORS.cyan,
          letterSpacing: 3,
          textAlign: "center",
          textTransform: "uppercase",
        }}
      >
        Indian roads have none of that.
      </div>
    </AbsoluteFill>
  );
};
