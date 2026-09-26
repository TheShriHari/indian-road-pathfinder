import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 750;

export const MotionPrediction: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [15, 40], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Arrows scale 0→1 over frames 10–40
  const arrowScale = interpolate(frame, [10, 40], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Ribbons draw via dashoffset over frames 40–120
  const rickshawRibbonProgress = interpolate(frame, [40, 120], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const cowRibbonProgress = interpolate(frame, [50, 130], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Covariance ellipse animation (grows larger)
  const ellipseScale = interpolate(frame, [60, 150], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Rickshaw position (right side, predictable)
  const rickX = 1100;
  const rickY = 520;
  const rickDX = -20;
  const rickDY = 140;
  const rickRibbonLen = 200;

  // Cow position (left-center, more uncertain)
  const cowX = 820;
  const cowY = 480;
  const cowDX = 30;
  const cowDY = 160;
  const cowRibbonLen = 200;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity }}>
      {/* Background grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.35} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.35} />
        ))}

        {/* Road */}
        <rect x={700} y={0} width={520} height={1080} fill="#161E2E" />
        <rect x={700} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />
        <rect x={1212} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />

        {/* ---- RICKSHAW trajectory (predictable, narrow ribbon) ---- */}
        {/* Ribbon — dashoffset animation */}
        <defs>
          <linearGradient id="rickGrad" x1={rickX} y1={rickY} x2={rickX + rickDX * 2} y2={rickY + rickDY * 2} gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={COLORS.cyan} stopOpacity={0.8} />
            <stop offset="100%" stopColor={COLORS.cyan} stopOpacity={0.15} />
          </linearGradient>
          <linearGradient id="cowGrad" x1={cowX} y1={cowY} x2={cowX + cowDX * 2} y2={cowY + cowDY * 2} gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={COLORS.amber} stopOpacity={0.7} />
            <stop offset="100%" stopColor={COLORS.amber} stopOpacity={0.1} />
          </linearGradient>
        </defs>

        {/* Rickshaw ribbon */}
        <line
          x1={rickX} y1={rickY}
          x2={rickX + rickDX * 1.5} y2={rickY + rickDY * 1.5}
          stroke="url(#rickGrad)"
          strokeWidth={30}
          strokeLinecap="round"
          strokeDasharray={rickshawRibbonProgress > 0 ? rickRibbonLen : 0}
          strokeDashoffset={(1 - rickshawRibbonProgress) * rickRibbonLen}
          opacity={0.7}
        />

        {/* Rickshaw velocity arrow */}
        <g transform={`translate(${rickX}, ${rickY}) scale(${arrowScale})`} style={{ transformOrigin: `${rickX}px ${rickY}px` }}>
          <line
            x1={0} y1={0}
            x2={rickDX * 0.8} y2={rickDY * 0.8}
            stroke={COLORS.cyan}
            strokeWidth={4}
          />
          <polygon
            points={`${rickDX * 0.8 - 6},${rickDY * 0.8 - 10} ${rickDX * 0.8 + 6},${rickDY * 0.8 - 10} ${rickDX * 0.8},${rickDY * 0.8}`}
            fill={COLORS.cyan}
          />
        </g>

        {/* Rickshaw covariance ellipse (small, tight) */}
        <ellipse
          cx={rickX + rickDX * 1.5}
          cy={rickY + rickDY * 1.5}
          rx={15 * ellipseScale}
          ry={25 * ellipseScale}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={1.5}
          strokeDasharray="4 4"
          opacity={0.7 * ellipseScale}
        />

        {/* Rickshaw object marker */}
        <rect x={rickX - 30} y={rickY - 45} width={60} height={50}
          rx={4} fill={`${COLORS.cyan}20`} stroke={COLORS.cyan} strokeWidth={1.5} />
        <text x={rickX} y={rickY - 52} fill={COLORS.cyan} fontSize={16}
          fontFamily="'Courier New', monospace" textAnchor="middle">AUTO-RICKSHAW</text>

        {/* ---- COW trajectory (uncertain, wide ribbon) ---- */}
        {/* Cow ribbon — wobblier, wider */}
        <line
          x1={cowX} y1={cowY}
          x2={cowX + cowDX * 1.5} y2={cowY + cowDY * 1.5}
          stroke="url(#cowGrad)"
          strokeWidth={55}
          strokeLinecap="round"
          strokeDasharray={cowRibbonProgress > 0 ? cowRibbonLen : 0}
          strokeDashoffset={(1 - cowRibbonProgress) * cowRibbonLen}
          opacity={0.5}
        />

        {/* Cow velocity arrow */}
        <g transform={`translate(${cowX}, ${cowY}) scale(${arrowScale})`} style={{ transformOrigin: `${cowX}px ${cowY}px` }}>
          <line
            x1={0} y1={0}
            x2={cowDX * 0.6} y2={cowDY * 0.6}
            stroke={COLORS.amber}
            strokeWidth={5}
          />
          <polygon
            points={`${cowDX * 0.6 - 8},${cowDY * 0.6 - 12} ${cowDX * 0.6 + 8},${cowDY * 0.6 - 12} ${cowDX * 0.6},${cowDY * 0.6}`}
            fill={COLORS.amber}
          />
        </g>

        {/* Cow covariance ellipse (large, dashed — high uncertainty) */}
        <ellipse
          cx={cowX + cowDX * 1.5}
          cy={cowY + cowDY * 1.5}
          rx={40 * ellipseScale}
          ry={50 * ellipseScale}
          fill="none"
          stroke={COLORS.amber}
          strokeWidth={2}
          strokeDasharray="8 6"
          opacity={0.8 * ellipseScale}
        />

        {/* Cow object marker */}
        <ellipse cx={cowX} cy={cowY} rx={38} ry={24}
          fill={`${COLORS.amber}20`} stroke={COLORS.amber} strokeWidth={1.5} />
        <text x={cowX} y={cowY - 32} fill={COLORS.amber} fontSize={16}
          fontFamily="'Courier New', monospace" textAnchor="middle">LARGE ANIMAL</text>

        {/* Uncertainty label for cow */}
        <text
          x={cowX + cowDX * 1.5 + 50}
          y={cowY + cowDY * 1.5}
          fill={COLORS.amber}
          fontSize={14}
          fontFamily="'Courier New', monospace"
          opacity={ellipseScale}
        >
          HIGH UNCERTAINTY
        </text>

        {/* Certainty label for rickshaw */}
        <text
          x={rickX + rickDX * 1.5 + 20}
          y={rickY + rickDY * 1.5}
          fill={COLORS.cyan}
          fontSize={14}
          fontFamily="'Courier New', monospace"
          opacity={ellipseScale}
        >
          LOW UNCERTAINTY
        </text>
      </svg>

      {/* Vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} />

      {/* HUD */}
      <HudLabel
        text="PREDICTION · per-agent motion model · uncertainty-aware"
        x={60}
        y={60}
        opacity={hudOpacity}
        fontSize={24}
        color={COLORS.cyan}
      />

      <div style={{
        position: "absolute",
        top: 60,
        right: 80,
        padding: "10px 24px",
        border: `1.5px solid ${COLORS.cyan}60`,
        borderRadius: 6,
        backgroundColor: `${COLORS.cyan}10`,
        fontFamily: "'Courier New', monospace",
        fontSize: 18,
        color: COLORS.cyan,
        letterSpacing: 3,
        opacity: hudOpacity,
      }}>
        STEP 2 / 7
      </div>
    </AbsoluteFill>
  );
};
