import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS, MIN_CLEARANCE_M } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 820;

// Road geometry
const ROAD_LEFT = 700;
const ROAD_RIGHT = 1220;

// Pinch point (where obstacles narrow the road)
const PINCH_Y = 400;
const PINCH_LEFT = 820;
const PINCH_RIGHT = 1080;

// Stop line (placed upstream of pinch)
const STOP_LINE_Y = 520;

export const CorridorCheck: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Caliper positions animate inward
  const leftCaliperX = interpolate(frame, [10, 70], [ROAD_LEFT, PINCH_LEFT], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const rightCaliperX = interpolate(frame, [10, 70], [ROAD_RIGHT, PINCH_RIGHT], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Measured width (metres), animates from open road to tight corridor
  const measuredWidth = interpolate(frame, [10, 70], [3.8, 2.3], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Color based on threshold
  const isNarrow = measuredWidth <= MIN_CLEARANCE_M;
  const widthColor = isNarrow ? COLORS.red : COLORS.green;

  // Virtual stop line pulsing
  const vslOpacity = frame > 80 ? (Math.sin(frame * 0.4) > 0 ? 1 : 0.4) : 0;

  // Caliper transition color
  const caliperColor = isNarrow ? COLORS.red : COLORS.amber;

  // Warning sign fade
  const warnOpacity = interpolate(frame, [80, 110], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Grid background */}
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}

        {/* Road */}
        <rect x={ROAD_LEFT} y={0} width={ROAD_RIGHT - ROAD_LEFT} height={1080} fill="#161E2E" />
        <rect x={ROAD_LEFT} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />
        <rect x={ROAD_RIGHT - 8} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />

        {/* Obstacle markers (cow & rickshaw creating the pinch) */}
        {/* Cow (left) */}
        <ellipse cx={PINCH_LEFT - 20} cy={PINCH_Y} rx={60} ry={35}
          fill={`${COLORS.amber}25`} stroke={COLORS.amber} strokeWidth={2} />
        <text x={PINCH_LEFT - 20} y={PINCH_Y + 5}
          fill={COLORS.amber} fontSize={16} fontFamily="'Courier New', monospace"
          textAnchor="middle">COW</text>

        {/* Rickshaw (right) */}
        <rect x={PINCH_RIGHT + 10} y={PINCH_Y - 35} width={60} height={70}
          rx={4} fill={`${COLORS.cyan}20`} stroke={COLORS.cyan} strokeWidth={2} />
        <text x={PINCH_RIGHT + 40} y={PINCH_Y + 5}
          fill={COLORS.cyan} fontSize={13} fontFamily="'Courier New', monospace"
          textAnchor="middle">RICK.</text>

        {/* ---- CALIPERS ---- */}
        {/* Left caliper */}
        <g>
          <line x1={leftCaliperX} y1={PINCH_Y - 80} x2={leftCaliperX} y2={PINCH_Y + 80}
            stroke={caliperColor} strokeWidth={3} />
          {/* Crosshatch tick */}
          <line x1={leftCaliperX} y1={PINCH_Y} x2={leftCaliperX + 20} y2={PINCH_Y}
            stroke={caliperColor} strokeWidth={2} />
        </g>
        {/* Right caliper */}
        <g>
          <line x1={rightCaliperX} y1={PINCH_Y - 80} x2={rightCaliperX} y2={PINCH_Y + 80}
            stroke={caliperColor} strokeWidth={3} />
          {/* Crosshatch tick */}
          <line x1={rightCaliperX} y1={PINCH_Y} x2={rightCaliperX - 20} y2={PINCH_Y}
            stroke={caliperColor} strokeWidth={2} />
        </g>

        {/* Width indicator line */}
        <line x1={leftCaliperX} y1={PINCH_Y} x2={rightCaliperX} y2={PINCH_Y}
          stroke={caliperColor} strokeWidth={1.5} strokeDasharray="6 4" opacity={0.7} />

        {/* Width dimension label */}
        <text
          x={(leftCaliperX + rightCaliperX) / 2}
          y={PINCH_Y - 20}
          fill={widthColor}
          fontSize={28}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          fontWeight="bold"
        >
          {measuredWidth.toFixed(2)}m
        </text>

        {/* Threshold label - shifted upward to y=400 */}
        <text
          x={(leftCaliperX + rightCaliperX) / 2}
          y={400}
          fill={COLORS.textMuted}
          fontSize={16}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          opacity={hudOpacity}
        >
          {`MIN CLEARANCE: ${MIN_CLEARANCE_M}m`}
        </text>

        {/* Virtual stop line — dashed, pulsing */}
        <g opacity={vslOpacity}>
          <line x1={ROAD_LEFT + 8} y1={STOP_LINE_Y} x2={ROAD_RIGHT - 8} y2={STOP_LINE_Y}
            stroke={COLORS.red} strokeWidth={4} strokeDasharray="20 12" />
          <text x={960} y={STOP_LINE_Y + 28}
            fill={COLORS.red} fontSize={18}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            letterSpacing={3}>
            VIRTUAL STOP LINE ENGAGED
          </text>
        </g>

        {/* Warning panel */}
        <g opacity={warnOpacity}>
          <rect x={1350} y={280} width={340} height={100} rx={8}
            fill={`${COLORS.red}18`} stroke={COLORS.red} strokeWidth={1.5} />
          <text x={1520} y={325} fill={COLORS.red} fontSize={20}
            fontFamily="'Courier New', monospace" textAnchor="middle" fontWeight="bold">
            CORRIDOR BLOCKED
          </text>
          <text x={1520} y={358} fill={COLORS.textMuted} fontSize={15}
            fontFamily="'Courier New', monospace" textAnchor="middle">
            Not enough room to pass
          </text>
        </g>
      </svg>

      {/* Vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} />

      {/* HUD */}
      <HudLabel
        text="CORRIDOR CHECK · is there enough room to pass?"
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
        STEP 4 / 7
      </div>

      {/* Width readout */}
      <div style={{
        position: "absolute",
        bottom: 100,
        left: "50%",
        transform: "translateX(-50%)",
        fontFamily: "'Courier New', monospace",
        fontSize: 22,
        color: widthColor,
        letterSpacing: 3,
        opacity: hudOpacity,
      }}>
        {isNarrow ? "⚠ INSUFFICIENT CLEARANCE — HOLDING" : "✓ CLEARANCE OK"}
      </div>
    </AbsoluteFill>
  );
};
