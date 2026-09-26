import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 750;

// Persistent anchor positions carried from Beat 3
const COW_X = 800;
const COW_Y = 490;
const RICK_X = 1080;
const RICK_Y = 470;

export const MotionPrediction: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Ribbons draw via strokeDashoffset over frames 30 to 180 (1s to 6s)
  const ribbonProgress = interpolate(frame, [30, 180], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Covariance uncertainty ellipse expansion (frames 90 to 240)
  const ellipseScale = interpolate(frame, [90, 240], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Continuous Camera Reframe on exit (frames 320 to 420):
  // Camera pulls up and back into a bird's-eye tactical angle, transitioning directly into Beat 5!
  const tacticalPullback = interpolate(frame, [320, 420], [1.0, 0.85], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const tacticalPanY = interpolate(frame, [320, 420], [0, 40], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const cowRibbonLen = 220;
  const rickRibbonLen = 240;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity, overflow: "hidden" }}>
      {/* Dynamic Tactical Camera Reframe container */}
      <div
        style={{
          width: "100%",
          height: "100%",
          transform: `scale(${tacticalPullback}) translateY(${tacticalPanY}px)`,
          transformOrigin: "center center",
          position: "absolute",
          top: 0,
          left: 0,
        }}
      >
        <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
          <defs>
            <linearGradient id="rickGrad" x1={RICK_X} y1={RICK_Y} x2={RICK_X - 30} y2={RICK_Y + 180} gradientUnits="userSpaceOnUse">
              <stop offset="0%" stopColor={COLORS.orange} stopOpacity={0.8} />
              <stop offset="100%" stopColor={COLORS.orange} stopOpacity={0.15} />
            </linearGradient>
            <linearGradient id="cowGrad" x1={COW_X} y1={COW_Y} x2={COW_X - 40} y2={COW_Y + 160} gradientUnits="userSpaceOnUse">
              <stop offset="0%" stopColor={COLORS.amber} stopOpacity={0.8} />
              <stop offset="100%" stopColor={COLORS.amber} stopOpacity={0.1} />
            </linearGradient>
          </defs>

          {/* Grid */}
          {Array.from({ length: 22 }).map((_, i) => (
            <line
              key={`h-${i}`}
              x1={0}
              y1={i * 52}
              x2={1920}
              y2={i * 52}
              stroke={COLORS.grid}
              strokeWidth={1}
              opacity={0.25}
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
              opacity={0.25}
            />
          ))}

          {/* Road */}
          <rect x={680} y={0} width={560} height={1080} fill={COLORS.road} />
          <rect x={680} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />
          <rect x={1232} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />

          {/* ---- RICKSHAW PREDICTION TRAIL ---- */}
          <line
            x1={RICK_X}
            y1={RICK_Y}
            x2={RICK_X - 25}
            y2={RICK_Y + 180}
            stroke="url(#rickGrad)"
            strokeWidth={32}
            strokeLinecap="round"
            strokeDasharray={rickRibbonLen}
            strokeDashoffset={(1 - ribbonProgress) * rickRibbonLen}
            opacity={0.7}
          />
          {/* Rickshaw small tight covariance ellipse (predictable heading) */}
          <ellipse
            cx={RICK_X - 25}
            cy={RICK_Y + 180}
            rx={18 * ellipseScale}
            ry={28 * ellipseScale}
            fill="none"
            stroke={COLORS.orange}
            strokeWidth={1.8}
            strokeDasharray="4 4"
            opacity={0.8 * ellipseScale}
          />

          {/* Persistent Rickshaw Bounding Box */}
          <g transform={`translate(${RICK_X}, ${RICK_Y})`}>
            <rect x={-31} y={-36} width={62} height={72} rx={14} fill={`${COLORS.orange}25`} stroke={COLORS.orange} strokeWidth={2.5} />
            <text x={0} y={-46} fill={COLORS.orange} fontSize={15} fontFamily="'Courier New', monospace" textAnchor="middle" fontWeight="bold">
              AUTO-RICKSHAW
            </text>
            <text x={45} y={15} fill={COLORS.orange} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="start" opacity={ellipseScale}>
              LOW UNCERTAINTY (LINEAR)
            </text>
          </g>

          {/* ---- COW PREDICTION TRAIL (Wide uncertain fan) ---- */}
          <line
            x1={COW_X}
            y1={COW_Y}
            x2={COW_X - 45}
            y2={COW_Y + 160}
            stroke="url(#cowGrad)"
            strokeWidth={56}
            strokeLinecap="round"
            strokeDasharray={cowRibbonLen}
            strokeDashoffset={(1 - ribbonProgress) * cowRibbonLen}
            opacity={0.55}
          />
          {/* Cow large wide covariance ellipse (stochastic wandering) */}
          <ellipse
            cx={COW_X - 45}
            cy={COW_Y + 160}
            rx={44 * ellipseScale}
            ry={54 * ellipseScale}
            fill="none"
            stroke={COLORS.amber}
            strokeWidth={2}
            strokeDasharray="8 6"
            opacity={0.85 * ellipseScale}
          />

          {/* Persistent Cow Bounding Box */}
          <g transform={`translate(${COW_X}, ${COW_Y})`}>
            <rect x={-35} y={-24} width={70} height={48} rx={14} fill={`${COLORS.amber}25`} stroke={COLORS.amber} strokeWidth={2.5} />
            <text x={0} y={-34} fill={COLORS.amber} fontSize={15} fontFamily="'Courier New', monospace" textAnchor="middle" fontWeight="bold">
              STRAY CATTLE
            </text>
            <text x={-55} y={15} fill={COLORS.amber} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="end" opacity={ellipseScale}>
              HIGH UNCERTAINTY (STOCHASTIC)
            </text>
          </g>
        </svg>

        {/* Ego vehicle */}
        <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} speedKmh={0} color={COLORS.cyan} />
      </div>

      {/* HUD Labels */}
      <HudLabel
        text="MOTION PREDICTION · TIME-HORIZON REACHABLE ENVELOPES"
        x={60}
        y={80}
        opacity={hudOpacity}
        fontSize={22}
        color={COLORS.cyan}
      />

      {/* Step badge */}
      <div
        style={{
          position: "absolute",
          top: 80,
          right: 80,
          padding: "10px 24px",
          border: `1.5px solid ${COLORS.cyan}`,
          borderRadius: 14,
          backgroundColor: `${COLORS.cyan}15`,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 3,
          opacity: hudOpacity,
        }}
      >
        STEP 2 / 7
      </div>
    </AbsoluteFill>
  );
};
