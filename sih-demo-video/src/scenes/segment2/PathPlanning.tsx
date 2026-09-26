import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 820;

// Spline control points (S-curve around obstacles)
const SPLINE_POINTS = [
  { x: 960, y: 820 },
  { x: 940, y: 720 },
  { x: 870, y: 620 },
  { x: 880, y: 520 },
  { x: 920, y: 420 },
  { x: 960, y: 320 },
  { x: 980, y: 200 },
];

function buildPath(pts: { x: number; y: number }[]): string {
  return pts.map((p, i) => `${i === 0 ? "M" : "L"} ${p.x} ${p.y}`).join(" ");
}

// Candidate ghost paths sampled by lattice planner
const CANDIDATE_PATHS = [
  [
    { x: 960, y: 820 }, { x: 1010, y: 700 }, { x: 1060, y: 580 },
    { x: 1030, y: 460 }, { x: 990, y: 340 }, { x: 960, y: 200 },
  ],
  [
    { x: 960, y: 820 }, { x: 910, y: 700 }, { x: 840, y: 580 },
    { x: 830, y: 470 }, { x: 880, y: 350 }, { x: 940, y: 220 },
  ],
  [
    { x: 960, y: 820 }, { x: 970, y: 690 }, { x: 990, y: 560 },
    { x: 980, y: 440 }, { x: 960, y: 320 }, { x: 950, y: 210 },
  ],
];

export const PathPlanning: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Green particles coalescing from dissolved stop line (frames 0 to 45)
  const coalesceProgress = interpolate(frame, [0, 45], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const numParticles = 24;
  const particles = Array.from({ length: numParticles }).map((_, i) => {
    const angle = (i * 137.5 * Math.PI) / 180;
    const startDist = (1 - coalesceProgress) * (180 + (i % 4) * 40);
    const startX = 960 + Math.cos(angle) * startDist;
    const startY = 820 + Math.sin(angle) * startDist;
    return { px: startX, py: startY, opacity: 1 - coalesceProgress * 0.9 };
  });

  // Candidate paths flash and fade inside first 3s (frames 15 to 90)
  const candidateOpacity = interpolate(frame, [15, 35, 75, 90], [0, 0.6, 0.4, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Selected optimal path draws via strokeDashoffset from frame 45 to 270 (9s)
  const SPLINE_LENGTH = 680;
  const splineProgress = interpolate(frame, [45, 270], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const dashOffset = SPLINE_LENGTH * (1 - splineProgress);

  // Curvature guide arc (9s to 12s, frames 270 to 360)
  const arcOpacity = interpolate(frame, [270, 310], [0, 0.7], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const labelOpacity = interpolate(frame, [260, 295], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // In frames 360 to 420, vehicle begins traversing forward along spline
  const egoTraverseProgress = interpolate(frame, [360, 420], [0, 0.15], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const egoY = VEHICLE_Y - egoTraverseProgress * 400;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity, overflow: "hidden" }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
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

        {/* Road */}
        <rect x={680} y={0} width={560} height={1080} fill={COLORS.road} />
        <rect x={680} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />
        <rect x={1232} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />

        {/* Obstacles */}
        <ellipse cx={820} cy={480} rx={55} ry={32} fill={`${COLORS.amber}25`} stroke={COLORS.amber} strokeWidth={1.5} />
        <text x={820} y={485} fill={COLORS.amber} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="middle">
          CATTLE
        </text>

        <rect x={1070} y={420} width={58} height={68} rx={6} fill={`${COLORS.orange}20`} stroke={COLORS.orange} strokeWidth={1.5} />
        <text x={1099} y={460} fill={COLORS.orange} fontSize={13} fontFamily="'Courier New', monospace" textAnchor="middle">
          RICKSHAW
        </text>

        {/* Coalescing particles (frames 0 to 45) */}
        {coalesceProgress < 1 &&
          particles.map((p, i) => (
            <circle
              key={`cp-${i}`}
              cx={p.px}
              cy={p.py}
              r={3.5}
              fill={COLORS.green}
              opacity={p.opacity}
              style={{ filter: `drop-shadow(0 0 6px ${COLORS.green})` }}
            />
          ))}

        {/* Candidate paths (ghost variations) */}
        {CANDIDATE_PATHS.map((pts, i) => (
          <path
            key={i}
            d={buildPath(pts)}
            fill="none"
            stroke={COLORS.curb}
            strokeWidth={2.5}
            strokeDasharray="8 6"
            opacity={candidateOpacity}
          />
        ))}

        {/* Selected optimal trajectory spline */}
        <path
          d={buildPath(SPLINE_POINTS)}
          fill="none"
          stroke={COLORS.green}
          strokeWidth={4.5}
          strokeLinecap="round"
          strokeLinejoin="round"
          strokeDasharray={SPLINE_LENGTH}
          strokeDashoffset={dashOffset}
          style={{ filter: `drop-shadow(0 0 10px ${COLORS.green})` }}
        />

        {/* Direction arrows along drawn spline */}
        {splineProgress > 0.4 &&
          SPLINE_POINTS.slice(1, -1).map((pt, i) => {
            const prev = SPLINE_POINTS[i];
            const angle = Math.atan2(pt.y - prev.y, pt.x - prev.x) * (180 / Math.PI);
            return (
              <g
                key={`arr-${i}`}
                transform={`translate(${pt.x}, ${pt.y}) rotate(${angle})`}
                opacity={interpolate(frame, [80 + i * 20, 110 + i * 20], [0, 0.85], {
                  extrapolateLeft: "clamp",
                  extrapolateRight: "clamp",
                })}
              >
                <polygon points="-7,-5 7,0 -7,5" fill={COLORS.green} />
              </g>
            );
          })}

        {/* Turning radius arc */}
        <circle
          cx={880}
          cy={520}
          r={90}
          fill="none"
          stroke={COLORS.green}
          strokeWidth={1.8}
          strokeDasharray="8 6"
          opacity={arcOpacity}
        />
        <text
          x={880}
          y={415}
          fill={COLORS.green}
          fontSize={15}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          opacity={arcOpacity}
          fontWeight="bold"
        >
          MIN TURNING RADIUS: 3.8m
        </text>
        <text
          x={880}
          y={435}
          fill={COLORS.textMuted}
          fontSize={13}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          opacity={arcOpacity}
        >
          Kinematically Feasible Ackermann Curve
        </text>

        {/* Selected path banner */}
        <text
          x={780}
          y={280}
          fill={COLORS.green}
          fontSize={22}
          fontFamily="'Courier New', monospace"
          opacity={labelOpacity}
          fontWeight="bold"
          style={{ filter: `drop-shadow(0 0 8px ${COLORS.green}80)` }}
        >
          ✓ OPTIMAL TRAJECTORY GENERATED
        </text>
      </svg>

      {/* Ego vehicle positioned at start of spline and begins rolling forward */}
      <VehicleSprite x={VEHICLE_X} y={egoY} speedKmh={egoTraverseProgress > 0 ? 12 : 0} color={COLORS.cyan} />

      {/* HUD Label */}
      <HudLabel
        text="PATH PLANNING · CONTINUOUS CURVATURE SPLINE GENERATION"
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
        STEP 5 / 7
      </div>
    </AbsoluteFill>
  );
};
