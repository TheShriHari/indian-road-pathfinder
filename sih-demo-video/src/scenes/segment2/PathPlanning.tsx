import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 820;

// Spline control points (approximate S-curve around obstacles)
const SPLINE_POINTS = [
  { x: 960, y: 820 },
  { x: 940, y: 720 },
  { x: 870, y: 620 },
  { x: 880, y: 520 },
  { x: 920, y: 420 },
  { x: 960, y: 320 },
  { x: 980, y: 220 },
];

// Build a polyline path string from points
function buildPath(pts: { x: number; y: number }[]): string {
  return pts.map((p, i) => `${i === 0 ? "M" : "L"} ${p.x} ${p.y}`).join(" ");
}

// Candidate ghost paths (slight variations)
const CANDIDATE_PATHS = [
  [
    { x: 960, y: 820 }, { x: 1000, y: 700 }, { x: 1050, y: 580 },
    { x: 1020, y: 460 }, { x: 990, y: 350 }, { x: 960, y: 220 },
  ],
  [
    { x: 960, y: 820 }, { x: 920, y: 700 }, { x: 850, y: 580 },
    { x: 840, y: 470 }, { x: 880, y: 360 }, { x: 940, y: 240 },
  ],
  [
    { x: 960, y: 820 }, { x: 970, y: 690 }, { x: 990, y: 560 },
    { x: 980, y: 440 }, { x: 960, y: 330 }, { x: 950, y: 230 },
  ],
];

export const PathPlanning: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Candidate paths fade out 5–30
  const candidateOpacity = interpolate(frame, [5, 30], [0.5, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Selected path draws across 45 frames (20 to 65) via strokeDashoffset
  const SPLINE_LENGTH = 620; // approximate total length
  const splineProgress = interpolate(frame, [20, 65], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const dashOffset = SPLINE_LENGTH * (1 - splineProgress);

  // Curvature guide arc visibility
  const arcOpacity = interpolate(frame, [55, 80], [0, 0.5], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Label opacity
  const labelOpacity = interpolate(frame, [60, 85], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Grid */}
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}

        {/* Road */}
        <rect x={700} y={0} width={520} height={1080} fill="#161E2E" />
        <rect x={700} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />
        <rect x={1212} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />

        {/* Obstacles */}
        <ellipse cx={820} cy={480} rx={55} ry={32} fill={`${COLORS.amber}25`} stroke={COLORS.amber} strokeWidth={1.5} />
        <text x={820} y={484} fill={COLORS.amber} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="middle">COW</text>

        <rect x={1060} y={445} width={55} height={65} rx={4}
          fill={`${COLORS.cyan}18`} stroke={COLORS.cyan} strokeWidth={1.5} />
        <text x={1087} y={480} fill={COLORS.cyan} fontSize={13} fontFamily="'Courier New', monospace" textAnchor="middle">RICK.</text>

        {/* Potholes */}
        <ellipse cx={900} cy={620} rx={20} ry={12} fill="#0B0F19" stroke="#334155" strokeWidth={2} />
        <ellipse cx={1050} cy={560} rx={15} ry={9} fill="#0B0F19" stroke="#334155" strokeWidth={2} />

        {/* Candidate paths (ghost, fading out) */}
        {CANDIDATE_PATHS.map((pts, i) => (
          <path
            key={i}
            d={buildPath(pts)}
            fill="none"
            stroke={COLORS.grid}
            strokeWidth={2.5}
            strokeDasharray="8 6"
            opacity={candidateOpacity * 0.8}
          />
        ))}

        {/* Selected optimal path */}
        <path
          d={buildPath(SPLINE_POINTS)}
          fill="none"
          stroke={COLORS.green}
          strokeWidth={4}
          strokeLinecap="round"
          strokeLinejoin="round"
          strokeDasharray={SPLINE_LENGTH}
          strokeDashoffset={dashOffset}
          style={{ filter: `drop-shadow(0 0 6px ${COLORS.green}80)` }}
        />

        {/* Path direction arrows along the selected path */}
        {splineProgress > 0.6 &&
          SPLINE_POINTS.slice(1, -1).map((pt, i) => {
            const prev = SPLINE_POINTS[i];
            const angle = Math.atan2(pt.y - prev.y, pt.x - prev.x) * (180 / Math.PI);
            return (
              <g key={i} transform={`translate(${pt.x}, ${pt.y}) rotate(${angle})`}
                opacity={interpolate(frame, [50 + i * 4, 65 + i * 4], [0, 0.7], {
                  extrapolateLeft: "clamp",
                  extrapolateRight: "clamp",
                })}>
                <polygon points="-6,-4 6,0 -6,4" fill={COLORS.green} />
              </g>
            );
          })
        }

        {/* Curvature guide arc (tightest bend, ghosted) */}
        <circle
          cx={878}
          cy={520}
          r={90}
          fill="none"
          stroke={COLORS.green}
          strokeWidth={1.5}
          strokeDasharray="10 8"
          opacity={arcOpacity}
        />
        <text x={860} y={420} fill={COLORS.green} fontSize={14}
          fontFamily="'Courier New', monospace" textAnchor="middle" opacity={arcOpacity}>
          TURNING RADIUS
        </text>
        <text x={860} y={438} fill={COLORS.textMuted} fontSize={12}
          fontFamily="'Courier New', monospace" textAnchor="middle" opacity={arcOpacity}>
          kinematically feasible
        </text>

        {/* Selected path label */}
        <text x={780} y={290} fill={COLORS.green} fontSize={20}
          fontFamily="'Courier New', monospace" opacity={labelOpacity}
          fontWeight="bold">
          ✓ OPTIMAL PATH SELECTED
        </text>
      </svg>

      {/* Vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} />

      {/* HUD */}
      <HudLabel
        text="PATH PLANNING · kinematically feasible route"
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
        STEP 5 / 7
      </div>
    </AbsoluteFill>
  );
};
