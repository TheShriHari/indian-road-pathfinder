import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 820;
const CELL_SIZE = 20;
const GRID_COLS = 52;
const GRID_ROWS = 36;
const GRID_LEFT = (1920 - GRID_COLS * CELL_SIZE) / 2;
const GRID_TOP = (1080 - GRID_ROWS * CELL_SIZE) / 2;

// Hazard positions (in grid-cell coordinates, relative to center)
const HAZARDS = [
  { cx: 960, cy: 540, label: "POTHOLE" },
  { cx: 820, cy: 430, label: "COW" },
  { cx: 1100, cy: 460, label: "RICKSHAW" },
] as const;

function lerpColor(a: string, b: string, t: number): string {
  const parse = (hex: string) => [
    parseInt(hex.slice(1, 3), 16),
    parseInt(hex.slice(3, 5), 16),
    parseInt(hex.slice(5, 7), 16),
  ];
  const ca = parse(a);
  const cb = parse(b);
  return `rgb(${Math.round(ca[0] + (cb[0] - ca[0]) * t)},${Math.round(ca[1] + (cb[1] - ca[1]) * t)},${Math.round(ca[2] + (cb[2] - ca[2]) * t)})`;
}

export const CostmapBuild: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Grid fades in
  const gridOpacity = interpolate(frame, [0, 40], [0, 0.6], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Cost bloom radius
  const costRadius = interpolate(frame, [20, 100], [0, 80], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        <defs>
          {HAZARDS.map((h, i) => (
            <radialGradient key={i} id={`costBloom${i}`} cx="50%" cy="50%" r="50%">
              <stop offset="0%" stopColor={COLORS.red} stopOpacity={0.95} />
              <stop offset="35%" stopColor={COLORS.red} stopOpacity={0.8} />
              <stop offset="60%" stopColor={COLORS.amber} stopOpacity={0.6} />
              <stop offset="80%" stopColor={COLORS.cyan} stopOpacity={0.3} />
              <stop offset="100%" stopColor={COLORS.cyan} stopOpacity={0} />
            </radialGradient>
          ))}
        </defs>

        {/* Road */}
        <rect x={700} y={0} width={520} height={1080} fill="#161E2E" />
        <rect x={700} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />
        <rect x={1212} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />

        {/* Cost grid cells (colored by distance to nearest hazard) */}
        {Array.from({ length: GRID_ROWS }).map((_, row) =>
          Array.from({ length: GRID_COLS }).map((_, col) => {
            const cx = GRID_LEFT + col * CELL_SIZE + CELL_SIZE / 2;
            const cy = GRID_TOP + row * CELL_SIZE + CELL_SIZE / 2;

            // Min distance to any hazard
            let minDist = Infinity;
            for (const h of HAZARDS) {
              const d = Math.sqrt((cx - h.cx) ** 2 + (cy - h.cy) ** 2);
              if (d < minDist) minDist = d;
            }

            // Normalize distance to cost
            const maxInfluence = costRadius + 10;
            if (minDist > maxInfluence) return null;

            const t = Math.max(0, 1 - minDist / maxInfluence);
            let cellColor: string;
            let cellOpacity: number;

            if (t > 0.7) {
              cellColor = COLORS.red;
              cellOpacity = 0.85;
            } else if (t > 0.4) {
              const localT = (t - 0.4) / 0.3;
              cellColor = lerpColor(COLORS.amber, COLORS.red, localT);
              cellOpacity = 0.6;
            } else if (t > 0.1) {
              const localT = (t - 0.1) / 0.3;
              cellColor = lerpColor(COLORS.cyan, COLORS.amber, localT);
              cellOpacity = 0.4;
            } else {
              cellColor = COLORS.cyan;
              cellOpacity = 0.15;
            }

            return (
              <rect
                key={`${row}-${col}`}
                x={cx - CELL_SIZE / 2 + 1}
                y={cy - CELL_SIZE / 2 + 1}
                width={CELL_SIZE - 2}
                height={CELL_SIZE - 2}
                fill={cellColor}
                opacity={cellOpacity * gridOpacity}
                rx={1}
              />
            );
          })
        )}

        {/* Grid lines */}
        {Array.from({ length: GRID_ROWS + 1 }).map((_, i) => (
          <line key={`gh-${i}`}
            x1={GRID_LEFT} y1={GRID_TOP + i * CELL_SIZE}
            x2={GRID_LEFT + GRID_COLS * CELL_SIZE} y2={GRID_TOP + i * CELL_SIZE}
            stroke={COLORS.grid} strokeWidth={0.5} opacity={gridOpacity * 0.6} />
        ))}
        {Array.from({ length: GRID_COLS + 1 }).map((_, i) => (
          <line key={`gv-${i}`}
            x1={GRID_LEFT + i * CELL_SIZE} y1={GRID_TOP}
            x2={GRID_LEFT + i * CELL_SIZE} y2={GRID_TOP + GRID_ROWS * CELL_SIZE}
            stroke={COLORS.grid} strokeWidth={0.5} opacity={gridOpacity * 0.6} />
        ))}

        {/* Hazard bloom circles */}
        {HAZARDS.map((h, i) => (
          <g key={i}>
            <circle
              cx={h.cx}
              cy={h.cy}
              r={costRadius}
              fill={`url(#costBloom${i})`}
              opacity={0.55}
            />
            <text x={h.cx} y={h.cy - costRadius - 8}
              fill={COLORS.textMuted} fontSize={14}
              fontFamily="'Courier New', monospace"
              textAnchor="middle"
              opacity={gridOpacity}>
              {h.label}
            </text>
          </g>
        ))}

        {/* Legend */}
        <g opacity={hudOpacity}>
          {[
            { color: COLORS.cyan, label: "LOW COST" },
            { color: COLORS.amber, label: "MED COST" },
            { color: COLORS.red, label: "HIGH COST / LETHAL" },
          ].map((item, i) => (
            <g key={i} transform={`translate(80, ${900 + i * 36})`}>
              <rect width={20} height={20} fill={item.color} rx={3} />
              <text x={30} y={15}
                fill={COLORS.textMuted}
                fontSize={16}
                fontFamily="'Courier New', monospace">
                {item.label}
              </text>
            </g>
          ))}
        </g>
      </svg>

      {/* Vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} />

      {/* HUD */}
      <HudLabel
        text="LOCAL COSTMAP · rolling, ego-centric · updates every tick"
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
        STEP 3 / 7
      </div>
    </AbsoluteFill>
  );
};
