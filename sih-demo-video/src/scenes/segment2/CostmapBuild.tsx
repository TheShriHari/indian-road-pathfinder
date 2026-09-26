import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 820;
const CELL_SIZE = 22;
const GRID_COLS = 50;
const GRID_ROWS = 36;
const GRID_LEFT = (1920 - GRID_COLS * CELL_SIZE) / 2;
const GRID_TOP = (1080 - GRID_ROWS * CELL_SIZE) / 2;

const HAZARDS = [
  { cx: 800, cy: 490, label: "CATTLE BLOOM", color: COLORS.amber },
  { cx: 1080, cy: 470, label: "RICKSHAW BLOOM", color: COLORS.orange },
  { cx: 940, cy: 620, label: "POTHOLE", color: COLORS.red },
] as const;

export const CostmapBuild: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Cellular bloom expansion (0 to 7s, frames 0 to 210)
  const costBloomRadius = interpolate(frame, [20, 210], [0, 100], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // At 7s to 11s (frames 210 to 330), the heat cells narrow toward a pinch point
  // creating the exact pinch corridor measured in Beat 6
  const pinchNarrow = interpolate(frame, [210, 330], [0, 55], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity, overflow: "hidden" }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Road */}
        <rect x={680} y={0} width={560} height={1080} fill={COLORS.road} />
        <rect x={680} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />
        <rect x={1232} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />

        {/* Dynamic Costmap Grid Cells */}
        {Array.from({ length: GRID_ROWS }).map((_, r) =>
          Array.from({ length: GRID_COLS }).map((_, c) => {
            const cx = GRID_LEFT + c * CELL_SIZE + CELL_SIZE / 2;
            const cy = GRID_TOP + r * CELL_SIZE + CELL_SIZE / 2;

            // Only render cells on or near the road
            if (cx < 640 || cx > 1280) return null;

            let minDist = Infinity;
            for (const h of HAZARDS) {
              const effectiveCX = h.cx + (h.cx > 960 ? -pinchNarrow : pinchNarrow);
              const d = Math.sqrt((cx - effectiveCX) ** 2 + (cy - h.cy) ** 2);
              if (d < minDist) minDist = d;
            }

            const maxReach = costBloomRadius + 20;
            if (minDist > maxReach) return null;

            const costIntensity = Math.max(0, 1 - minDist / maxReach);

            let cellColor: string = COLORS.cyan;
            let cellAlpha = 0.2;

            if (costIntensity > 0.65) {
              cellColor = COLORS.red;
              cellAlpha = 0.85;
            } else if (costIntensity > 0.35) {
              cellColor = COLORS.amber;
              cellAlpha = 0.55;
            }

            return (
              <rect
                key={`${r}-${c}`}
                x={cx - CELL_SIZE / 2 + 1}
                y={cy - CELL_SIZE / 2 + 1}
                width={CELL_SIZE - 2}
                height={CELL_SIZE - 2}
                rx={4}
                fill={cellColor}
                opacity={cellAlpha}
              />
            );
          })
        )}

        {/* Grid lines texture */}
        {Array.from({ length: GRID_ROWS + 1 }).map((_, i) => (
          <line
            key={`gh-${i}`}
            x1={GRID_LEFT}
            y1={GRID_TOP + i * CELL_SIZE}
            x2={GRID_LEFT + GRID_COLS * CELL_SIZE}
            y2={GRID_TOP + i * CELL_SIZE}
            stroke={COLORS.grid}
            strokeWidth={0.5}
            opacity={0.3}
          />
        ))}
        {Array.from({ length: GRID_COLS + 1 }).map((_, i) => (
          <line
            key={`gv-${i}`}
            x1={GRID_LEFT + i * CELL_SIZE}
            y1={GRID_TOP}
            x2={GRID_LEFT + i * CELL_SIZE}
            y2={GRID_TOP + GRID_ROWS * CELL_SIZE}
            stroke={COLORS.grid}
            strokeWidth={0.5}
            opacity={0.3}
          />
        ))}

        {/* Hazard Core Badges */}
        {HAZARDS.map((h, i) => (
          <g key={i}>
            <circle cx={h.cx} cy={h.cy} r={28} fill={`${h.color}30`} stroke={h.color} strokeWidth={2} />
            <text
              x={h.cx}
              y={h.cy - 36}
              fill={h.color}
              fontSize={14}
              fontFamily="'Courier New', monospace"
              textAnchor="middle"
              fontWeight="bold"
            >
              {h.label}
            </text>
          </g>
        ))}

        {/* Pinch corridor convergence guides (setup for Beat 6's calipers) */}
        {pinchNarrow > 10 && (
          <g opacity={pinchNarrow / 55}>
            <line x1={800 + pinchNarrow} y1={360} x2={800 + pinchNarrow} y2={560} stroke={COLORS.amber} strokeWidth={2} strokeDasharray="6 4" />
            <line x1={1080 - pinchNarrow} y1={360} x2={1080 - pinchNarrow} y2={560} stroke={COLORS.amber} strokeWidth={2} strokeDasharray="6 4" />
            <text x={960} y={380} fill={COLORS.amber} fontSize={16} fontFamily="'Courier New', monospace" textAnchor="middle" fontWeight="bold">
              CORRIDOR CONSTRICTION DETECTED
            </text>
          </g>
        )}
      </svg>

      {/* Ego vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} speedKmh={0} color={COLORS.cyan} />

      {/* HUD Label */}
      <HudLabel
        text="LOCAL COSTMAP · MULTI-LAYER CELLULAR OCCUPANCY BLOOM"
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
        STEP 3 / 7
      </div>

      {/* Legend */}
      <div
        style={{
          position: "absolute",
          bottom: 70,
          left: 80,
          display: "flex",
          gap: 28,
          padding: "12px 24px",
          borderRadius: 14,
          backgroundColor: `${COLORS.bg}EE`,
          border: `1px solid ${COLORS.curb}`,
          opacity: hudOpacity,
        }}
      >
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 14, height: 14, borderRadius: 4, backgroundColor: COLORS.cyan }} />
          <span style={{ fontFamily: "'Courier New', monospace", fontSize: 14, color: COLORS.textMuted }}>FREE SPACE</span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 14, height: 14, borderRadius: 4, backgroundColor: COLORS.amber }} />
          <span style={{ fontFamily: "'Courier New', monospace", fontSize: 14, color: COLORS.textMuted }}>BUFFER MARGIN</span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 14, height: 14, borderRadius: 4, backgroundColor: COLORS.red }} />
          <span style={{ fontFamily: "'Courier New', monospace", fontSize: 14, color: COLORS.textMuted }}>LETHAL ZONE</span>
        </div>
      </div>
    </AbsoluteFill>
  );
};
