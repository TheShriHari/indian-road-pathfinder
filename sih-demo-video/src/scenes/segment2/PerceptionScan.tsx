import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, PX_PER_M } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 750;
const RANGE_M = 35;
const RANGE_PX = RANGE_M * PX_PER_M; // 280px
const FOV_DEG = 140;

// Compute SVG arc path for the scan cone
function scanConePath(
  cx: number,
  cy: number,
  r: number,
  startAngle: number,
  endAngle: number
): string {
  // Angles measured from "up" (north), positive = clockwise
  const toRad = (d: number) => ((d - 90) * Math.PI) / 180;
  const x1 = cx + r * Math.cos(toRad(startAngle));
  const y1 = cy + r * Math.sin(toRad(startAngle));
  const x2 = cx + r * Math.cos(toRad(endAngle));
  const y2 = cy + r * Math.sin(toRad(endAngle));
  const largeArc = endAngle - startAngle > 180 ? 1 : 0;
  return `M ${cx} ${cy} L ${x1} ${y1} A ${r} ${r} 0 ${largeArc} 1 ${x2} ${y2} Z`;
}

// Objects to detect — angular position (degrees from north, from vehicle)
const DETECTIONS = [
  { id: "pothole1", label: "POTHOLE", x: 860, y: 590, angle: -20, color: COLORS.amber },
  { id: "cow",      label: "LARGE ANIMAL", x: 780, y: 500, angle: -40, color: COLORS.red },
  { id: "rickshaw", label: "VEHICLE",      x: 1060, y: 480, angle: 12,  color: COLORS.cyan },
  { id: "pothole2", label: "POTHOLE", x: 1100, y: 580, angle: 24, color: COLORS.amber },
] as const;

export const PerceptionScan: React.FC = () => {
  const frame = useCurrentFrame();

  // Scan angle sweeps from -70° to +70° (relative to north)
  const scanAngle = interpolate(frame, [10, 150], [-70, 70], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Range arc dash animation
  const arcDash = interpolate(frame, [0, 30], [20, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // HUD opacity
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // The sweep starts at -70 and goes to current scanAngle
  // Show full FOV arc as background, active sweep on top
  const sweepStart = -FOV_DEG / 2;

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
        <rect x={700} y={0} width={520} height={1080} fill="#161E2E" rx={0} />
        <rect x={700} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />
        <rect x={1212} y={0} width={8} height={1080} fill="#334155" opacity={0.5} />

        {/* Range ring — dashed */}
        <circle
          cx={VEHICLE_X}
          cy={VEHICLE_Y}
          r={RANGE_PX}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={1.5}
          strokeDasharray={`${12 - arcDash} ${8 + arcDash}`}
          opacity={0.45}
        />

        {/* Full FOV ghost arc */}
        <path
          d={scanConePath(VEHICLE_X, VEHICLE_Y, RANGE_PX, sweepStart, FOV_DEG / 2)}
          fill={`${COLORS.cyan}08`}
          stroke={`${COLORS.cyan}25`}
          strokeWidth={1}
        />

        {/* Active sweep cone */}
        <path
          d={scanConePath(VEHICLE_X, VEHICLE_Y, RANGE_PX, sweepStart, scanAngle)}
          fill={`${COLORS.cyan}20`}
          stroke={COLORS.cyan}
          strokeWidth={1.5}
        />

        {/* Sweep leading edge line */}
        {(() => {
          const rad = ((scanAngle - 90) * Math.PI) / 180;
          return (
            <line
              x1={VEHICLE_X}
              y1={VEHICLE_Y}
              x2={VEHICLE_X + RANGE_PX * Math.cos(rad)}
              y2={VEHICLE_Y + RANGE_PX * Math.sin(rad)}
              stroke={COLORS.cyan}
              strokeWidth={2}
              opacity={0.9}
            />
          );
        })()}

        {/* Range label */}
        <text
          x={VEHICLE_X + RANGE_PX + 10}
          y={VEHICLE_Y - 8}
          fill={COLORS.cyan}
          fontSize={18}
          fontFamily="'Courier New', monospace"
          opacity={hudOpacity}
        >
          35m
        </text>

        {/* Detection objects */}
        {DETECTIONS.map((det) => {
          const detected = scanAngle >= det.angle;
          const springVal = spring({
            frame: detected ? frame - (
              // approximate frame when scanAngle crossed this angle
              interpolate(det.angle, [-70, 70], [10, 150], { extrapolateLeft: "clamp", extrapolateRight: "clamp" })
            ) : 0,
            fps: FPS,
            config: { damping: 10, stiffness: 120 },
          });
          const scale = detected ? interpolate(springVal, [0, 1], [0, 1]) : 0;
          const objOpacity = detected ? 1 : 0;

          return (
            <g key={det.id} opacity={objOpacity}>
              {/* Object marker */}
              <rect
                x={det.x - 28}
                y={det.y - 20}
                width={56}
                height={40}
                fill={`${det.color}20`}
                stroke={det.color}
                strokeWidth={1.5}
                rx={3}
                transform={`scale(${scale})`}
                style={{ transformOrigin: `${det.x}px ${det.y}px` }}
              />
              {/* Dot center */}
              <circle cx={det.x} cy={det.y} r={5} fill={det.color} />
              {/* Label */}
              <text
                x={det.x}
                y={det.y - 28}
                fill={det.color}
                fontSize={14}
                fontFamily="'Courier New', monospace"
                textAnchor="middle"
                opacity={scale}
              >
                {det.label}
              </text>
            </g>
          );
        })}
      </svg>

      {/* Vehicle */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} />

      {/* HUD labels */}
      <HudLabel
        text="PERCEPTION · 35m range · 140° FOV"
        x={60}
        y={60}
        opacity={hudOpacity}
        fontSize={24}
        color={COLORS.cyan}
      />
      <HudLabel
        text={`SCAN ANGLE · ${scanAngle.toFixed(0)}°`}
        x={60}
        y={100}
        opacity={hudOpacity}
        fontSize={18}
        color={COLORS.textMuted}
      />

      {/* Step badge */}
      <div
        style={{
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
        }}
      >
        STEP 1 / 7
      </div>
    </AbsoluteFill>
  );
};
