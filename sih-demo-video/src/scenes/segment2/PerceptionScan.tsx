import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, PX_PER_M, SPRING_PRESETS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;
const VEHICLE_Y = 750;
const RANGE_M = 35;
const RANGE_PX = RANGE_M * PX_PER_M; // 280px
const FOV_DEG = 140;

function scanConePath(cx: number, cy: number, r: number, startAngle: number, endAngle: number): string {
  const toRad = (d: number) => ((d - 90) * Math.PI) / 180;
  const x1 = cx + r * Math.cos(toRad(startAngle));
  const y1 = cy + r * Math.sin(toRad(startAngle));
  const x2 = cx + r * Math.cos(toRad(endAngle));
  const y2 = cy + r * Math.sin(toRad(endAngle));
  const largeArc = endAngle - startAngle > 180 ? 1 : 0;
  return `M ${cx} ${cy} L ${x1} ${y1} A ${r} ${r} 0 ${largeArc} 1 ${x2} ${y2} Z`;
}

// Bounding box detection objects (positions match Beat 4 and 5)
const DETECTIONS = [
  { id: "cow",      label: "STRAY CATTLE",  x: 800,  y: 490, triggerFrame: 100, color: COLORS.amber, w: 70, h: 48, vx: -20, vy: 40 },
  { id: "rickshaw", label: "AUTO-RICKSHAW", x: 1080, y: 470, triggerFrame: 160, color: COLORS.orange, w: 62, h: 72, vx: -15, vy: 120 },
  { id: "pothole",  label: "POTHOLE DEEP",  x: 940,  y: 600, triggerFrame: 70,  color: COLORS.red,    w: 52, h: 36, vx: 0,   vy: 0 },
] as const;

export const PerceptionScan: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Scan angle sweeps from -70° to +70° over frames 20 to 240 (8s)
  const scanAngle = interpolate(frame, [20, 240], [-70, 70], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Concentric ripple pulse radiating from roof sensor
  const rippleRadius = (frame * 6) % RANGE_PX;
  const rippleOpacity = interpolate(rippleRadius, [0, RANGE_PX * 0.8, RANGE_PX], [0.8, 0.4, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

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

        {/* Outer 35m Range ring */}
        <circle
          cx={VEHICLE_X}
          cy={VEHICLE_Y}
          r={RANGE_PX}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={1.8}
          strokeDasharray="8 6"
          opacity={0.5}
        />

        {/* Concentric expanding pulse wave */}
        <circle
          cx={VEHICLE_X}
          cy={VEHICLE_Y}
          r={rippleRadius}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={2}
          opacity={rippleOpacity}
        />

        {/* Full 140° FOV ghost arc */}
        <path
          d={scanConePath(VEHICLE_X, VEHICLE_Y, RANGE_PX, -FOV_DEG / 2, FOV_DEG / 2)}
          fill={`${COLORS.cyan}0A`}
          stroke={`${COLORS.cyan}30`}
          strokeWidth={1.5}
        />

        {/* Active sweep cone */}
        <path
          d={scanConePath(VEHICLE_X, VEHICLE_Y, RANGE_PX, -FOV_DEG / 2, scanAngle)}
          fill={`${COLORS.cyan}25`}
          stroke={COLORS.cyan}
          strokeWidth={2}
        />

        {/* Leading sweep laser ray */}
        {(() => {
          const rad = ((scanAngle - 90) * Math.PI) / 180;
          return (
            <line
              x1={VEHICLE_X}
              y1={VEHICLE_Y}
              x2={VEHICLE_X + RANGE_PX * Math.cos(rad)}
              y2={VEHICLE_Y + RANGE_PX * Math.sin(rad)}
              stroke={COLORS.cyan}
              strokeWidth={3}
              style={{ filter: `drop-shadow(0 0 8px ${COLORS.cyan})` }}
            />
          );
        })()}

        {/* Range label */}
        <text
          x={VEHICLE_X + RANGE_PX + 14}
          y={VEHICLE_Y - 10}
          fill={COLORS.cyan}
          fontSize={20}
          fontFamily="'Courier New', monospace"
          opacity={hudOpacity}
          fontWeight="bold"
        >
          35m RANGE
        </text>

        {/* Persistent Bounding Box Detections (carried forward into Beat 4!) */}
        {DETECTIONS.map((det) => {
          const isTriggered = frame >= det.triggerFrame;
          const pop = spring({
            frame: isTriggered ? frame - det.triggerFrame : 0,
            fps: FPS,
            config: SPRING_PRESETS.overshoot,
          });
          const scale = isTriggered ? interpolate(pop, [0, 1], [0.4, 1.0]) : 0;
          const arrowLength = isTriggered ? interpolate(frame - det.triggerFrame, [10, 40], [0, 1], { extrapolateLeft: "clamp", extrapolateRight: "clamp" }) : 0;

          if (!isTriggered) return null;

          return (
            <g key={det.id} transform={`translate(${det.x}, ${det.y}) scale(${scale})`} style={{ transformOrigin: "center center" }}>
              {/* Bounding box with 14px squircle radius */}
              <rect
                x={-det.w / 2}
                y={-det.h / 2}
                width={det.w}
                height={det.h}
                rx={14}
                fill={`${det.color}25`}
                stroke={det.color}
                strokeWidth={2.5}
                style={{ filter: `drop-shadow(0 0 10px ${det.color}80)` }}
              />

              {/* Center point */}
              <circle cx={0} cy={0} r={5} fill={det.color} />

              {/* Label */}
              <text
                x={0}
                y={-det.h / 2 - 12}
                fill={det.color}
                fontSize={15}
                fontFamily="'Courier New', monospace"
                textAnchor="middle"
                fontWeight="bold"
              >
                {det.label}
              </text>

              {/* Sprouting velocity vector */}
              {arrowLength > 0 && (det.vx !== 0 || det.vy !== 0) && (
                <g opacity={arrowLength}>
                  <line
                    x1={0}
                    y1={0}
                    x2={det.vx * arrowLength}
                    y2={det.vy * arrowLength}
                    stroke={det.color}
                    strokeWidth={3}
                  />
                  <polygon
                    points={`${det.vx * arrowLength - 5},${det.vy * arrowLength - 8} ${det.vx * arrowLength + 5},${det.vy * arrowLength - 8} ${det.vx * arrowLength},${det.vy * arrowLength}`}
                    fill={det.color}
                  />
                </g>
              )}
            </g>
          );
        })}
      </svg>

      {/* Vehicle sprite docked at sensor pod origin */}
      <VehicleSprite x={VEHICLE_X} y={VEHICLE_Y} speedKmh={0} color={COLORS.cyan} />

      {/* HUD Labels */}
      <HudLabel
        text="PERCEPTION · 360° SENSOR FUSION · 35m RANGE · 140° FOV"
        x={60}
        y={80}
        opacity={hudOpacity}
        fontSize={22}
        color={COLORS.cyan}
      />
      <HudLabel
        text={`SWEEP ANGLE · ${scanAngle.toFixed(0)}° AZIMUTH`}
        x={60}
        y={118}
        opacity={hudOpacity}
        fontSize={16}
        color={COLORS.textMuted}
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
        STEP 1 / 7
      </div>
    </AbsoluteFill>
  );
};
