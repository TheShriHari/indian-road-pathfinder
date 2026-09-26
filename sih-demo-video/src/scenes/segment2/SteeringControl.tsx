import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, Easing } from "remotion";
import { COLORS, MAX_STEERING_RATE_DEG_S } from "../../theme";
import { HudLabel } from "../../components/HudLabel";
import { VehicleSprite } from "../../components/VehicleSprite";

// Steering gauge constants
const GAUGE_CX = 960;
const GAUGE_CY = 460;
const GAUGE_R = 210;
const MAX_STEER_DEG = 25; // synchronized to MAX_STEERING_RATE_DEG_S

// Pedal bar constants (positioned on left to clear central gauge)
const THROTTLE_X = 380;
const BRAKE_X = 540;
const BAR_Y_TOP = 320;
const BAR_MAX_HEIGHT = 300;

function polarToXY(cx: number, cy: number, r: number, angleDeg: number) {
  const rad = ((angleDeg - 90) * Math.PI) / 180;
  return {
    x: cx + r * Math.cos(rad),
    y: cy + r * Math.sin(rad),
  };
}

export const SteeringControl: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Steering angle smoothly oscillates (S-curve following the planned trajectory)
  const steeringAngle = interpolate(
    frame,
    [0, 60, 140, 220, 300, 360],
    [0, -MAX_STEER_DEG * 0.85, MAX_STEER_DEG * 0.75, -MAX_STEER_DEG * 0.4, MAX_STEER_DEG * 0.2, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Throttle: decelerating through obstacle zone then resuming
  const throttlePct = interpolate(
    frame,
    [0, 120, 220, 320, 360],
    [75, 20, 0, 45, 60],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Brake percentage
  const brakePct = interpolate(
    frame,
    [0, 120, 220, 300, 360],
    [0, 40, 95, 10, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp", easing: Easing.inOut(Easing.ease) }
  );

  const needleAngle = Math.max(-MAX_STEER_DEG, Math.min(MAX_STEER_DEG, steeringAngle));

  // Needle tip and base
  const needleTip = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 20, 180 + needleAngle);
  const needleBase = polarToXY(GAUGE_CX, GAUGE_CY, 28, 180 + needleAngle);

  const TICK_COUNT = 13;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, overflow: "hidden" }}>
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

        {/* ---- STEERING GAUGE ---- */}
        <circle
          cx={GAUGE_CX}
          cy={GAUGE_CY}
          r={GAUGE_R + 24}
          fill={`${COLORS.road}80`}
          stroke={COLORS.curb}
          strokeWidth={1.5}
        />

        {/* Arc track */}
        <path
          d={`M ${GAUGE_CX - GAUGE_R} ${GAUGE_CY} A ${GAUGE_R} ${GAUGE_R} 0 0 1 ${GAUGE_CX + GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={COLORS.curb}
          strokeWidth={16}
          strokeLinecap="round"
        />

        {/* Left side steer (electric cyan) */}
        <path
          d={`M ${GAUGE_CX} ${GAUGE_CY - GAUGE_R} A ${GAUGE_R} ${GAUGE_R} 0 0 0 ${GAUGE_CX - GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={COLORS.cyan}
          strokeWidth={8}
          strokeLinecap="round"
          opacity={0.6}
        />
        {/* Right side steer (emerald green) */}
        <path
          d={`M ${GAUGE_CX} ${GAUGE_CY - GAUGE_R} A ${GAUGE_R} ${GAUGE_R} 0 0 1 ${GAUGE_CX + GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={COLORS.green}
          strokeWidth={8}
          strokeLinecap="round"
          opacity={0.6}
        />

        {/* Tick marks */}
        {Array.from({ length: TICK_COUNT }).map((_, i) => {
          const angle = -90 + (i / (TICK_COUNT - 1)) * 180;
          const inner = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 26, angle);
          const outer = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 4, angle);
          const isMax = i === 0 || i === TICK_COUNT - 1;
          return (
            <line
              key={i}
              x1={inner.x}
              y1={inner.y}
              x2={outer.x}
              y2={outer.y}
              stroke={isMax ? COLORS.red : COLORS.textMuted}
              strokeWidth={isMax ? 3.5 : 1.5}
              opacity={0.8}
            />
          );
        })}

        {/* Needle */}
        <line
          x1={needleBase.x}
          y1={needleBase.y}
          x2={needleTip.x}
          y2={needleTip.y}
          stroke={COLORS.cyan}
          strokeWidth={5}
          strokeLinecap="round"
          style={{ filter: `drop-shadow(0 0 8px ${COLORS.cyan})` }}
        />
        <circle cx={GAUGE_CX} cy={GAUGE_CY} r={14} fill={COLORS.bg} stroke={COLORS.cyan} strokeWidth={2.5} />

        {/* Readout */}
        <text
          x={GAUGE_CX}
          y={GAUGE_CY + 55}
          fill={COLORS.cyan}
          fontSize={36}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          fontWeight="bold"
        >
          {needleAngle.toFixed(1)}°
        </text>
        <text
          x={GAUGE_CX}
          y={GAUGE_CY + 85}
          fill={COLORS.textMuted}
          fontSize={16}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          letterSpacing={3}
        >
          STEERING ANGLE
        </text>

        {/* ---- PEDAL BARS ---- */}
        {/* Throttle bar */}
        <g>
          <text
            x={THROTTLE_X + 35}
            y={BAR_Y_TOP - 20}
            fill={COLORS.green}
            fontSize={18}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            letterSpacing={2}
          >
            THROTTLE
          </text>
          <rect x={THROTTLE_X} y={BAR_Y_TOP} width={70} height={BAR_MAX_HEIGHT} rx={12} fill={`${COLORS.road}CC`} />
          <rect
            x={THROTTLE_X}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT * (1 - throttlePct / 100)}
            width={70}
            height={BAR_MAX_HEIGHT * (throttlePct / 100)}
            rx={12}
            fill={COLORS.green}
            style={{ filter: `drop-shadow(0 0 8px ${COLORS.green}80)` }}
          />
          <text
            x={THROTTLE_X + 35}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT + 32}
            fill={COLORS.green}
            fontSize={22}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            fontWeight="bold"
          >
            {throttlePct.toFixed(0)}%
          </text>
        </g>

        {/* Brake bar */}
        <g>
          <text
            x={BRAKE_X + 35}
            y={BAR_Y_TOP - 20}
            fill={COLORS.red}
            fontSize={18}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            letterSpacing={2}
          >
            BRAKE
          </text>
          <rect x={BRAKE_X} y={BAR_Y_TOP} width={70} height={BAR_MAX_HEIGHT} rx={12} fill={`${COLORS.road}CC`} />
          <rect
            x={BRAKE_X}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT * (1 - brakePct / 100)}
            width={70}
            height={BAR_MAX_HEIGHT * (brakePct / 100)}
            rx={12}
            fill={COLORS.red}
            style={{ filter: `drop-shadow(0 0 8px ${COLORS.red}80)` }}
          />
          <text
            x={BRAKE_X + 35}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT + 32}
            fill={COLORS.red}
            fontSize={22}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            fontWeight="bold"
          >
            {brakePct.toFixed(0)}%
          </text>
        </g>
      </svg>

      {/* Synchronized vehicle sprite showing wheels actively turning with the gauge */}
      <div style={{ position: "absolute", bottom: 160, right: 360 }}>
        <VehicleSprite
          x={0}
          y={0}
          wheelAngle={needleAngle}
          speedKmh={interpolate(throttlePct, [0, 80], [0, 16], { extrapolateLeft: "clamp", extrapolateRight: "clamp" })}
          color={COLORS.cyan}
        />
        <div
          style={{
            position: "absolute",
            top: 75,
            left: "50%",
            transform: "translateX(-50%)",
            fontFamily: "'Courier New', monospace",
            fontSize: 14,
            color: COLORS.textMuted,
            letterSpacing: 2,
            whiteSpace: "nowrap",
            textAlign: "center",
          }}
        >
          WHEEL ANGLE: {needleAngle.toFixed(1)}°
        </div>
      </div>

      {/* Mandatory Limit Badge: Max steering rate: 25°/s */}
      <div
        style={{
          position: "absolute",
          top: 140,
          left: "50%",
          transform: "translateX(-50%)",
          padding: "12px 32px",
          borderRadius: 16,
          border: `2px solid ${COLORS.cyan}`,
          backgroundColor: `${COLORS.bg}EE`,
          boxShadow: `0 0 20px ${COLORS.cyan}40`,
          fontFamily: "'Courier New', monospace",
          fontSize: 22,
          fontWeight: 700,
          color: COLORS.cyan,
          letterSpacing: 3,
          textTransform: "uppercase",
          opacity: hudOpacity,
        }}
      >
        {`Max steering rate: ${MAX_STEERING_RATE_DEG_S}°/s`}
      </div>

      {/* HUD Label */}
      <HudLabel
        text="ACTUATION · CLOSED-LOOP LATERAL & LONGITUDINAL CONTROLLER"
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
        STEP 7 / 7
      </div>

      {/* Bottom note */}
      <div
        style={{
          position: "absolute",
          bottom: 70,
          left: "50%",
          transform: "translateX(-50%)",
          fontFamily: "'Courier New', monospace",
          fontSize: 20,
          color: COLORS.textMuted,
          letterSpacing: 3,
          opacity: interpolate(frame, [60, 90], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        Pure Pursuit + MPC · Zero high-frequency jerk · Passenger comfort compliant
      </div>
    </AbsoluteFill>
  );
};
