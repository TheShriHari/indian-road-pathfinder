import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, Easing } from "remotion";
import { COLORS } from "../../theme";
import { HudLabel } from "../../components/HudLabel";

// Steering gauge constants
const GAUGE_CX = 960;
const GAUGE_CY = 500;
const GAUGE_R = 200;
const MAX_STEER_DEG = 30; // max needle angle from center

// Pedal bar constants (positioned on left to clear central gauge)
const THROTTLE_X = 420;
const BRAKE_X = 580;
const BAR_Y_TOP = 380;
const BAR_MAX_HEIGHT = 320;

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

  // Steering angle smoothly oscillates (S-curve following the planned path)
  const steeringAngle = interpolate(
    frame,
    [0, 30, 60, 90, 120, 150],
    [0, -MAX_STEER_DEG * 0.8, -MAX_STEER_DEG * 0.4, MAX_STEER_DEG * 0.2, -MAX_STEER_DEG * 0.15, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Throttle: high → low (decelerating through obstacle zone)
  const throttlePct = interpolate(frame, [0, 60], [80, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Brake: low → high (eased)
  const brakePct = interpolate(frame, [0, 60], [0, 90], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
    easing: Easing.inOut(Easing.ease),
  });

  // Rate limit tick mark — needle never exceeds MAX_STEER_DEG
  const needleAngle = Math.max(-MAX_STEER_DEG, Math.min(MAX_STEER_DEG, steeringAngle));

  // Needle tip
  const needleTip = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 20, 180 + needleAngle);
  // Needle base
  const needleBase = polarToXY(GAUGE_CX, GAUGE_CY, 30, 180 + needleAngle);

  // Tick marks for the semicircle
  const TICK_COUNT = 13;

  // Gauge arc colors
  const leftColor = COLORS.cyan;
  const rightColor = COLORS.green;

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg }}>
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Grid */}
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.25} />
        ))}

        {/* ---- STEERING GAUGE ---- */}
        {/* Gauge backdrop */}
        <circle cx={GAUGE_CX} cy={GAUGE_CY} r={GAUGE_R + 20}
          fill={`${COLORS.grid}40`} stroke={COLORS.grid} strokeWidth={1} />

        {/* Arc track — semicircle (180° wide, pointing down) */}
        <path
          d={`M ${GAUGE_CX - GAUGE_R} ${GAUGE_CY} A ${GAUGE_R} ${GAUGE_R} 0 0 1 ${GAUGE_CX + GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={COLORS.grid}
          strokeWidth={16}
          strokeLinecap="round"
        />
        {/* Left side (steer left = cyan) */}
        <path
          d={`M ${GAUGE_CX} ${GAUGE_CY - GAUGE_R} A ${GAUGE_R} ${GAUGE_R} 0 0 0 ${GAUGE_CX - GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={leftColor}
          strokeWidth={8}
          strokeLinecap="round"
          opacity={0.5}
        />
        {/* Right side (steer right = green) */}
        <path
          d={`M ${GAUGE_CX} ${GAUGE_CY - GAUGE_R} A ${GAUGE_R} ${GAUGE_R} 0 0 1 ${GAUGE_CX + GAUGE_R} ${GAUGE_CY}`}
          fill="none"
          stroke={rightColor}
          strokeWidth={8}
          strokeLinecap="round"
          opacity={0.5}
        />

        {/* Tick marks */}
        {Array.from({ length: TICK_COUNT }).map((_, i) => {
          const angle = -90 + (i / (TICK_COUNT - 1)) * 180; // -90 to +90 from top
          const inner = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 26, angle);
          const outer = polarToXY(GAUGE_CX, GAUGE_CY, GAUGE_R - 4, angle);
          const isMax = i === 0 || i === TICK_COUNT - 1;
          return (
            <line
              key={i}
              x1={inner.x} y1={inner.y} x2={outer.x} y2={outer.y}
              stroke={isMax ? COLORS.red : COLORS.textMuted}
              strokeWidth={isMax ? 3 : 1.5}
              opacity={0.8}
            />
          );
        })}

        {/* Rate-limit bracket labels */}
        <text x={GAUGE_CX - GAUGE_R - 12} y={GAUGE_CY + 4}
          fill={COLORS.red} fontSize={14} fontFamily="'Courier New', monospace"
          textAnchor="end">MAX</text>
        <text x={GAUGE_CX + GAUGE_R + 12} y={GAUGE_CY + 4}
          fill={COLORS.red} fontSize={14} fontFamily="'Courier New', monospace">MAX</text>

        {/* Needle */}
        <line
          x1={needleBase.x} y1={needleBase.y}
          x2={needleTip.x} y2={needleTip.y}
          stroke={COLORS.cyan}
          strokeWidth={5}
          strokeLinecap="round"
          style={{ filter: `drop-shadow(0 0 4px ${COLORS.cyan})` }}
        />
        <circle cx={GAUGE_CX} cy={GAUGE_CY} r={14}
          fill={COLORS.bg} stroke={COLORS.cyan} strokeWidth={2.5} />

        {/* Angle readout */}
        <text x={GAUGE_CX} y={GAUGE_CY + 60}
          fill={COLORS.cyan} fontSize={32}
          fontFamily="'Courier New', monospace"
          textAnchor="middle" fontWeight="bold">
          {needleAngle.toFixed(1)}°
        </text>
        <text x={GAUGE_CX} y={GAUGE_CY + 90}
          fill={COLORS.textMuted} fontSize={16}
          fontFamily="'Courier New', monospace"
          textAnchor="middle" letterSpacing={3}>
          STEER ANGLE
        </text>

        {/* ---- PEDAL BARS ---- */}
        {/* Throttle bar */}
        <g>
          <text x={THROTTLE_X + 35} y={BAR_Y_TOP - 20}
            fill={COLORS.green} fontSize={18}
            fontFamily="'Courier New', monospace"
            textAnchor="middle" letterSpacing={2}>THROTTLE</text>
          <rect x={THROTTLE_X} y={BAR_Y_TOP} width={70} height={BAR_MAX_HEIGHT}
            rx={6} fill={`${COLORS.grid}80`} />
          <rect
            x={THROTTLE_X}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT * (1 - throttlePct / 100)}
            width={70}
            height={BAR_MAX_HEIGHT * (throttlePct / 100)}
            rx={6}
            fill={COLORS.green}
            style={{ filter: `drop-shadow(0 0 6px ${COLORS.green}80)` }}
          />
          <text x={THROTTLE_X + 35} y={BAR_Y_TOP + BAR_MAX_HEIGHT + 30}
            fill={COLORS.green} fontSize={22}
            fontFamily="'Courier New', monospace"
            textAnchor="middle">
            {throttlePct.toFixed(0)}%
          </text>
        </g>

        {/* Brake bar */}
        <g>
          <text x={BRAKE_X + 35} y={BAR_Y_TOP - 20}
            fill={COLORS.red} fontSize={18}
            fontFamily="'Courier New', monospace"
            textAnchor="middle" letterSpacing={2}>BRAKE</text>
          <rect x={BRAKE_X} y={BAR_Y_TOP} width={70} height={BAR_MAX_HEIGHT}
            rx={6} fill={`${COLORS.grid}80`} />
          <rect
            x={BRAKE_X}
            y={BAR_Y_TOP + BAR_MAX_HEIGHT * (1 - brakePct / 100)}
            width={70}
            height={BAR_MAX_HEIGHT * (brakePct / 100)}
            rx={6}
            fill={COLORS.red}
            style={{ filter: `drop-shadow(0 0 6px ${COLORS.red}80)` }}
          />
          <text x={BRAKE_X + 35} y={BAR_Y_TOP + BAR_MAX_HEIGHT + 30}
            fill={COLORS.red} fontSize={22}
            fontFamily="'Courier New', monospace"
            textAnchor="middle">
            {brakePct.toFixed(0)}%
          </text>
        </g>

        {/* "SMOOTH" label */}
        <text x={535} y={BAR_Y_TOP + BAR_MAX_HEIGHT / 2}
          fill={COLORS.textMuted} fontSize={14}
          fontFamily="'Courier New', monospace"
          textAnchor="middle" opacity={hudOpacity}>
          RATE-
        </text>
        <text x={535} y={BAR_Y_TOP + BAR_MAX_HEIGHT / 2 + 20}
          fill={COLORS.textMuted} fontSize={14}
          fontFamily="'Courier New', monospace"
          textAnchor="middle" opacity={hudOpacity}>
          LIMITED
        </text>
      </svg>

      {/* HUD */}
      <HudLabel
        text="CONTROL · smooth, rate-limited steering & speed"
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
        STEP 7 / 7
      </div>

      {/* Bottom note */}
      <div style={{
        position: "absolute",
        bottom: 80,
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
      }}>
        No sudden jerks · Smooth, comfortable control
      </div>
    </AbsoluteFill>
  );
};
