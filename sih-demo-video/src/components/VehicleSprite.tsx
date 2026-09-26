import React from "react";
import { interpolate } from "remotion";
import { COLORS } from "../theme";

export interface VehicleSpriteProps {
  x: number;
  y: number;
  rotation?: number;
  color?: string;
  width?: number;
  height?: number;
  opacity?: number;
  speedKmh?: number;     // actual computed speed from state machine (0 to 18 km/h)
  wheelAngle?: number;   // physical steering angle in degrees (-25 to +25 deg)
}

export const VehicleSprite: React.FC<VehicleSpriteProps> = ({
  x,
  y,
  rotation = 0,
  color = COLORS.cyan,
  width = 64,
  height = 116,
  opacity = 1,
  speedKmh = 18,
  wheelAngle = 0,
}) => {
  // Physical chassis squash & stretch driven by actual speed (Section 1.5)
  // Higher speed = streamlined/extended; decelerating/stopping = nose pitch down & lateral spread
  const chassisScaleY = interpolate(speedKmh, [0, 18], [0.92, 1.0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const chassisScaleX = interpolate(speedKmh, [0, 18], [1.05, 1.0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const pitchDeg = interpolate(speedKmh, [0, 18], [-3.5, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <div
      style={{
        position: "absolute",
        left: x - width / 2,
        top: y - height / 2,
        width,
        height,
        opacity,
        transform: `rotate(${rotation + pitchDeg}deg) scale(${chassisScaleX}, ${chassisScaleY})`,
        transformOrigin: "center center",
        filter: `drop-shadow(0 0 10px ${color}88)`,
        pointerEvents: "none",
      }}
    >
      <svg
        width={width}
        height={height}
        viewBox={`0 0 ${width} ${height}`}
        style={{ display: "block" }}
      >
        {/* Left Front Wheel with dynamic steering angle */}
        <g transform={`translate(2, 22) rotate(${wheelAngle}, 4, 10)`}>
          <rect
            x={0}
            y={0}
            width={7}
            height={20}
            rx={3}
            fill="#1E293B"
            stroke={color}
            strokeWidth={1}
          />
        </g>

        {/* Right Front Wheel with dynamic steering angle */}
        <g transform={`translate(${width - 9}, 22) rotate(${wheelAngle}, 4, 10)`}>
          <rect
            x={0}
            y={0}
            width={7}
            height={20}
            rx={3}
            fill="#1E293B"
            stroke={color}
            strokeWidth={1}
          />
        </g>

        {/* Rear Wheels (fixed) */}
        <rect x={2} y={height - 38} width={7} height={20} rx={3} fill="#1E293B" stroke={color} strokeWidth={1} />
        <rect x={width - 9} y={height - 38} width={7} height={20} rx={3} fill="#1E293B" stroke={color} strokeWidth={1} />

        {/* Aerodynamic chassis body with 14px squircle corners */}
        <rect
          x={6}
          y={8}
          width={width - 12}
          height={height - 16}
          rx={14}
          ry={14}
          fill={`${color}1A`}
          stroke={color}
          strokeWidth={2.8}
        />

        {/* Canopy / Roof Sensor Pod with neon electric glow */}
        <rect
          x={13}
          y={18}
          width={width - 26}
          height={26}
          rx={8}
          ry={8}
          fill={`${color}40`}
          stroke={color}
          strokeWidth={1.5}
        />

        {/* Sensor Pod Turret */}
        <circle
          cx={width / 2}
          cy={28}
          r={5.5}
          fill={COLORS.bg}
          stroke={color}
          strokeWidth={2}
        />
        <circle
          cx={width / 2}
          cy={28}
          r={2.5}
          fill={COLORS.cyan}
        />

        {/* Forward Headlights */}
        <ellipse cx={17} cy={12} rx={6} ry={4} fill={color} opacity={0.95} />
        <ellipse cx={width - 17} cy={12} rx={6} ry={4} fill={color} opacity={0.95} />

        {/* Rear Braking LEDs (illuminate brighter when decelerating / stopped) */}
        <ellipse
          cx={17}
          cy={height - 12}
          rx={6}
          ry={4}
          fill={speedKmh === 0 ? COLORS.red : "#EF4444"}
          opacity={speedKmh < 8 ? 1.0 : 0.6}
        />
        <ellipse
          cx={width - 17}
          cy={height - 12}
          rx={6}
          ry={4}
          fill={speedKmh === 0 ? COLORS.red : "#EF4444"}
          opacity={speedKmh < 8 ? 1.0 : 0.6}
        />

        {/* Center Line Guide */}
        <line
          x1={width / 2}
          y1={46}
          x2={width / 2}
          y2={height - 24}
          stroke={`${color}80`}
          strokeWidth={1.5}
          strokeDasharray="4 4"
        />
      </svg>
    </div>
  );
};
