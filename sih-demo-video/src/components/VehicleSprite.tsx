import React from "react";
import { COLORS } from "../theme";

export interface VehicleSpriteProps {
  x: number;
  y: number;
  rotation?: number;
  color?: string;
  width?: number;
  height?: number;
  opacity?: number;
  wheelAngle?: number;   // physical steering angle in degrees (-25 to +25 deg)
}

export const VehicleSprite: React.FC<VehicleSpriteProps> = ({
  x,
  y,
  rotation = 0,
  color = COLORS.cyan,
  width = 60,
  height = 112,
  opacity = 1,
  wheelAngle = 0,
}) => {
  return (
    <div
      style={{
        position: "absolute",
        left: x - width / 2,
        top: y - height / 2,
        width,
        height,
        opacity,
        transform: `rotate(${rotation}deg)`,
        transformOrigin: "center center",
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
        <g transform={`translate(2, 20) rotate(${wheelAngle}, 4, 10)`}>
          <rect
            x={0}
            y={0}
            width={7}
            height={20}
            rx={2}
            fill="#0F172A"
            stroke={color}
            strokeWidth={1.5}
          />
        </g>

        {/* Right Front Wheel with dynamic steering angle */}
        <g transform={`translate(${width - 9}, 20) rotate(${wheelAngle}, 4, 10)`}>
          <rect
            x={0}
            y={0}
            width={7}
            height={20}
            rx={2}
            fill="#0F172A"
            stroke={color}
            strokeWidth={1.5}
          />
        </g>

        {/* Rear Wheels (fixed) */}
        <rect x={2} y={height - 36} width={7} height={20} rx={2} fill="#0F172A" stroke={color} strokeWidth={1.5} />
        <rect x={width - 9} y={height - 36} width={7} height={20} rx={2} fill="#0F172A" stroke={color} strokeWidth={1.5} />

        {/* Utilitarian chassis body with clean engineering lines */}
        <rect
          x={6}
          y={6}
          width={width - 12}
          height={height - 12}
          rx={8}
          fill="#111C30"
          stroke={color}
          strokeWidth={2}
        />

        {/* Windshield / Canopy */}
        <path
          d={`M ${12} ${34} L ${width - 12} ${34} L ${width - 16} ${56} L ${16} ${56} Z`}
          fill="#0F172A"
          stroke={color}
          strokeWidth={1}
          opacity={0.8}
        />

        {/* Rear Window */}
        <rect
          x={14}
          y={height - 40}
          width={width - 28}
          height={14}
          rx={3}
          fill="#0F172A"
          stroke={color}
          strokeWidth={1}
          opacity={0.8}
        />

        {/* Roof Sensor Pod (Lidar / Camera Array) */}
        <circle
          cx={width / 2}
          cy={height / 2 - 2}
          r={7}
          fill="#0B111E"
          stroke={color}
          strokeWidth={1.5}
        />
        <circle
          cx={width / 2}
          cy={height / 2 - 2}
          r={2.5}
          fill={color}
        />

        {/* Front Headlights */}
        <rect x={10} y={7} width={8} height={4} rx={1.5} fill="#FFFFFF" opacity={0.9} />
        <rect x={width - 18} y={7} width={8} height={4} rx={1.5} fill="#FFFFFF" opacity={0.9} />

        {/* Tail Brake Lights */}
        <rect x={10} y={height - 10} width={8} height={3} rx={1} fill={COLORS.red} opacity={0.9} />
        <rect x={width - 18} y={height - 10} width={8} height={3} rx={1} fill={COLORS.red} opacity={0.9} />

        {/* Centerline orientation reference */}
        <line
          x1={width / 2}
          y1={12}
          x2={width / 2}
          y2={28}
          stroke={color}
          strokeWidth={1}
          strokeDasharray="2 2"
          opacity={0.6}
        />
      </svg>
    </div>
  );
};
