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
}

export const VehicleSprite: React.FC<VehicleSpriteProps> = ({
  x,
  y,
  rotation = 0,
  color = COLORS.cyan,
  width = 60,
  height = 110,
  opacity = 1,
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
      }}
    >
      <svg
        width={width}
        height={height}
        viewBox={`0 0 ${width} ${height}`}
        style={{ display: "block" }}
      >
        {/* Car body */}
        <rect
          x={4}
          y={8}
          width={width - 8}
          height={height - 16}
          rx={8}
          fill={`${color}22`}
          stroke={color}
          strokeWidth={2.5}
        />
        {/* Windshield */}
        <rect
          x={10}
          y={14}
          width={width - 20}
          height={22}
          rx={4}
          fill={`${color}55`}
        />
        {/* Headlights */}
        <ellipse cx={16} cy={12} rx={6} ry={4} fill={color} opacity={0.9} />
        <ellipse cx={width - 16} cy={12} rx={6} ry={4} fill={color} opacity={0.9} />
        {/* Rear lights */}
        <ellipse cx={16} cy={height - 12} rx={6} ry={4} fill="#EF4444" opacity={0.8} />
        <ellipse cx={width - 16} cy={height - 12} rx={6} ry={4} fill="#EF4444" opacity={0.8} />
        {/* Center stripe */}
        <line
          x1={width / 2}
          y1={40}
          x2={width / 2}
          y2={height - 24}
          stroke={`${color}60`}
          strokeWidth={1.5}
          strokeDasharray="4 4"
        />
      </svg>
    </div>
  );
};
