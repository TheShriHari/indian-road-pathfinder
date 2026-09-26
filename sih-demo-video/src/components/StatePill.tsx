import React from "react";
import { COLORS } from "../theme";

export interface StatePillProps {
  label: string;
  color: string;
  speedKmh: number;
  subtitle: string;
  opacity?: number;
  scale?: number;
}

export const StatePill: React.FC<StatePillProps> = ({
  label,
  color,
  speedKmh,
  subtitle,
  opacity = 1,
  scale = 1,
}) => {
  return (
    <div
      style={{
        opacity,
        transform: `scale(${scale})`,
        display: "inline-flex",
        flexDirection: "column",
        alignItems: "center",
        padding: "16px 36px",
        borderRadius: 18,
        border: `2px solid ${color}`,
        backgroundColor: `${COLORS.bg}EE`,
        boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 15px ${color}20`,
        filter: `drop-shadow(0 0 8px ${color}66)`,
        minWidth: 230,
        gap: 4,
      }}
    >
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 700,
          fontSize: 28,
          letterSpacing: 3,
          color,
        }}
      >
        {label}
      </span>
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 600,
          fontSize: 40,
          color,
          lineHeight: 1,
        }}
      >
        {speedKmh.toFixed(1)} km/h
      </span>
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 400,
          fontSize: 18,
          color: COLORS.textMuted,
          marginTop: 4,
        }}
      >
        {subtitle}
      </span>
    </div>
  );
};
