import React from "react";
import { COLORS } from "../theme";

export interface StatePillProps {
  label: string;
  color: string;
  speedKmh: number;
  subtitle: string;
  opacity?: number;
}

export const StatePill: React.FC<StatePillProps> = ({
  label,
  color,
  speedKmh,
  subtitle,
  opacity = 1,
}) => {
  return (
    <div
      style={{
        opacity,
        display: "inline-flex",
        flexDirection: "column",
        alignItems: "center",
        padding: "18px 40px",
        borderRadius: 999,
        border: `2px solid ${color}`,
        backgroundColor: `${color}26`, // ~15% opacity
        minWidth: 220,
        gap: 4,
      }}
    >
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 700,
          fontSize: 32,
          letterSpacing: 3,
          color,
        }}
      >
        {label}
      </span>
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 400,
          fontSize: 42,
          color,
          lineHeight: 1,
        }}
      >
        {speedKmh.toFixed(0)} km/h
      </span>
      <span
        style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 400,
          fontSize: 20,
          color: COLORS.textMuted,
          marginTop: 4,
        }}
      >
        {subtitle}
      </span>
    </div>
  );
};
