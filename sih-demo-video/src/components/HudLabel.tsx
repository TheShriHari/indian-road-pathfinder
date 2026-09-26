import React from "react";
import { COLORS } from "../theme";

export interface HudLabelProps {
  text: string;
  x: number;
  y: number;
  opacity: number;
  fontSize?: number;
  color?: string;
}

export const HudLabel: React.FC<HudLabelProps> = ({
  text,
  x,
  y,
  opacity,
  fontSize = 22,
  color = COLORS.textMuted,
}) => {
  return (
    <div
      style={{
        position: "absolute",
        left: x,
        top: y,
        opacity,
        fontFamily: "'Courier New', monospace",
        fontSize,
        color,
        letterSpacing: 1.5,
        textTransform: "uppercase",
        whiteSpace: "nowrap",
        pointerEvents: "none",
        textShadow: `0 0 8px ${color}80`,
      }}
    >
      {text}
    </div>
  );
};
