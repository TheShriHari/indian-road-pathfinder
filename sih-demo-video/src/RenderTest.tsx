import React from "react";
import { AbsoluteFill, spring, useCurrentFrame, useVideoConfig } from "remotion";

export const RenderTest: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const scale = spring({
    frame,
    fps,
    config: {
      damping: 12,
      mass: 0.5,
      stiffness: 100,
    },
  });

  return (
    <AbsoluteFill
      style={{
        backgroundColor: "#0B0F19",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        fontFamily: "system-ui, -apple-system, sans-serif",
      }}
    >
      <h1
        style={{
          color: "#38BDF8",
          fontSize: "64px",
          fontWeight: 800,
          letterSpacing: "4px",
          textShadow: "0 0 30px rgba(56, 189, 248, 0.8), 0 0 60px rgba(56, 189, 248, 0.4)",
          transform: `scale(${scale})`,
        }}
      >
        REMOTION ENGINE READY
      </h1>
    </AbsoluteFill>
  );
};
