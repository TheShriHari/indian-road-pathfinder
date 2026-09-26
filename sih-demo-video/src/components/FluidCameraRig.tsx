import React from "react";
import { useCurrentFrame, interpolate } from "remotion";

export const FluidCameraRig: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const frame = useCurrentFrame();

  // Smooth continuous camera dolly/zoom keyed across the 12 beats (5160 frames total)
  const zoom = interpolate(
    frame,
    [
      0, 180,       // Beat 1: Title push-through
      480, 720,     // Beat 2: Road wide -> dolly into roof sensor
      900, 1140,    // Beat 3: Perception scan sweep
      1350, 1560,   // Beat 4: Pull up to tactical bird's-eye
      1750, 1950,   // Beat 5: Costmap build
      2150, 2400,   // Beat 6: Caliper pinch zoom -> particle shatter
      2600, 2820,   // Beat 7: Path planning spline reveal
      3100, 3420,   // Beat 8: FSM vehicle track
      3600, 3780,   // Beat 9: Steering control cockpit
      4000, 4260,   // Beat 10: BugsFoundFixed diagnostic push
      4450, 4680,   // Beat 11: Rigor KPIs wide pull-back
      4900, 5160,   // Beat 12: Vehicle final acceleration toward camera
    ],
    [
      1.0, 1.06,
      1.01, 1.08,
      1.07, 1.04,
      1.02, 0.98,
      0.99, 1.04,
      1.06, 1.03,
      1.02, 1.06,
      1.06, 1.02,
      1.01, 1.05,
      1.06, 1.01,
      0.98, 1.0,
      1.01, 1.07,
    ],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  const panY = interpolate(
    frame,
    [
      0, 180,
      480, 720,
      1140, 1560,
      1950, 2400,
      2820, 3420,
      3780, 4260,
      4680, 5160,
    ],
    [
      0, -18,
      -5, -22,
      -10, 0,
      0, -15,
      -8, -2,
      0, -12,
      -5, 8,
    ],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  return (
    <div
      style={{
        width: "100%",
        height: "100%",
        transform: `scale(${zoom}) translateY(${panY}px)`,
        transformOrigin: "center center",
        willChange: "transform",
      }}
    >
      {children}
    </div>
  );
};
