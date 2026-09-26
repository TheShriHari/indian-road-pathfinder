import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";

// A simple pothole shape
const Pothole: React.FC<{ cx: number; cy: number; r?: number }> = ({ cx, cy, r = 18 }) => (
  <ellipse
    cx={cx}
    cy={cy}
    rx={r}
    ry={r * 0.6}
    fill="#0B0F19"
    stroke="#334155"
    strokeWidth={2}
  />
);

// A silhouette of an obstacle (cow / rickshaw) moving laterally
const ObstacleSilhouette: React.FC<{
  x: number;
  y: number;
  type: "cow" | "rickshaw";
  lateralOffset: number;
}> = ({ x, y, type, lateralOffset }) => {
  if (type === "cow") {
    return (
      <g transform={`translate(${x + lateralOffset}, ${y})`}>
        {/* Cow body */}
        <ellipse cx={0} cy={0} rx={36} ry={22} fill="#334155" />
        {/* Cow head */}
        <ellipse cx={36} cy={-10} rx={16} ry={12} fill="#334155" />
        {/* Legs */}
        {[-24, -8, 8, 22].map((lx, i) => (
          <rect key={i} x={lx - 3} y={20} width={6} height={20} rx={2} fill="#334155" />
        ))}
        {/* Horns */}
        <line x1={42} y1={-20} x2={48} y2={-32} stroke="#94A3B8" strokeWidth={2} />
        <line x1={52} y1={-20} x2={58} y2={-32} stroke="#94A3B8" strokeWidth={2} />
      </g>
    );
  }
  // Rickshaw
  return (
    <g transform={`translate(${x + lateralOffset}, ${y})`}>
      {/* Main body */}
      <rect x={-30} y={-40} width={60} height={50} rx={6} fill="#334155" />
      {/* Canopy */}
      <rect x={-32} y={-55} width={64} height={18} rx={4} fill="#1E293B" stroke="#475569" strokeWidth={1.5} />
      {/* Wheels */}
      {[-28, 28].map((wx, i) => (
        <circle key={i} cx={wx} cy={14} r={14} fill="#1E293B" stroke="#475569" strokeWidth={2} />
      ))}
      {/* Windscreen */}
      <rect x={-20} y={-38} width={40} height={18} rx={2} fill="#0B0F19" opacity={0.7} />
    </g>
  );
};

export const RealityIntro: React.FC = () => {
  const frame = useCurrentFrame();

  const vignetteOpacity = interpolate(frame, [0, 30], [0.6, 0.2], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const panY = interpolate(frame, [0, 150], [0, -120], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const sceneOpacity = interpolate(frame, [0, 15], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Lateral animation for cow and rickshaw (accelerated for 5s scene)
  const cowOffset = interpolate(frame, [0, 150], [0, -180], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const rickshawOffset = interpolate(frame, [25, 150], [0, 220], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Road edge erosion intensity
  const erosionAlpha = interpolate(frame, [0, 50], [0.25, 0.65], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill
      style={{
        opacity: sceneOpacity,
        transform: `translateY(${panY}px)`,
        backgroundColor: COLORS.bg,
        overflow: "hidden",
      }}
    >
      <svg
        width={1920}
        height={1280}
        viewBox="0 0 1920 1280"
        style={{ position: "absolute", top: 0, left: 0 }}
      >
        {/* Sky / background */}
        <rect width={1920} height={1280} fill={COLORS.bg} />

        {/* Road base */}
        <rect x={500} y={0} width={920} height={1280} fill="#161E2E" />

        {/* Road edges — eroded, uneven */}
        {/* Left edge strokes */}
        {[0, 80, 180, 310, 420, 550, 680, 800, 950, 1080, 1200].map((y, i) => (
          <rect
            key={`le-${i}`}
            x={490 + ((i % 3) * 8 - 8)}
            y={y}
            width={28 + (i % 4) * 6}
            height={60 + (i % 3) * 20}
            fill="#334155"
            opacity={erosionAlpha}
            rx={3}
          />
        ))}
        {/* Right edge strokes */}
        {[40, 160, 290, 390, 520, 660, 790, 920, 1060, 1180, 1280].map((y, i) => (
          <rect
            key={`re-${i}`}
            x={1398 - ((i % 3) * 8)}
            y={y}
            width={24 + (i % 4) * 5}
            height={55 + (i % 3) * 18}
            fill="#334155"
            opacity={erosionAlpha}
            rx={3}
          />
        ))}

        {/* Center faded line remnants (barely visible, worn) */}
        {[100, 280, 460, 640, 820, 1000, 1180].map((y, i) => (
          <rect
            key={`cl-${i}`}
            x={952}
            y={y}
            width={16}
            height={80 + (i % 3) * 12}
            fill="#334155"
            opacity={0.15}
            rx={2}
          />
        ))}

        {/* Potholes — static */}
        <Pothole cx={720} cy={350} r={22} />
        <Pothole cx={1180} cy={580} r={16} />
        <Pothole cx={850} cy={820} r={26} />
        <Pothole cx={1050} cy={240} r={14} />

        {/* Obstacles */}
        <ObstacleSilhouette x={800} y={460} type="cow" lateralOffset={cowOffset} />
        <ObstacleSilhouette x={1100} y={650} type="rickshaw" lateralOffset={rickshawOffset} />

        {/* Ground / dirt shoulders */}
        <rect x={0} y={0} width={500} height={1280} fill="#0D1520" />
        <rect x={1420} y={0} width={500} height={1280} fill="#0D1520" />

        {/* Vignette overlay */}
        <defs>
          <radialGradient id="vignette" cx="50%" cy="50%" r="70%">
            <stop offset="30%" stopColor="transparent" />
            <stop offset="100%" stopColor="#000" />
          </radialGradient>
        </defs>
        <rect
          width={1920}
          height={1280}
          fill="url(#vignette)"
          opacity={vignetteOpacity}
        />
      </svg>

      {/* Text label */}
      <div
        style={{
          position: "absolute",
          bottom: 200,
          left: "50%",
          transform: "translateX(-50%)",
          fontFamily: "'Courier New', monospace",
          fontSize: 24,
          color: COLORS.textMuted,
          letterSpacing: 4,
          textTransform: "uppercase",
          opacity: interpolate(frame, [15, 45], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        Indian roads — no maps, no lanes, no guarantees
      </div>
    </AbsoluteFill>
  );
};
