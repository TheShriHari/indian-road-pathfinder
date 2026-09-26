import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, SPRING_PRESETS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";

const Pothole: React.FC<{ cx: number; cy: number; r?: number }> = ({ cx, cy, r = 20 }) => (
  <ellipse
    cx={cx}
    cy={cy}
    rx={r}
    ry={r * 0.55}
    fill="#080D17"
    stroke={COLORS.curb}
    strokeWidth={2}
  />
);

interface AssumptionCardProps {
  text: string;
  y: number;
  enterFrame: number;
  strikeStart: number;
  strikeEnd: number;
  frame: number;
}

const AssumptionCard: React.FC<AssumptionCardProps> = ({
  text,
  y,
  enterFrame,
  strikeStart,
  strikeEnd,
  frame,
}) => {
  const cardSpring = spring({
    frame: Math.max(0, frame - enterFrame),
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });
  const cardScale = interpolate(cardSpring, [0, 1], [0.8, 1.0]);
  const cardOpacity = interpolate(cardSpring, [0, 0.2], [0, 1]);

  const strikeWidth = interpolate(frame, [strikeStart, strikeEnd], [0, 100], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const isStruck = frame >= strikeEnd;
  const isStriking = frame >= strikeStart;

  return (
    <div
      style={{
        position: "absolute",
        left: "50%",
        top: y,
        transform: `translateX(-50%) scale(${cardScale})`,
        opacity: cardOpacity,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        width: 1140,
      }}
    >
      <div
        style={{
          padding: "20px 44px",
          borderRadius: 18,
          border: `2px solid ${isStriking ? COLORS.red : COLORS.curb}`,
          backgroundColor: `${COLORS.bg}EE`,
          boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 15px ${isStriking ? COLORS.red + "25" : "transparent"}`,
          position: "relative",
          width: "100%",
          boxSizing: "border-box",
        }}
      >
        <span
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 28,
            color: isStruck ? COLORS.red : COLORS.text,
            letterSpacing: 2,
            display: "block",
            whiteSpace: "nowrap",
            opacity: isStruck ? 0.7 : 1.0,
            textShadow: isStriking ? `0 0 12px ${COLORS.red}60` : "none",
          }}
        >
          {text}
        </span>

        {/* Laser Strikethrough bar */}
        <div
          style={{
            position: "absolute",
            top: "50%",
            left: 44,
            height: 4,
            width: `${(strikeWidth * (1140 - 88)) / 1140}%`,
            backgroundColor: COLORS.red,
            transform: "translateY(-50%)",
            transformOrigin: "left",
            boxShadow: `0 0 12px ${COLORS.red}`,
            borderRadius: 2,
          }}
        />
      </div>
    </div>
  );
};

export const ProblemStatement: React.FC = () => {
  const frame = useCurrentFrame();

  // Road vertical pan animation
  const roadPanY = interpolate(frame, [0, 540], [0, -160], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Camera push into vehicle sensor pod at the end of the beat (frames 360 to 540)
  const dollyScale = interpolate(frame, [360, 540], [1.0, 1.8], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const dollyY = interpolate(frame, [360, 540], [0, 240], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Fade out cards before camera push
  const cardsFade = interpolate(frame, [350, 390], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Obstacles lateral movement
  const cowX = interpolate(frame, [0, 540], [740, 620], { extrapolateLeft: "clamp", extrapolateRight: "clamp" });
  const rickshawX = interpolate(frame, [0, 540], [1160, 1280], { extrapolateLeft: "clamp", extrapolateRight: "clamp" });

  // Floating dust particles (secondary motion from reference)
  const dustOffsets = [
    { x: 920, y: 760, driftX: (frame * 0.4) % 60 - 30, driftY: (frame * 0.2) % 40 - 20 },
    { x: 990, y: 780, driftX: -(frame * 0.3) % 50 + 25, driftY: (frame * 0.15) % 30 - 15 },
    { x: 940, y: 810, driftX: (frame * 0.5) % 70 - 35, driftY: -(frame * 0.25) % 35 + 17 },
  ];

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        overflow: "hidden",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      {/* Zoom / Dolly container */}
      <div
        style={{
          width: "100%",
          height: "100%",
          transform: `scale(${dollyScale}) translateY(${dollyY}px)`,
          transformOrigin: "960px 750px", // focus directly on ego vehicle sensor pod
          position: "absolute",
          top: 0,
          left: 0,
        }}
      >
        <svg
          width={1920}
          height={1280}
          viewBox="0 0 1920 1280"
          style={{ position: "absolute", top: roadPanY, left: 0 }}
        >
          {/* Background ground */}
          <rect width={1920} height={1280} fill={COLORS.bg} />

          {/* Road asphalt */}
          <rect x={540} y={0} width={840} height={1280} fill={COLORS.road} />

          {/* Eroded curb edges */}
          {[0, 120, 240, 360, 480, 600, 720, 840, 960, 1080, 1200].map((y, i) => (
            <rect
              key={`left-curb-${i}`}
              x={530 + ((i % 3) * 8 - 4)}
              y={y}
              width={26}
              height={80}
              fill={COLORS.curb}
              rx={6}
              opacity={0.8}
            />
          ))}
          {[40, 160, 280, 400, 520, 640, 760, 880, 1000, 1120, 1240].map((y, i) => (
            <rect
              key={`right-curb-${i}`}
              x={1370 - ((i % 3) * 8 - 4)}
              y={y}
              width={26}
              height={80}
              fill={COLORS.curb}
              rx={6}
              opacity={0.8}
            />
          ))}

          {/* Center line fragments (faded & discontinuous) */}
          {[60, 260, 480, 700, 920, 1140].map((y, i) => (
            <rect
              key={`cl-${i}`}
              x={954}
              y={y}
              width={12}
              height={90}
              fill={COLORS.curb}
              opacity={0.15}
              rx={3}
            />
          ))}

          {/* Potholes */}
          <Pothole cx={750} cy={380} r={26} />
          <Pothole cx={1180} cy={540} r={22} />
          <Pothole cx={880} cy={920} r={30} />
          <Pothole cx={1060} cy={260} r={18} />

          {/* Cow Silhouette */}
          <g transform={`translate(${cowX}, 480)`}>
            <ellipse cx={0} cy={0} rx={38} ry={22} fill={COLORS.curb} />
            <ellipse cx={36} cy={-10} rx={18} ry={14} fill={COLORS.curb} />
            <line x1={42} y1={-20} x2={48} y2={-32} stroke={COLORS.amber} strokeWidth={2} />
            <line x1={52} y1={-20} x2={58} y2={-32} stroke={COLORS.amber} strokeWidth={2} />
            <text x={0} y={-30} fill={COLORS.amber} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="middle">
              STRAY CATTLE
            </text>
          </g>

          {/* Rickshaw Silhouette */}
          <g transform={`translate(${rickshawX}, 620)`}>
            <rect x={-32} y={-42} width={64} height={52} rx={8} fill={COLORS.curb} />
            <rect x={-34} y={-58} width={68} height={18} rx={6} fill="#1E293B" stroke={COLORS.orange} strokeWidth={1.5} />
            <circle cx={-26} cy={14} r={12} fill="#0F172A" stroke={COLORS.orange} strokeWidth={2} />
            <circle cx={26} cy={14} r={12} fill="#0F172A" stroke={COLORS.orange} strokeWidth={2} />
            <text x={0} y={-68} fill={COLORS.orange} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="middle">
              WRONG-WAY RICKSHAW
            </text>
          </g>

          {/* Secondary dust particles idling around vehicle */}
          {dustOffsets.map((d, i) => (
            <circle
              key={`dust-${i}`}
              cx={d.x + d.driftX}
              cy={d.y + d.driftY}
              r={3}
              fill={COLORS.curb}
              opacity={0.4}
            />
          ))}
        </svg>

        {/* Ego Vehicle idling on shoulder / lane */}
        <VehicleSprite x={960} y={750} speedKmh={0} color={COLORS.cyan} />
      </div>

      {/* Overlaid Assumption Cards */}
      <div
        style={{
          position: "absolute",
          width: "100%",
          height: "100%",
          top: 0,
          left: 0,
          opacity: cardsFade,
          pointerEvents: "none",
        }}
      >
        {/* Header */}
        <div
          style={{
            position: "absolute",
            top: 70,
            left: "50%",
            transform: "translateX(-50%)",
            fontFamily: "'Courier New', monospace",
            fontSize: 22,
            color: COLORS.cyan,
            letterSpacing: 6,
            textTransform: "uppercase",
            fontWeight: 700,
          }}
        >
          Conventional Autonomy Assumes:
        </div>

        {/* 3 Assumption Cards staggered 0.3s (9 frames) apart */}
        <AssumptionCard
          text="01 · Lanes and curbs exist and are visible"
          y={150}
          enterFrame={15}
          strikeStart={100}
          strikeEnd={160}
          frame={frame}
        />
        <AssumptionCard
          text="02 · Other traffic agents respect boundaries"
          y={260}
          enterFrame={30}
          strikeStart={180}
          strikeEnd={240}
          frame={frame}
        />
        <AssumptionCard
          text="03 · Centimetre-accurate HD vector maps exist"
          y={370}
          enterFrame={45}
          strikeStart={260}
          strikeEnd={320}
          frame={frame}
        />

        {/* Punchline Badge with Spring Pop */}
        {frame >= 320 && (
          <div
            style={{
              position: "absolute",
              top: 500,
              left: "50%",
              transform: `translateX(-50%) scale(${interpolate(
                spring({ frame: frame - 320, fps: FPS, config: SPRING_PRESETS.overshoot }),
                [0, 1],
                [0.75, 1.0]
              )})`,
              padding: "16px 48px",
              borderRadius: 18,
              border: `2px solid ${COLORS.red}`,
              backgroundColor: `${COLORS.bg}F5`,
              boxShadow: `0 0 30px ${COLORS.red}60`,
              fontFamily: "'Courier New', monospace",
              fontSize: 32,
              fontWeight: 700,
              color: COLORS.red,
              letterSpacing: 4,
              textTransform: "uppercase",
              textAlign: "center",
            }}
          >
            INDIAN ROADS HAVE NONE OF THAT.
          </div>
        )}
      </div>
    </AbsoluteFill>
  );
};
