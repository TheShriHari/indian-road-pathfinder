import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, SPRING_PRESETS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";

const VEHICLE_X = 960;

export const ClosingBranding: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 25], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Vehicle accelerates forward and halts cleanly at y: 640
  const vehicleY = interpolate(frame, [0, 160], [860, 640], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const vehicleSpeed = interpolate(frame, [0, 100, 160], [14, 18, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const vehicleOpacity = interpolate(frame, [430, 470], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Staggered reveals for branding
  const logoOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const teamTitleSpring = spring({
    frame: Math.max(0, frame - 40),
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });
  const teamTitleScale = interpolate(teamTitleSpring, [0, 1], [0.85, 1.0]);

  const deptOpacity = interpolate(frame, [70, 105], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // "PS-26037 solved, real gaps remain" line plays for 3s (frames 140 to 230)
  const gapsLineOpacity = interpolate(
    frame,
    [140, 160, 230, 250],
    [0, 1, 1, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Closing tagline reveals after gaps line (frames 250 to 460)
  const taglineOpacity = interpolate(frame, [255, 290], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const finalFade = interpolate(frame, [440, 480], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const taglineGlow = (Math.sin(frame * 0.08) + 1) / 2;

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        opacity: sceneOpacity * finalFade,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        overflow: "hidden",
      }}
    >
      {/* Grid */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line
            key={`h-${i}`}
            x1={0}
            y1={i * 52}
            x2={1920}
            y2={i * 52}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.2}
          />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line
            key={`v-${i}`}
            x1={i * 52}
            y1={0}
            x2={i * 52}
            y2={1080}
            stroke={COLORS.grid}
            strokeWidth={1}
            opacity={0.2}
          />
        ))}

        {/* Road strip */}
        <rect x={850} y={0} width={220} height={1080} fill={COLORS.road} />
        <rect x={850} y={0} width={6} height={1080} fill={COLORS.curb} opacity={0.5} />
        <rect x={1064} y={0} width={6} height={1080} fill={COLORS.curb} opacity={0.5} />

        {/* Vehicle trail */}
        <line
          x1={VEHICLE_X}
          y1={vehicleY + 60}
          x2={VEHICLE_X}
          y2={1080}
          stroke={COLORS.green}
          strokeWidth={4}
          strokeDasharray="8 8"
          opacity={0.6}
        />
      </svg>

      {/* Vehicle with speed-driven chassis squash & stretch */}
      <VehicleSprite
        x={VEHICLE_X}
        y={vehicleY}
        color={COLORS.green}
        opacity={vehicleOpacity}
        speedKmh={vehicleSpeed}
      />

      {/* Header */}
      <div
        style={{
          position: "absolute",
          top: 70,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: logoOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.textMuted,
          letterSpacing: 5,
          textTransform: "uppercase",
          textAlign: "center",
          whiteSpace: "nowrap",
        }}
      >
        Smart India Hackathon 2026 · Ministry of Education · MathWorks
      </div>

      {/* Team title - upper third */}
      <div
        style={{
          position: "absolute",
          top: 130,
          left: "50%",
          transform: `translateX(-50%) scale(${teamTitleScale})`,
          textAlign: "center",
        }}
      >
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontWeight: 700,
            fontSize: 88,
            color: COLORS.text,
            letterSpacing: 6,
            textTransform: "uppercase",
            textShadow: `0 0 30px ${COLORS.cyan}40`,
          }}
        >
          OORUM BLOOD
        </div>
      </div>

      {/* Department tag */}
      <div
        style={{
          position: "absolute",
          top: 250,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: deptOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 24,
          color: COLORS.textMuted,
          letterSpacing: 4,
          textAlign: "center",
          whiteSpace: "nowrap",
        }}
      >
        Indian Institute of Technology Kharagpur
      </div>

      {/* PS tag */}
      <div
        style={{
          position: "absolute",
          top: 290,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: deptOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 5,
          textAlign: "center",
        }}
      >
        Problem Statement 26037 (MathWorks)
      </div>

      {/* Line: "PS-26037 solved, real gaps remain" (3s duration) */}
      <div
        style={{
          position: "absolute",
          top: 360,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: gapsLineOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 26,
          color: COLORS.amber,
          letterSpacing: 4,
          textAlign: "center",
          whiteSpace: "nowrap",
          padding: "10px 28px",
          borderRadius: 14,
          border: `1.5px solid ${COLORS.amber}60`,
          backgroundColor: `${COLORS.amber}15`,
          boxShadow: `0 0 20px ${COLORS.amber}30`,
        }}
      >
        PS-26037 SOLVED · REAL GAPS REMAIN
      </div>

      {/* Final Closing tagline */}
      <div
        style={{
          position: "absolute",
          top: 360,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: taglineOpacity,
          fontFamily: "'Courier New', monospace",
          fontSize: 28,
          color: COLORS.cyan,
          letterSpacing: 3,
          textAlign: "center",
          whiteSpace: "nowrap",
          textShadow: `0 0 ${20 + taglineGlow * 10}px ${COLORS.cyan}80`,
        }}
      >
        Autonomous Mobility Built for the Roads India Actually Has.
      </div>
    </AbsoluteFill>
  );
};
