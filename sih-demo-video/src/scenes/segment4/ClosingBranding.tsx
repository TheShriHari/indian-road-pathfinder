import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate } from "remotion";
import { COLORS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";

const VEHICLE_X = 960;

export const ClosingBranding: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Vehicle drives forward and stops at y: 650px (preventing text occlusion)
  const vehicleY = interpolate(frame, [0, 140], [840, 650], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const vehicleOpacity = interpolate(frame, [340, 380], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Trail opacity (fades with distance from vehicle)
  const trailOpacity = interpolate(frame, [0, 40], [0, 0.6], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const trailFadeOut = interpolate(frame, [340, 380], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Text elements
  const logoOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const teamTitleOpacity = interpolate(frame, [40, 75], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const deptOpacity = interpolate(frame, [60, 95], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const taglineOpacity = interpolate(frame, [100, 140], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Team title letter spacing: 2px → 6px
  const teamLetterSpacing = interpolate(frame, [40, 200], [2, 6], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Final fade out
  const finalFade = interpolate(frame, [350, 390], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Glow on tagline
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
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.2} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.2} />
        ))}

        {/* Road strip */}
        <rect x={860} y={0} width={200} height={1080} fill="#161E2E" />
        <rect x={860} y={0} width={6} height={1080} fill="#334155" opacity={0.4} />
        <rect x={1054} y={0} width={6} height={1080} fill="#334155" opacity={0.4} />

        {/* Vehicle trail — green fading gradient */}
        {frame > 10 && (
          <defs>
            <linearGradient id="trailGrad" x1={0} y1={vehicleY} x2={0} y2={vehicleY + 300} gradientUnits="userSpaceOnUse">
              <stop offset="0%" stopColor={COLORS.green} stopOpacity={0} />
              <stop offset="100%" stopColor={COLORS.green} stopOpacity={0.5} />
            </linearGradient>
          </defs>
        )}
        <rect
          x={VEHICLE_X - 10}
          y={vehicleY + 60}
          width={20}
          height={Math.max(0, 1080 - vehicleY - 60)}
          fill="url(#trailGrad)"
          opacity={trailOpacity * trailFadeOut}
          rx={4}
        />
      </svg>

      {/* Vehicle */}
      <VehicleSprite
        x={VEHICLE_X}
        y={vehicleY}
        color={COLORS.green}
        opacity={vehicleOpacity}
      />

      {/* Logo header */}
      <div style={{
        position: "absolute",
        top: 80,
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
      }}>
        Smart India Hackathon 2026 · Ministry of Education · MathWorks
      </div>

      {/* Team title - upper third */}
      <div style={{
        position: "absolute",
        top: 140,
        left: "50%",
        transform: "translateX(-50%)",
        opacity: teamTitleOpacity,
        textAlign: "center",
      }}>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontWeight: 700,
          fontSize: 88,
          color: COLORS.text,
          letterSpacing: teamLetterSpacing,
          textTransform: "uppercase",
          textShadow: `0 0 30px ${COLORS.cyan}40`,
        }}>
          OORUM BLOOD
        </div>
      </div>

      {/* Department tag */}
      <div style={{
        position: "absolute",
        top: 260,
        left: "50%",
        transform: "translateX(-50%)",
        opacity: deptOpacity,
        fontFamily: "'Courier New', monospace",
        fontSize: 24,
        color: COLORS.textMuted,
        letterSpacing: 4,
        textAlign: "center",
        whiteSpace: "nowrap",
      }}>
        Indian Institute of Technology Kharagpur
      </div>

      {/* PS tag */}
      <div style={{
        position: "absolute",
        top: 300,
        left: "50%",
        transform: "translateX(-50%)",
        opacity: deptOpacity,
        fontFamily: "'Courier New', monospace",
        fontSize: 18,
        color: COLORS.cyan,
        letterSpacing: 5,
        textAlign: "center",
      }}>
        PS-26037 (MathWorks)
      </div>

      {/* Closing tagline */}
      <div style={{
        position: "absolute",
        top: 380,
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
      }}>
        Autonomous Mobility Built for the Roads India Actually Has.
      </div>
    </AbsoluteFill>
  );
};
