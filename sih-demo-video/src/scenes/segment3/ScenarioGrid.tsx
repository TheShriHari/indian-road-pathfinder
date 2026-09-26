import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS } from "../../theme";
import { HudLabel } from "../../components/HudLabel";

const SCENARIOS = [
  {
    title: "Rural Road",
    subtitle: "Unmarked village track",
    icon: "🛤",
    enterFrame: 8,
  },
  {
    title: "Urban Intersection",
    subtitle: "Signal-less junction",
    icon: "🚦",
    enterFrame: 16,
  },
  {
    title: "Highway Merge",
    subtitle: "High-speed lane entry",
    icon: "🛣",
    enterFrame: 24,
  },
  {
    title: "Market Corridor",
    subtitle: "Dense, narrow squeeze",
    icon: "🏪",
    enterFrame: 32,
  },
  {
    title: "Cattle Crossing",
    subtitle: "Sudden animal obstacle",
    icon: "🐄",
    enterFrame: 40,
  },
] as const;

export const ScenarioGrid: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Header
  const headerOpacity = interpolate(frame, [0, 25], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Border pulse for all cards
  const borderPulse = (Math.sin(frame * 0.06) + 1) / 2; // 0→1 slow sine

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      {/* Grid background */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
        {Array.from({ length: 38 }).map((_, i) => (
          <line key={`v-${i}`} x1={i * 52} y1={0} x2={i * 52} y2={1080}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}
      </svg>

      {/* Header */}
      <div style={{
        position: "absolute",
        top: 100,
        left: "50%",
        transform: "translateX(-50%)",
        opacity: headerOpacity,
        textAlign: "center",
      }}>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 8,
          textTransform: "uppercase",
          marginBottom: 12,
        }}>
          Stress-tested across real Indian road scenarios
        </div>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontSize: 14,
          color: COLORS.textMuted,
          letterSpacing: 4,
        }}>
          Randomized trials · Not a single scripted run
        </div>
      </div>

      {/* Scenario cards */}
      <div style={{
        display: "flex",
        flexDirection: "row",
        gap: 32,
        marginTop: 40,
      }}>
        {SCENARIOS.map((scenario) => {
          const sp = spring({
            frame: Math.max(0, frame - scenario.enterFrame),
            fps: FPS,
            config: { damping: 14, stiffness: 100 },
          });
          const scale = interpolate(sp, [0, 1], [0.5, 1]);
          const opacity = interpolate(sp, [0, 0.3], [0, 1]);

          const glowIntensity = borderPulse;
          const glowAlpha = Math.round(glowIntensity * 80 + 40).toString(16).padStart(2, "0");

          return (
            <div
              key={scenario.title}
              style={{
                opacity,
                transform: `scale(${scale})`,
                width: 280,
                padding: "32px 24px",
                borderRadius: 16,
                border: `1.5px solid ${COLORS.cyan}${glowAlpha}`,
                backgroundColor: `${COLORS.cyan}08`,
                boxShadow: `0 0 ${12 + glowIntensity * 12}px ${COLORS.cyan}${Math.round(glowIntensity * 40).toString(16).padStart(2, "0")}`,
                display: "flex",
                flexDirection: "column",
                alignItems: "center",
                gap: 14,
                textAlign: "center",
              }}
            >
              <div style={{ fontSize: 56 }}>{scenario.icon}</div>
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontWeight: 700,
                fontSize: 22,
                color: COLORS.text,
                letterSpacing: 1,
              }}>
                {scenario.title}
              </div>
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 16,
                color: COLORS.textMuted,
                letterSpacing: 1,
              }}>
                {scenario.subtitle}
              </div>
              {/* Mini status bar */}
              <div style={{
                width: "100%",
                height: 3,
                backgroundColor: COLORS.grid,
                borderRadius: 2,
                overflow: "hidden",
              }}>
                <div style={{
                  height: "100%",
                  width: `${60 + glowIntensity * 40}%`,
                  backgroundColor: COLORS.green,
                  borderRadius: 2,
                }} />
              </div>
              <div style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 14,
                color: COLORS.green,
                letterSpacing: 2,
              }}>
                VALIDATED ✓
              </div>
            </div>
          );
        })}
      </div>

      {/* HUD */}
      <HudLabel
        text="ENGINEERING RIGOR · multi-scenario validation"
        x={60}
        y={60}
        opacity={hudOpacity}
        fontSize={24}
        color={COLORS.cyan}
      />
    </AbsoluteFill>
  );
};
