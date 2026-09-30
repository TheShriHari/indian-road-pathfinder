import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, SPRING_PRESETS } from "../../theme";
import { HudLabel } from "../../components/HudLabel";

export const BugsFoundFixed: React.FC = () => {
  const frame = useCurrentFrame();

  // Active sub-card: Card 1 (0..240 frames / 8s), Card 2 (240..480 frames / 8s)
  const isCard1 = frame < 240;
  const localFrame = isCard1 ? frame : frame - 240;

  // Spring pop for card entry
  const cardSpring = spring({
    frame: localFrame,
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });
  const cardScale = interpolate(cardSpring, [0, 1], [0.85, 1.0]);

  // Transition dissolve between card 1 and card 2
  const cardOpacity = interpolate(
    localFrame,
    [0, 15, 225, 240],
    [0, 1, 1, isCard1 ? 0 : 1],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Staggered reveal of Fail vs Fixed states
  const failReveal = spring({
    frame: Math.max(0, localFrame - 15),
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });
  const fixedReveal = spring({
    frame: Math.max(0, localFrame - 60),
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });

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
      {/* Subtle tactical grid */}
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
            opacity={0.25}
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
            opacity={0.25}
          />
        ))}
      </svg>

      {/* Top Header HUD */}
      <div style={{ position: "absolute", top: 80, left: 80, display: "flex", flexDirection: "column", gap: 8 }}>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 24,
            fontWeight: 700,
            color: COLORS.cyan,
            letterSpacing: 2,
            textTransform: "uppercase",
            textShadow: `0 0 8px ${COLORS.cyan}80`,
          }}
        >
          FIELD DIAGNOSTICS · FAILURE ANALYSIS & VERIFIED FIXES
        </div>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 16,
            color: COLORS.textMuted,
            letterSpacing: 3,
          }}
        >
          {isCard1
            ? "CASE 01 / 02 · CORNER-CUTTING AT UNPAVED SHOULDER EROSION"
            : "CASE 02 / 02 · SYMMETRIC SLEW LIMITER EMERGENCY BRAKE CHOKE (SEED 438)"}
        </div>
      </div>

      {/* Step Badge */}
      <div
        style={{
          position: "absolute",
          top: 80,
          right: 80,
          padding: "10px 24px",
          border: `1.5px solid ${COLORS.cyan}`,
          borderRadius: 14,
          backgroundColor: `${COLORS.cyan}18`,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 3,
          boxShadow: `0 10px 30px rgba(0,0,0,0.4)`,
        }}
      >
        BEAT 10 / RIGOR
      </div>

      {/* Main Diagnostic Comparison Container */}
      <div
        style={{
          position: "absolute",
          top: 185,
          width: 1760,
          height: 790,
          display: "flex",
          gap: 40,
          justifyContent: "center",
          opacity: cardOpacity,
          transform: `scale(${cardScale})`,
          transformOrigin: "center center",
        }}
      >
        {/* LEFT PANEL: BEFORE / FAILURE STATE (CRIMSON) */}
        <div
          style={{
            flex: 1,
            borderRadius: 20,
            border: `2px solid ${COLORS.red}88`,
            backgroundColor: `${COLORS.bg}F0`,
            boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 20px ${COLORS.red}15`,
            padding: "36px 44px",
            display: "flex",
            flexDirection: "column",
            transform: `scale(${failReveal})`,
            transformOrigin: "center left",
          }}
        >
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
            <span
              style={{
                fontFamily: "'Courier New', monospace",
                fontWeight: 700,
                fontSize: 22,
                letterSpacing: 4,
                color: COLORS.red,
                padding: "6px 16px",
                borderRadius: 10,
                border: `1px solid ${COLORS.red}`,
                backgroundColor: `${COLORS.red}20`,
              }}
            >
              FAILURE MODE (UNPATCHED)
            </span>
            <span
              style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 18,
                color: COLORS.red,
                fontWeight: 600,
              }}
            >
              {isCard1 ? "ENCROACHMENT: -0.18m" : "DECEL CHOKED: -1.67 m/s²"}
            </span>
          </div>

          <h2
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 32,
              color: COLORS.text,
              marginTop: 24,
              marginBottom: 12,
              fontWeight: 700,
            }}
          >
            {isCard1
              ? "Shoulder Corner Cutting"
              : "Symmetric Slew Limiter Choke"}
          </h2>

          <p
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 20,
              lineHeight: 1.5,
              color: COLORS.textMuted,
              marginBottom: 24,
            }}
          >
            {isCard1
              ? "Naive spline optimization minimized path curvature by clipping into unpaved, eroded road boundaries. Vehicles clipped soft dirt shoulders."
              : "Uniform 0.95 m/s³ slew clamp choked braking deceleration to -1.67 m/s² during an emergency squeeze, causing a 23.2 km/h crash despite commanded -8.95 m/s²."}
          </p>

          {/* SVG Diagram: Failure visual */}
          <div
            style={{
              flex: 1,
              borderRadius: 16,
              border: `1px solid ${COLORS.red}40`,
              backgroundColor: "#111827",
              position: "relative",
              overflow: "hidden",
            }}
          >
            <svg width="100%" height="100%" viewBox="0 0 800 360">
              {/* Road surface */}
              <rect x={100} y={40} width={600} height={280} fill={COLORS.road} rx={12} />
              {/* Eroded curb boundary */}
              <path
                d="M 100 80 Q 250 110 400 70 T 700 85"
                stroke={COLORS.red}
                strokeWidth={3}
                fill="none"
                strokeDasharray="6 4"
              />
              <text x={120} y={65} fill={COLORS.red} fontSize={16} fontFamily="'Courier New', monospace">
                EROSED SHOULDER EDGE
              </text>

              {/* Errant path cutting across curb into ditch */}
              <path
                d="M 120 280 C 260 270, 320 60, 680 80"
                stroke={COLORS.red}
                strokeWidth={4}
                fill="none"
              />
              {/* Collision danger circle */}
              <circle cx={345} cy={72} r={18} fill={`${COLORS.red}40`} stroke={COLORS.red} strokeWidth={2} />
              <text x={375} y={78} fill={COLORS.red} fontSize={16} fontFamily="'Courier New', monospace" fontWeight={700}>
                COLLISION / ROLLOVER HAZARD
              </text>
            </svg>
          </div>
        </div>

        {/* RIGHT PANEL: AFTER / FIXED STATE (EMERALD GREEN) */}
        <div
          style={{
            flex: 1,
            borderRadius: 20,
            border: `2px solid ${COLORS.green}88`,
            backgroundColor: `${COLORS.bg}F0`,
            boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 20px ${COLORS.green}15`,
            padding: "36px 44px",
            display: "flex",
            flexDirection: "column",
            transform: `scale(${fixedReveal})`,
            transformOrigin: "center right",
          }}
        >
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
            <span
              style={{
                fontFamily: "'Courier New', monospace",
                fontWeight: 700,
                fontSize: 22,
                letterSpacing: 4,
                color: COLORS.green,
                padding: "6px 16px",
                borderRadius: 10,
                border: `1px solid ${COLORS.green}`,
                backgroundColor: `${COLORS.green}20`,
              }}
            >
              VERIFIED ARCHITECTURAL FIX
            </span>
            <span
              style={{
                fontFamily: "'Courier New', monospace",
                fontSize: 18,
                color: COLORS.green,
                fontWeight: 600,
              }}
            >
              {isCard1 ? "CLEARANCE: +0.65m" : "EMERGENCY SLEW: 8.0 m/s³"}
            </span>
          </div>

          <h2
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 32,
              color: COLORS.text,
              marginTop: 24,
              marginBottom: 12,
              fontWeight: 700,
            }}
          >
            {isCard1
              ? "Voronoi Repulsive Barrier"
              : "Asymmetric Jerk Limiter + Latch"}
          </h2>

          <p
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 20,
              lineHeight: 1.5,
              color: COLORS.textMuted,
              marginBottom: 24,
            }}
          >
            {isCard1
              ? "Augmented cellular costmap with non-linear Voronoi repulsive field from detected boundary contours. Enforces mandatory 0.60m buffer from unpaved drop-offs."
              : "Engineered asymmetric slew limiting: permits up to 8.0 m/s³ deceleration during reflex triggers, while strictly clamping throttle release to 0.95 m/s³ comfort."}
          </p>

          {/* SVG Diagram: Fixed visual */}
          <div
            style={{
              flex: 1,
              borderRadius: 16,
              border: `1px solid ${COLORS.green}40`,
              backgroundColor: "#111827",
              position: "relative",
              overflow: "hidden",
            }}
          >
            <svg width="100%" height="100%" viewBox="0 0 800 360">
              {/* Road surface */}
              <rect x={100} y={40} width={600} height={280} fill={COLORS.road} rx={12} />

              {/* Repulsive buffer zone */}
              <path
                d="M 100 120 Q 250 150 400 110 T 700 125"
                stroke={`${COLORS.green}60`}
                strokeWidth={2}
                fill="none"
                strokeDasharray="4 4"
              />
              <text x={120} y={110} fill={COLORS.green} fontSize={15} fontFamily="'Courier New', monospace">
                0.60m MANDATORY SAFETY BUFFER
              </text>

              {/* Safe trajectory smoothly avoiding shoulder */}
              <path
                d="M 120 280 C 260 270, 320 160, 680 180"
                stroke={COLORS.green}
                strokeWidth={4.5}
                fill="none"
              />

              {/* Verified pass badge */}
              <circle cx={420} cy={165} r={18} fill={`${COLORS.green}40`} stroke={COLORS.green} strokeWidth={2} />
              <text x={450} y={171} fill={COLORS.green} fontSize={16} fontFamily="'Courier New', monospace" fontWeight={700}>
                STABLE CLEARANCE MAINTAINED
              </text>
            </svg>
          </div>
        </div>
      </div>
    </AbsoluteFill>
  );
};
