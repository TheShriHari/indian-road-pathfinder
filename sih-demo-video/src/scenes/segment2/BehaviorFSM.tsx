import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, interpolateColors } from "remotion";
import { COLORS, FSM_STATES } from "../../theme";
import { StatePill } from "../../components/StatePill";
import { HudLabel } from "../../components/HudLabel";

export const BehaviorFSM: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Determine current state index
  let stateIdx = 0;
  for (let i = FSM_STATES.length - 1; i >= 0; i--) {
    if (frame >= FSM_STATES[i].start) {
      stateIdx = i;
      break;
    }
  }

  const currentState = FSM_STATES[stateIdx];
  const nextState = FSM_STATES[Math.min(stateIdx + 1, FSM_STATES.length - 1)];

  // Smooth color transition at boundaries (±10 frames)
  const BLEND_FRAMES = 10;
  const blendStart = currentState.end - BLEND_FRAMES;
  const blendT = interpolate(frame, [blendStart, currentState.end], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const pillColor = frame >= blendStart
    ? interpolateColors(blendT, [0, 1], [currentState.color, nextState.color])
    : currentState.color;

  // Smooth speed interpolation across transitions
  const rawSpeed = interpolate(
    frame,
    FSM_STATES.map((s) => s.start),
    FSM_STATES.map((s) => s.speedKmh),
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Speed Telemetry Sanitization: clamp strictly to 0.0 km/h when in YIELD_WAIT
  const displaySpeed = currentState.label === "YIELD_WAIT" ? 0.0 : rawSpeed;

  // Pill opacity entrance
  const pillOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Stop-line pulsing during YIELD states
  const inYield = frame >= FSM_STATES[3].start && frame < FSM_STATES[4].start;
  const stopLineOpacity = inYield ? (Math.sin(frame * 0.4) > 0 ? 1 : 0.4) : 0;

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
      {/* Background SVG */}
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Grid */}
        {Array.from({ length: 22 }).map((_, i) => (
          <line key={`h-${i}`} x1={0} y1={i * 52} x2={1920} y2={i * 52}
            stroke={COLORS.grid} strokeWidth={1} opacity={0.3} />
        ))}

        {/* Road */}
        <rect x={700} y={0} width={520} height={1080} fill="#161E2E" />
        <rect x={700} y={0} width={8} height={1080} fill="#334155" opacity={0.4} />
        <rect x={1212} y={0} width={8} height={1080} fill="#334155" opacity={0.4} />

        {/* Stop line (visible during YIELD states) */}
        <g opacity={stopLineOpacity}>
          <line x1={708} y1={520} x2={1212} y2={520}
            stroke={COLORS.red} strokeWidth={4} strokeDasharray="20 12" />
          <text x={960} y={505} fill={COLORS.red} fontSize={16}
            fontFamily="'Courier New', monospace" textAnchor="middle" letterSpacing={3}>
            STOP LINE
          </text>
        </g>

        {/* Obstacle markers (for YIELD context) */}
        {frame >= FSM_STATES[2].start && (
          <>
            <ellipse cx={820} cy={400} rx={55} ry={32}
              fill={`${COLORS.amber}20`} stroke={COLORS.amber} strokeWidth={1.5}
              opacity={0.7} />
            <rect x={1060} y={365} width={55} height={65} rx={4}
              fill={`${COLORS.cyan}15`} stroke={COLORS.cyan} strokeWidth={1.5}
              opacity={0.7} />
          </>
        )}
      </svg>

      {/* State pills row — all states shown, current one highlighted */}
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          gap: 24,
          position: "absolute",
          top: 120,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: hudOpacity,
        }}
      >
        {FSM_STATES.map((state) => {
          const isPast = frame > state.end;
          const isCurrent = frame >= state.start && frame < state.end;
          const isFuture = frame < state.start;

          return (
            <div
              key={state.label}
              style={{
                opacity: isFuture ? 0.25 : isPast ? 0.5 : 1,
                transform: isCurrent ? "scale(1.1)" : "scale(1)",
              }}
            >
              <div style={{
                padding: "8px 18px",
                border: `1.5px solid ${state.color}${isCurrent ? "ff" : "60"}`,
                borderRadius: 6,
                backgroundColor: `${state.color}${isCurrent ? "25" : "10"}`,
                fontFamily: "'Courier New', monospace",
                fontSize: 14,
                color: isCurrent ? state.color : COLORS.textMuted,
                letterSpacing: 2,
                textAlign: "center",
              }}>
                {state.label}
              </div>
            </div>
          );
        })}
      </div>

      {/* Arrow indicating flow */}
      <div style={{
        position: "absolute",
        top: 185,
        left: "50%",
        transform: "translateX(-50%)",
        fontFamily: "'Courier New', monospace",
        fontSize: 16,
        color: COLORS.textMuted,
        letterSpacing: 8,
        opacity: hudOpacity * 0.5,
      }}>
        ──────────────────────────────────────────────▶
      </div>

      {/* Main state pill — large, centered */}
      <div style={{
        position: "absolute",
        top: "50%",
        left: "50%",
        transform: "translate(-50%, -50%)",
      }}>
        <StatePill
          label={currentState.label}
          color={pillColor}
          speedKmh={displaySpeed}
          subtitle={currentState.subtitle}
          opacity={pillOpacity}
        />
      </div>

      {/* HUD */}
      <HudLabel
        text="BEHAVIOR FSM · five-state decision layer"
        x={60}
        y={60}
        opacity={hudOpacity}
        fontSize={24}
        color={COLORS.cyan}
      />

      {/* Step badge */}
      <div style={{
        position: "absolute",
        top: 60,
        right: 80,
        padding: "10px 24px",
        border: `1.5px solid ${COLORS.cyan}60`,
        borderRadius: 6,
        backgroundColor: `${COLORS.cyan}10`,
        fontFamily: "'Courier New', monospace",
        fontSize: 18,
        color: COLORS.cyan,
        letterSpacing: 3,
        opacity: hudOpacity,
      }}>
        STEP 6 / 7
      </div>

      {/* Speed readout bar */}
      <div style={{
        position: "absolute",
        bottom: 140,
        left: "50%",
        transform: "translateX(-50%)",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        gap: 8,
        opacity: hudOpacity,
      }}>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.textMuted,
          letterSpacing: 4,
        }}>
          VEHICLE SPEED
        </div>
        <div style={{
          width: 400,
          height: 12,
          backgroundColor: COLORS.grid,
          borderRadius: 6,
          overflow: "hidden",
        }}>
          <div style={{
            height: "100%",
            width: `${(displaySpeed / 20) * 100}%`,
            backgroundColor: pillColor,
            borderRadius: 6,
            boxShadow: `0 0 8px ${pillColor}80`,
          }} />
        </div>
        <div style={{
          fontFamily: "'Courier New', monospace",
          fontSize: 22,
          color: pillColor,
          letterSpacing: 3,
        }}>
          {displaySpeed.toFixed(1)} km/h
        </div>
      </div>
    </AbsoluteFill>
  );
};
