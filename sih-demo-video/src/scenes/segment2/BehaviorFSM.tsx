import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FSM_STATES, FPS, SPRING_PRESETS } from "../../theme";
import { StatePill } from "../../components/StatePill";
import { HudLabel } from "../../components/HudLabel";
import { VehicleSprite } from "../../components/VehicleSprite";

export const BehaviorFSM: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Determine current active FSM state
  let stateIdx = 0;
  for (let i = FSM_STATES.length - 1; i >= 0; i--) {
    if (frame >= FSM_STATES[i].start) {
      stateIdx = i;
      break;
    }
  }
  const currentState = FSM_STATES[stateIdx];

  // Exact speed computation across the 5 states
  const rawSpeed = interpolate(
    frame,
    [0, 120, 240, 360, 380, 480, 520, 600],
    [18.0, 18.0, 12.0, 5.0, 0.0, 0.0, 8.0, 12.0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );
  // Strict clamp to 0.0 during YIELD_WAIT
  const displaySpeed = currentState.label === "YIELD_WAIT" ? 0.0 : rawSpeed;

  // Ego vehicle traverses forward along the road
  const egoY = interpolate(
    frame,
    [0, 120, 240, 360, 480, 600],
    [850, 720, 600, 520, 520, 360],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Ego lateral offset (nudging left around hazard)
  const egoX = interpolate(
    frame,
    [0, 120, 180, 240, 360, 480, 600],
    [960, 960, 910, 920, 940, 940, 960],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Dynamic steering angle computed from lateral offset rate of change
  const wheelAngle = interpolate(
    frame,
    [120, 160, 220, 260, 480, 540, 600],
    [0, -18, 12, 0, 8, -6, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Stop line visible during YIELD states (frames 240 to 480)
  const inYield = frame >= 240 && frame < 480;
  const stopLineOpacity = inYield ? (Math.sin(frame * 0.4) > 0 ? 0.9 : 0.4) : 0;

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        overflow: "hidden",
      }}
    >
      <svg width={1920} height={1080} style={{ position: "absolute", top: 0, left: 0 }}>
        {/* Grid */}
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

        {/* Road */}
        <rect x={680} y={0} width={560} height={1080} fill={COLORS.road} />
        <rect x={680} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />
        <rect x={1232} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />

        {/* Drawn planned spline (carried from Beat 7) */}
        <path
          d="M 960 850 L 960 720 Q 910 660 920 600 L 940 520 L 960 360 L 960 160"
          stroke={COLORS.green}
          strokeWidth={3.5}
          fill="none"
          strokeDasharray="6 6"
          opacity={0.7}
        />

        {/* Stop Line during Yield states */}
        {stopLineOpacity > 0 && (
          <g opacity={stopLineOpacity}>
            <line
              x1={688}
              y1={480}
              x2={1232}
              y2={480}
              stroke={COLORS.red}
              strokeWidth={4.5}
              strokeDasharray="20 12"
            />
            <text
              x={960}
              y={465}
              fill={COLORS.red}
              fontSize={16}
              fontFamily="'Courier New', monospace"
              textAnchor="middle"
              letterSpacing={3}
              fontWeight="bold"
            >
              VIRTUAL STOP LINE
            </text>
          </g>
        )}

        {/* Dynamic hazards */}
        <ellipse cx={820} cy={380} rx={55} ry={32} fill={`${COLORS.amber}20`} stroke={COLORS.amber} strokeWidth={1.5} />
        <text x={820} y={385} fill={COLORS.amber} fontSize={14} fontFamily="'Courier New', monospace" textAnchor="middle">
          CATTLE
        </text>

        <rect x={1080} y={350} width={55} height={65} rx={6} fill={`${COLORS.orange}18`} stroke={COLORS.orange} strokeWidth={1.5} />
        <text x={1107} y={388} fill={COLORS.orange} fontSize={13} fontFamily="'Courier New', monospace" textAnchor="middle">
          RICKSHAW
        </text>
      </svg>

      {/* Ego vehicle physically moving and squashing/stretching based on speed & steering */}
      <VehicleSprite
        x={egoX}
        y={egoY}
        speedKmh={displaySpeed}
        wheelAngle={wheelAngle}
        color={currentState.color}
      />

      {/* Top FSM State Pills Row */}
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          gap: 20,
          position: "absolute",
          top: 110,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: hudOpacity,
        }}
      >
        {FSM_STATES.map((state, idx) => {
          const isCurrent = idx === stateIdx;
          const isPast = idx < stateIdx;
          const pop = spring({
            frame: isCurrent ? frame - state.start : 0,
            fps: FPS,
            config: SPRING_PRESETS.overshoot,
          });
          const scale = isCurrent ? interpolate(pop, [0, 1], [0.95, 1.08]) : 1.0;

          return (
            <div
              key={state.label}
              style={{
                transform: `scale(${scale})`,
                padding: "10px 24px",
                borderRadius: 14,
                border: `2px solid ${isCurrent ? state.color : COLORS.curb}`,
                backgroundColor: isCurrent ? `${state.color}25` : `${COLORS.bg}EE`,
                boxShadow: isCurrent ? `0 0 20px ${state.color}60` : "none",
                display: "flex",
                flexDirection: "column",
                alignItems: "center",
                gap: 2,
                opacity: isCurrent ? 1.0 : isPast ? 0.6 : 0.35,
              }}
            >
              <span
                style={{
                  fontFamily: "'Courier New', monospace",
                  fontSize: 15,
                  fontWeight: 700,
                  letterSpacing: 2,
                  color: isCurrent ? state.color : COLORS.text,
                }}
              >
                {state.label}
              </span>
              <span
                style={{
                  fontFamily: "'Courier New', monospace",
                  fontSize: 12,
                  color: COLORS.textMuted,
                }}
              >
                {state.speedKmh} km/h
              </span>
            </div>
          );
        })}
      </div>

      {/* Primary Centered HUD State Card */}
      <div
        style={{
          position: "absolute",
          bottom: 120,
          left: 100,
        }}
      >
        <StatePill
          label={currentState.label}
          color={currentState.color}
          speedKmh={displaySpeed}
          subtitle={currentState.subtitle}
          opacity={hudOpacity}
        />
      </div>

      {/* Speedometer telemetry gauge */}
      <div
        style={{
          position: "absolute",
          bottom: 120,
          right: 100,
          width: 320,
          padding: "20px 28px",
          borderRadius: 18,
          border: `2px solid ${currentState.color}`,
          backgroundColor: `${COLORS.bg}EE`,
          boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 15px ${currentState.color}15`,
          display: "flex",
          flexDirection: "column",
          gap: 10,
          opacity: hudOpacity,
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between" }}>
          <span style={{ fontFamily: "'Courier New', monospace", fontSize: 16, color: COLORS.textMuted, letterSpacing: 2 }}>
            TELEMETRY SPEED
          </span>
          <span style={{ fontFamily: "'Courier New', monospace", fontSize: 24, fontWeight: 700, color: currentState.color }}>
            {displaySpeed.toFixed(1)} km/h
          </span>
        </div>
        <div style={{ width: "100%", height: 12, backgroundColor: COLORS.road, borderRadius: 6, overflow: "hidden" }}>
          <div
            style={{
              height: "100%",
              width: `${(displaySpeed / 20) * 100}%`,
              backgroundColor: currentState.color,
              boxShadow: `0 0 10px ${currentState.color}`,
            }}
          />
        </div>
        <div style={{ fontFamily: "'Courier New', monospace", fontSize: 13, color: COLORS.textMuted }}>
          LIMIT: 20 km/h · ACCEL: {displaySpeed === 0 ? "0.0 m/s²" : "0.6 m/s²"}
        </div>
      </div>

      {/* HUD Header */}
      <HudLabel
        text="BEHAVIOR FSM · 5-STATE DETERMINISTIC DECISION ENGINE"
        x={60}
        y={80}
        opacity={hudOpacity}
        fontSize={22}
        color={COLORS.cyan}
      />

      {/* Step badge */}
      <div
        style={{
          position: "absolute",
          top: 80,
          right: 80,
          padding: "10px 24px",
          border: `1.5px solid ${COLORS.cyan}`,
          borderRadius: 14,
          backgroundColor: `${COLORS.cyan}15`,
          fontFamily: "'Courier New', monospace",
          fontSize: 18,
          color: COLORS.cyan,
          letterSpacing: 3,
          opacity: hudOpacity,
        }}
      >
        STEP 6 / 7
      </div>
    </AbsoluteFill>
  );
};
