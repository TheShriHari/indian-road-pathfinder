import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, MIN_CLEARANCE_M, FPS, SPRING_PRESETS } from "../../theme";
import { VehicleSprite } from "../../components/VehicleSprite";
import { HudLabel } from "../../components/HudLabel";

const VEHICLE_X = 960;

const ROAD_LEFT = 680;
const ROAD_RIGHT = 1240;

const PINCH_Y = 400;
const PINCH_LEFT = 820;
const PINCH_RIGHT = 1070;

const STOP_LINE_Y = 560;

export const CorridorCheck: React.FC = () => {
  const frame = useCurrentFrame();

  const sceneOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const hudOpacity = interpolate(frame, [20, 50], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Golden caliper arms slide in with spring overshoot physics (Section 1.5)
  const caliperSpring = spring({
    frame: Math.max(0, frame - 15),
    fps: FPS,
    config: SPRING_PRESETS.overshoot,
  });

  // Calipers animate from road edges to pinch point
  const leftCaliperX = interpolate(caliperSpring, [0, 1], [ROAD_LEFT, PINCH_LEFT]);
  const rightCaliperX = interpolate(caliperSpring, [0, 1], [ROAD_RIGHT, PINCH_RIGHT]);

  // Measured width dynamically decreases from 3.8m down to 2.35m
  const measuredWidth = interpolate(frame, [15, 120], [3.8, 2.35], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Threshold breach triggers at frame 90 (below 2.55m)
  const isBreached = measuredWidth < MIN_CLEARANCE_M;
  const caliperColor = isBreached ? COLORS.red : COLORS.amber;

  // Virtual stop line deploys via a crisp horizontal wipe (frames 100 to 130)
  const stopLineWipe = interpolate(frame, [100, 130], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Shockwave ripple expanding across road when stop line strikes
  const rippleRadius = interpolate(frame, [115, 180], [0, 360], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const rippleOpacity = interpolate(frame, [115, 130, 180], [0, 0.8, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Ego vehicle halts cleanly at y: 720 with deceleration squash
  const egoSpeed = interpolate(frame, [0, 80, 140], [14, 8, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const egoY = interpolate(frame, [0, 140], [860, 720], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Rickshaw moves forward to clear the corridor (frames 270 to 380)
  const rickshawY = interpolate(frame, [270, 380], [PINCH_Y - 35, PINCH_Y - 340], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // At frames 360 to 450, stop line shatters into green particles (transitioning into Beat 7)
  const shatterProgress = interpolate(frame, [360, 430], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const numParticles = 28;
  const particles = Array.from({ length: numParticles }).map((_, i) => {
    const baseX = ROAD_LEFT + 20 + ((ROAD_RIGHT - ROAD_LEFT - 40) / numParticles) * i;
    const angle = (i * 137.5 * Math.PI) / 180;
    const dist = shatterProgress * (40 + (i % 5) * 18);
    const px = baseX + Math.cos(angle) * dist;
    const py = STOP_LINE_Y + Math.sin(angle) * dist - shatterProgress * 80;
    return { px, py, alpha: 1 - shatterProgress * 0.7 };
  });

  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg, opacity: sceneOpacity, overflow: "hidden" }}>
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
        <rect x={ROAD_LEFT} y={0} width={ROAD_RIGHT - ROAD_LEFT} height={1080} fill={COLORS.road} />
        <rect x={ROAD_LEFT} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />
        <rect x={ROAD_RIGHT - 8} y={0} width={8} height={1080} fill={COLORS.curb} opacity={0.6} />

        {/* Cow obstacle (left) */}
        <ellipse
          cx={PINCH_LEFT - 30}
          cy={PINCH_Y}
          rx={60}
          ry={35}
          fill={`${COLORS.amber}25`}
          stroke={COLORS.amber}
          strokeWidth={2}
        />
        <text
          x={PINCH_LEFT - 30}
          y={PINCH_Y + 6}
          fill={COLORS.amber}
          fontSize={16}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          fontWeight="bold"
        >
          CATTLE
        </text>

        {/* Rickshaw obstacle (right) */}
        <g transform={`translate(${PINCH_RIGHT + 15}, ${rickshawY})`}>
          <rect x={0} y={0} width={64} height={75} rx={8} fill={`${COLORS.orange}25`} stroke={COLORS.orange} strokeWidth={2} />
          <text
            x={32}
            y={42}
            fill={COLORS.orange}
            fontSize={14}
            fontFamily="'Courier New', monospace"
            textAnchor="middle"
            fontWeight="bold"
          >
            RICKSHAW
          </text>
        </g>

        {/* ---- CALIPERS ---- */}
        {/* Left caliper arm */}
        <g>
          <line
            x1={leftCaliperX}
            y1={PINCH_Y - 90}
            x2={leftCaliperX}
            y2={PINCH_Y + 90}
            stroke={caliperColor}
            strokeWidth={4}
            style={{ filter: `drop-shadow(0 0 8px ${caliperColor})` }}
          />
          <line x1={leftCaliperX} y1={PINCH_Y} x2={leftCaliperX + 24} y2={PINCH_Y} stroke={caliperColor} strokeWidth={3} />
        </g>

        {/* Right caliper arm */}
        <g>
          <line
            x1={rightCaliperX}
            y1={PINCH_Y - 90}
            x2={rightCaliperX}
            y2={PINCH_Y + 90}
            stroke={caliperColor}
            strokeWidth={4}
            style={{ filter: `drop-shadow(0 0 8px ${caliperColor})` }}
          />
          <line x1={rightCaliperX} y1={PINCH_Y} x2={rightCaliperX - 24} y2={PINCH_Y} stroke={caliperColor} strokeWidth={3} />
        </g>

        {/* Width dashed measurement line */}
        <line
          x1={leftCaliperX}
          y1={PINCH_Y}
          x2={rightCaliperX}
          y2={PINCH_Y}
          stroke={caliperColor}
          strokeWidth={2}
          strokeDasharray="8 6"
        />

        {/* Measured Clearance label */}
        <text
          x={(leftCaliperX + rightCaliperX) / 2}
          y={PINCH_Y - 24}
          fill={caliperColor}
          fontSize={34}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          fontWeight="bold"
          style={{ filter: `drop-shadow(0 0 10px ${caliperColor}80)` }}
        >
          {measuredWidth.toFixed(2)}m
        </text>

        {/* Threshold indicator */}
        <text
          x={(leftCaliperX + rightCaliperX) / 2}
          y={PINCH_Y - 60}
          fill={COLORS.textMuted}
          fontSize={16}
          fontFamily="'Courier New', monospace"
          textAnchor="middle"
          letterSpacing={2}
        >
          {`AUDITED THRESHOLD: ${MIN_CLEARANCE_M.toFixed(2)}m`}
        </text>

        {/* Stop line shockwave ripple */}
        {rippleOpacity > 0 && (
          <ellipse
            cx={960}
            cy={STOP_LINE_Y}
            rx={rippleRadius}
            ry={rippleRadius * 0.35}
            fill="none"
            stroke={COLORS.red}
            strokeWidth={3}
            opacity={rippleOpacity}
          />
        )}

        {/* Virtual Stop Line with horizontal wipe */}
        {shatterProgress < 1 && stopLineWipe > 0 && (
          <g opacity={1 - shatterProgress}>
            <line
              x1={ROAD_LEFT + 10}
              y1={STOP_LINE_Y}
              x2={ROAD_LEFT + 10 + (ROAD_RIGHT - ROAD_LEFT - 20) * stopLineWipe}
              y2={STOP_LINE_Y}
              stroke={COLORS.red}
              strokeWidth={5}
              strokeDasharray="24 12"
              style={{ filter: `drop-shadow(0 0 12px ${COLORS.red})` }}
            />
            <text
              x={960}
              y={STOP_LINE_Y + 32}
              fill={COLORS.red}
              fontSize={20}
              fontFamily="'Courier New', monospace"
              textAnchor="middle"
              letterSpacing={4}
              fontWeight="bold"
            >
              VIRTUAL STOP LINE ENGAGED
            </text>
          </g>
        )}

        {/* Stop line shatter particles (transitioning into Beat 7's green spline) */}
        {shatterProgress > 0 &&
          particles.map((p, i) => (
            <circle
              key={`p-${i}`}
              cx={p.px}
              cy={p.py}
              r={4}
              fill={COLORS.green}
              opacity={p.alpha}
              style={{ filter: `drop-shadow(0 0 6px ${COLORS.green})` }}
            />
          ))}
      </svg>

      {/* Ego vehicle halting cleanly before the virtual stop line */}
      <VehicleSprite x={VEHICLE_X} y={egoY} speedKmh={egoSpeed} color={COLORS.cyan} />

      {/* HUD Header */}
      <HudLabel
        text="CORRIDOR CHECK · MIN CLEARANCE 2.55m AUDIT"
        x={60}
        y={85}
        opacity={hudOpacity}
        fontSize={22}
        color={COLORS.cyan}
      />

      {/* Step badge */}
      <div
        style={{
          position: "absolute",
          top: 85,
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
        STEP 4 / 7
      </div>

      {/* Status Banner */}
      <div
        style={{
          position: "absolute",
          bottom: 85,
          left: "50%",
          transform: "translateX(-50%)",
          padding: "12px 36px",
          borderRadius: 16,
          border: `2px solid ${isBreached ? COLORS.red : COLORS.green}`,
          backgroundColor: `${COLORS.bg}F0`,
          boxShadow: `0 0 20px ${isBreached ? COLORS.red + "60" : COLORS.green + "60"}`,
          fontFamily: "'Courier New', monospace",
          fontSize: 22,
          fontWeight: 700,
          color: isBreached ? COLORS.red : COLORS.green,
          letterSpacing: 3,
        }}
      >
        {shatterProgress > 0
          ? "CORRIDOR CLEARED · SEEDING TRAJECTORY"
          : isBreached
          ? "⚠ CLEARANCE < 2.55m · YIELD_WAIT TRIGGERED"
          : "✓ CLEARANCE ADEQUATE"}
      </div>
    </AbsoluteFill>
  );
};
