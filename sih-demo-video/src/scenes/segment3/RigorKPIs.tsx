import React from "react";
import { AbsoluteFill, useCurrentFrame, interpolate, spring } from "remotion";
import { COLORS, FPS, SPRING_PRESETS } from "../../theme";
import { HudLabel } from "../../components/HudLabel";

const SCENARIOS = [
  { id: "cattle",    label: "01 · Stray Cattle Crossing",    status: "PASS (0.0 coll)" },
  { id: "rickshaw",  label: "02 · Wrong-Way Rickshaw",      status: "PASS (0.0 coll)" },
  { id: "slalom",    label: "03 · Pothole Cluster Slalom",  status: "PASS (0.0 coll)" },
  { id: "pinch",     label: "04 · Pedestrian Corridor Pinch", status: "PASS (0.0 coll)" },
  { id: "shoulder",  label: "05 · Eroded Shoulder Drop-off", status: "PASS (0.0 coll)" },
] as const;

export const RigorKPIs: React.FC = () => {
  const frame = useCurrentFrame();

  const hudOpacity = interpolate(frame, [0, 20], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const headerOpacity = interpolate(frame, [0, 25], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Staggered springs for the 3 main cards
  const sp1 = spring({ frame: Math.max(0, frame - 15), fps: FPS, config: SPRING_PRESETS.overshoot });
  const sp2 = spring({ frame: Math.max(0, frame - 35), fps: FPS, config: SPRING_PRESETS.overshoot });
  const sp3 = spring({ frame: Math.max(0, frame - 55), fps: FPS, config: SPRING_PRESETS.overshoot });

  const card1Scale = interpolate(sp1, [0, 1], [0.85, 1.0]);
  const card2Scale = interpolate(sp2, [0, 1], [0.85, 1.0]);
  const card3Scale = interpolate(sp3, [0, 1], [0.85, 1.0]);

  // Bottom scenario strip fade-in (calm entrance, no jumpy springs)
  const stripOpacity = interpolate(frame, [80, 120], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

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
      {/* Subtle grid background */}
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

      {/* Top Header */}
      <div
        style={{
          position: "absolute",
          top: 80,
          left: "50%",
          transform: "translateX(-50%)",
          opacity: headerOpacity,
          textAlign: "center",
        }}
      >
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 22,
            color: COLORS.cyan,
            letterSpacing: 8,
            textTransform: "uppercase",
            fontWeight: 700,
          }}
        >
          Quantitative Engineering Rigor
        </div>
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 16,
            color: COLORS.textMuted,
            letterSpacing: 3,
            marginTop: 6,
          }}
        >
          REPRODUCIBLE BENCHMARKS · CLOSED-LOOP SIMULATION & EMBEDDED TELEMETRY
        </div>
      </div>

      {/* 3 Main KPI Cards Row */}
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          gap: 40,
          alignItems: "stretch",
          position: "absolute",
          top: 185,
          width: 1740,
        }}
      >
        {/* Card 1: 1,000 Trials */}
        <div
          style={{
            flex: 1,
            padding: "36px 32px",
            borderRadius: 20,
            border: `2px solid ${COLORS.cyan}80`,
            backgroundColor: `${COLORS.bg}EE`,
            boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 20px ${COLORS.cyan}15`,
            transform: `scale(${card1Scale})`,
            display: "flex",
            flexDirection: "column",
            gap: 14,
          }}
        >
          <div style={{ height: 4, width: "100%", backgroundColor: COLORS.cyan, borderRadius: 2 }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 54,
              color: COLORS.cyan,
              lineHeight: 1,
            }}
          >
            90.6%
          </div>
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 24,
              color: COLORS.text,
              lineHeight: 1.2,
            }}
          >
            Scenario Completion Rate
          </div>
          <div style={{ height: 1, backgroundColor: `${COLORS.cyan}30`, width: "100%" }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 17,
              color: COLORS.textMuted,
              lineHeight: 1.5,
            }}
          >
            436 Successes + 470 Safe Stops across 1,000 randomized Monte Carlo trials. 4.2% collision rate, down 56.25% from baseline.
          </div>
          <div
            style={{
              marginTop: "auto",
              fontFamily: "'Courier New', monospace",
              fontSize: 15,
              color: COLORS.cyan,
              letterSpacing: 2,
              fontWeight: 600,
            }}
          >
            ✓ 90.6% SCENARIO COMPLETION · 4.2% COLLISION
          </div>
        </div>

        {/* Card 2: 0 Toolboxes */}
        <div
          style={{
            flex: 1,
            padding: "36px 32px",
            borderRadius: 20,
            border: `2px solid ${COLORS.green}80`,
            backgroundColor: `${COLORS.bg}EE`,
            boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 20px ${COLORS.green}15`,
            transform: `scale(${card2Scale})`,
            display: "flex",
            flexDirection: "column",
            gap: 14,
          }}
        >
          <div style={{ height: 4, width: "100%", backgroundColor: COLORS.green, borderRadius: 2 }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 54,
              color: COLORS.green,
              lineHeight: 1,
            }}
          >
            ZERO
          </div>
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 24,
              color: COLORS.text,
              lineHeight: 1.2,
            }}
          >
            External Toolboxes
          </div>
          <div style={{ height: 1, backgroundColor: `${COLORS.green}30`, width: "100%" }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 17,
              color: COLORS.textMuted,
              lineHeight: 1.5,
            }}
          >
            First-principles vectorized matrix mathematics in 100% MATLAB and Python. Zero dependency on proprietary toolboxes.
          </div>
          <div
            style={{
              marginTop: "auto",
              fontFamily: "'Courier New', monospace",
              fontSize: 15,
              color: COLORS.green,
              letterSpacing: 2,
              fontWeight: 600,
            }}
          >
            ✓ 100% FIRST-PRINCIPLES MAT/PY
          </div>
        </div>

        {/* Card 3: Measured Latency */}
        <div
          style={{
            flex: 1,
            padding: "36px 32px",
            borderRadius: 20,
            border: `2px solid ${COLORS.amber}80`,
            backgroundColor: `${COLORS.bg}EE`,
            boxShadow: `0 10px 30px rgba(0,0,0,0.5), inset 0 0 20px ${COLORS.amber}15`,
            transform: `scale(${card3Scale})`,
            display: "flex",
            flexDirection: "column",
            gap: 14,
          }}
        >
          <div style={{ height: 4, width: "100%", backgroundColor: COLORS.amber, borderRadius: 2 }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 54,
              color: COLORS.amber,
              lineHeight: 1,
            }}
          >
            3.6 ms
          </div>
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontWeight: 700,
              fontSize: 24,
              color: COLORS.text,
              lineHeight: 1.2,
            }}
          >
            Mean Replan Latency
          </div>
          <div style={{ height: 1, backgroundColor: `${COLORS.amber}30`, width: "100%" }} />
          <div
            style={{
              fontFamily: "'Courier New', monospace",
              fontSize: 17,
              color: COLORS.textMuted,
              lineHeight: 1.5,
            }}
          >
            Real-time candidate path sampling and collision checking (42.85 ms P99) under 10 Hz closed-loop control.
          </div>
          <div
            style={{
              marginTop: "auto",
              fontFamily: "'Courier New', monospace",
              fontSize: 15,
              color: COLORS.amber,
              letterSpacing: 2,
              fontWeight: 600,
            }}
          >
            ✓ 10 Hz REAL-TIME CLOSED LOOP
          </div>
        </div>
      </div>

      {/* Bottom Scenario Strip: ScenarioGrid folded into a calm 5-badge strip */}
      <div
        style={{
          position: "absolute",
          bottom: 70,
          width: 1740,
          opacity: stripOpacity,
          display: "flex",
          flexDirection: "column",
          gap: 14,
        }}
      >
        <div
          style={{
            fontFamily: "'Courier New', monospace",
            fontSize: 15,
            color: COLORS.textMuted,
            letterSpacing: 4,
            textTransform: "uppercase",
            textAlign: "center",
          }}
        >
          MULTI-SCENARIO STRESS BENCHMARK SUITE
        </div>
        <div style={{ display: "flex", gap: 16 }}>
          {SCENARIOS.map((sc) => (
            <div
              key={sc.id}
              style={{
                flex: 1,
                padding: "14px 18px",
                borderRadius: 14,
                border: `1px solid ${COLORS.curb}`,
                backgroundColor: `${COLORS.road}CC`,
                display: "flex",
                flexDirection: "column",
                gap: 4,
              }}
            >
              <span
                style={{
                  fontFamily: "'Courier New', monospace",
                  fontSize: 15,
                  fontWeight: 600,
                  color: COLORS.text,
                  whiteSpace: "nowrap",
                  overflow: "hidden",
                  textOverflow: "ellipsis",
                }}
              >
                {sc.label}
              </span>
              <span
                style={{
                  fontFamily: "'Courier New', monospace",
                  fontSize: 13,
                  color: COLORS.green,
                  fontWeight: 700,
                  letterSpacing: 1,
                }}
              >
                {sc.status}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* HUD Header */}
      <HudLabel
        text="VERIFICATION · REAL TEST METRICS"
        x={60}
        y={80}
        opacity={hudOpacity}
        fontSize={22}
        color={COLORS.cyan}
      />
    </AbsoluteFill>
  );
};
