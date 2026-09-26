export const FPS = 30;
export const WIDTH = 1920;
export const HEIGHT = 1080;
export const TOTAL_FRAMES = 2850; // 95s (within 90s–105s target budget)

export const COLORS = {
  bg: "#0B0F19",
  road: "#161E2E",
  grid: "#2A3A52", // Brightened from #1E293B for enhanced contrast
  cyan: "#38BDF8",
  amber: "#F59E0B",
  red: "#EF4444",
  green: "#10B981",
  text: "#F8FAFC",
  textMuted: "#E2E8F0", // Crisp off-white boosted from #94A3B8 / #334155
} as const;

// Segment / scene frame ranges — single source of truth.
// Target total length: 95s = 2850 frames at 30 fps
export const TIMING = {
  segment1: { from: 0, durationInFrames: 450 },
  scene1_1_realityIntro:        { from: 0,    durationInFrames: 150 }, // 5s (trimmed from 12s)
  scene1_2_assumptionBreakdown: { from: 150,  durationInFrames: 180 }, // 6s (accelerated from 10s)
  scene1_3_titleCard:           { from: 330,  durationInFrames: 120 }, // 4s (reduced from 12s)

  segment2: { from: 450, durationInFrames: 1410 },
  step1_perceptionScan:    { from: 450,  durationInFrames: 210 }, // 7s
  step2_motionPrediction:  { from: 660,  durationInFrames: 210 }, // 7s
  step3_costmapBuild:      { from: 870,  durationInFrames: 210 }, // 7s
  step4_corridorCheck:     { from: 1080, durationInFrames: 210 }, // 7s
  step5_pathPlanning:      { from: 1290, durationInFrames: 180 }, // 6s
  step6_behaviorFSM:       { from: 1470, durationInFrames: 240 }, // 8s
  step7_steeringControl:   { from: 1710, durationInFrames: 150 }, // 5s

  segment3: { from: 1860, durationInFrames: 600 },
  scene3_1_scenarioGrid: { from: 1860, durationInFrames: 240 }, // 8s (condensed from 19s)
  scene3_2_rigorKpis:    { from: 2100, durationInFrames: 360 }, // 12s

  segment4: { from: 2460, durationInFrames: 390 },
  scene4_1_closingBranding: { from: 2460, durationInFrames: 390 }, // 13s
} as const;

// Physics constants used across scenes — add here, never inline
export const PX_PER_M = 8;                // pixels per metre in top-down views
export const MIN_CLEARANCE_M = 2.55;      // corridor-check threshold (metres)

// AssumptionBreakdown timing offsets (local frames within the scene, 180 frames total)
export const ASSUMPTION_TIMING = {
  card1Enter: 6,
  card1Strike: { start: 34, end: 48 },
  card2Enter: 46,
  card2Strike: { start: 74, end: 88 },
  card3Enter: 86,
  card3Strike: { start: 114, end: 128 },
} as const;

// BehaviorFSM state local frame ranges (240 frames total)
export const FSM_STATES = [
  { label: "CRUISE",       color: "#38BDF8", speedKmh: 18, subtitle: "Open road traversal",       start: 0,   end: 45  },
  { label: "NUDGE",        color: "#06B6D4", speedKmh: 12, subtitle: "Deflecting around hazard",  start: 45,  end: 90 },
  { label: "YIELD_DECEL",  color: "#F59E0B", speedKmh: 5,  subtitle: "Approaching pinch corridor",start: 90,  end: 140 },
  { label: "YIELD_WAIT",   color: "#EF4444", speedKmh: 0,  subtitle: "Holding at stop line",      start: 140, end: 195 },
  { label: "RESUME",       color: "#10B981", speedKmh: 8,  subtitle: "Accelerating once clear",   start: 195, end: 240 },
] as const;

