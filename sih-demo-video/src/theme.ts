export const FPS = 30;
export const WIDTH = 1920;
export const HEIGHT = 1080;
export const TOTAL_FRAMES = 5160; // 172s (within 165–180s target budget)

export const COLORS = {
  bg: "#0B111E",         // Base canvas/background everywhere
  road: "#172033",       // All road/asphalt fills
  curb: "#2D3B55",       // Road edges, shoulder erosion strokes
  grid: "#2D3B55",       // Grid texture lines (15-30% opacity)
  cyan: "#00F0FF",       // Electric cyan: vehicle chassis, canopy glow, primary HUD accents
  amber: "#FFB703",      // Golden amber: dynamic hazards (cattle/rickshaw), calipers
  orange: "#FB8500",     // Safety orange: hazard vectors, dynamic markers
  red: "#FF0054",        // Neon crimson: lethal zones, virtual stop lines, fail state
  green: "#06D6A0",      // Emerald green: safe path spline, SUCCESS states, fixed state
  text: "#F8FAFC",       // Crisp off-white monospace
  textMuted: "#94A3B8",  // Secondary muted labels
} as const;

// Spring physics presets (Section 1.5)
export const SPRING_PRESETS = {
  overshoot: { stiffness: 180, damping: 12, mass: 0.7 }, // ~112% overshoot for UI/caliper arrivals
  gentle: { stiffness: 120, damping: 14, mass: 0.9 },
  camera: { stiffness: 90, damping: 18, mass: 1.0 },
} as const;

// 12-Beat Timeline Architecture (total: 5160 frames = 172.0s @ 30 FPS)
export const TIMING = {
  totalFrames: 5160,
  beat1_titleCard:        { from: 0,    durationInFrames: 180 }, // 6s (0:00 - 0:06)
  beat2_problemStatement: { from: 180,  durationInFrames: 540 }, // 18s (0:06 - 0:24)
  beat3_perceptionScan:   { from: 720,  durationInFrames: 420 }, // 14s (0:24 - 0:38)
  beat4_motionPrediction: { from: 1140, durationInFrames: 420 }, // 14s (0:38 - 0:52)
  beat5_costmapBuild:     { from: 1560, durationInFrames: 390 }, // 13s (0:52 - 1:05)
  beat6_corridorCheck:    { from: 1950, durationInFrames: 450 }, // 15s (1:05 - 1:20)
  beat7_pathPlanning:     { from: 2400, durationInFrames: 420 }, // 14s (1:20 - 1:34)
  beat8_behaviorFSM:      { from: 2820, durationInFrames: 600 }, // 20s (1:34 - 1:54)
  beat9_steeringControl:  { from: 3420, durationInFrames: 360 }, // 12s (1:54 - 2:06)
  beat10_bugsFoundFixed:  { from: 3780, durationInFrames: 480 }, // 16s (2:06 - 2:22)
  beat11_rigorKpis:       { from: 4260, durationInFrames: 420 }, // 14s (2:22 - 2:36)
  beat12_closingBranding: { from: 4680, durationInFrames: 480 }, // 16s (2:36 - 2:52)
} as const;

// Physics constants used across scenes
export const PX_PER_M = 8;                // pixels per metre in top-down views
export const MIN_CLEARANCE_M = 2.55;      // audited corridor-check threshold (metres)
export const MAX_STEERING_RATE_DEG_S = 25; // max physical steering rate limit

export const ASSUMPTION_TIMING = {
  card1Enter: 6,
  card1Strike: { start: 24, end: 38 },
  card2Enter: 36,
  card2Strike: { start: 54, end: 68 },
  card3Enter: 66,
  card3Strike: { start: 84, end: 98 },
} as const;

// BehaviorFSM state local frame ranges (600 frames total = 20s, 4s / 120 frames per state)
export const FSM_STATES = [
  { label: "CRUISE",       color: "#00F0FF", speedKmh: 18, subtitle: "Open road traversal",        start: 0,   end: 120 },
  { label: "NUDGE",        color: "#38BDF8", speedKmh: 12, subtitle: "Deflecting around hazard",   start: 120, end: 240 },
  { label: "YIELD_DECEL",  color: "#FFB703", speedKmh: 5,  subtitle: "Approaching pinch corridor", start: 240, end: 360 },
  { label: "YIELD_WAIT",   color: "#FF0054", speedKmh: 0,  subtitle: "Holding at stop line",       start: 360, end: 480 },
  { label: "RESUME",       color: "#06D6A0", speedKmh: 8,  subtitle: "Accelerating once clear",    start: 480, end: 600 },
] as const;


