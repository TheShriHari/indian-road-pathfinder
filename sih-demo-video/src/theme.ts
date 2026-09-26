export const FPS = 30;
export const WIDTH = 1920;
export const HEIGHT = 1080;
export const TOTAL_FRAMES = 4680; // 156.0s (within 150–180s target budget)

export const COLORS = {
  bg: "#0B111E",         // Oscilloscope dark slate background
  road: "#172033",       // Raw asphalt fill
  curb: "#2D3B55",       // Unpaved road shoulder / boundary line
  grid: "#1E293B",       // Subtle telemetry coordinate grid (15-25% opacity)
  cyan: "#00F0FF",       // Primary telemetry accent / planned path / ego state
  amber: "#FFB703",      // Warning / caution band / dynamic cattle
  orange: "#FB8500",     // Dynamic rickshaw / secondary hazard
  red: "#FF0054",        // Critical threshold / Virtual Stop Line / crash failure
  green: "#06D6A0",      // Safe clearance / verified fix / nominal comfort band
  text: "#F8FAFC",       // Off-white monospace readouts
  textMuted: "#94A3B8",  // Secondary technical parameters / units
} as const;

// Chrome-only spring physics (UI panels / section badges only — NEVER on telemetry)
export const SPRING_PRESETS = {
  chromeOvershoot: { stiffness: 180, damping: 14, mass: 0.8 },
  gentleFade: { stiffness: 120, damping: 16, mass: 1.0 },
} as const;

// 7-Segment Evidence Timeline Architecture (total: 4680 frames = 156.0s @ 30 FPS)
export const TIMING = {
  totalFrames: 4680,
  segment1_hookFailure:           { from: 0,    durationInFrames: 450 },  // 15s (0:00 - 0:15)
  segment2_problemFraming:        { from: 450,  durationInFrames: 450 },  // 15s (0:15 - 0:30)
  segment3_architectureWalkthrough:{ from: 900,  durationInFrames: 1200 }, // 40s (0:30 - 1:10)
  segment4_bottleneckYieldDemo:   { from: 2100, durationInFrames: 900 },  // 30s (1:10 - 1:40) [CENTERPIECE]
  segment5_validationRigor:       { from: 3000, durationInFrames: 750 },  // 25s (1:40 - 2:05)
  segment6_honestLimitations:     { from: 3750, durationInFrames: 390 },  // 13s (2:05 - 2:18)
  segment7_closingBranding:       { from: 4140, durationInFrames: 540 },  // 18s (2:18 - 2:36)
} as const;

// Audited Physical Constants & Constraints
export const PX_PER_M = 16;                    // pixels per metre for high-fidelity technical views
export const HERO_CORRIDOR_WIDTH_M = 2.55;     // hero bottleneck drivable threshold
export const MIN_TURNING_RADIUS_M = 3.8;       // Ackermann kinodynamic minimum turning radius
export const MAX_STEERING_RATE_DEG_S = 25;     // maximum physical steering rate limit
export const COMFORT_JERK_LIMIT_MPS3 = 0.80;   // nominal longitudinal comfort jerk ceiling
export const EMERGENCY_JERK_LIMIT_MPS3 = 8.00; // emergency braking reflex jerk limit
export const TARGET_HARDWARE = "AMD Ryzen 5 7530U (6C/12T @ 2.0-4.5 GHz)";
