import React from "react";
import { AbsoluteFill, Sequence } from "remotion";
import { TIMING, COLORS } from "./theme";
import { HookFailure } from "./scenes/evidence/HookFailure";
import { ProblemFraming } from "./scenes/evidence/ProblemFraming";
import { ArchitectureWalkthrough } from "./scenes/evidence/ArchitectureWalkthrough";
import { BottleneckYieldDemo } from "./scenes/evidence/BottleneckYieldDemo";
import { ValidationRigor } from "./scenes/evidence/ValidationRigor";
import { HonestLimitations } from "./scenes/evidence/HonestLimitations";
import { ClosingBranding } from "./scenes/evidence/ClosingBranding";

export const PS26037Explainer: React.FC = () => (
  <AbsoluteFill style={{ backgroundColor: COLORS.bg }}>
    {/* Segment 1: Hook — The Failure, Not The Solution (0:00 - 0:15 / 450 frames) */}
    <Sequence {...TIMING.segment1_hookFailure}>
      <HookFailure />
    </Sequence>

    {/* Segment 2: Problem Framing (0:15 - 0:30 / 450 frames) */}
    <Sequence {...TIMING.segment2_problemFraming}>
      <ProblemFraming />
    </Sequence>

    {/* Segment 3: Architecture Walkthrough (0:30 - 1:10 / 1200 frames) */}
    <Sequence {...TIMING.segment3_architectureWalkthrough}>
      <ArchitectureWalkthrough />
    </Sequence>

    {/* Segment 4: Bottleneck / Yield Centerpiece Demo (1:10 - 1:40 / 900 frames) */}
    <Sequence {...TIMING.segment4_bottleneckYieldDemo}>
      <BottleneckYieldDemo />
    </Sequence>

    {/* Segment 5: Validation Rigor & Profiling (1:40 - 2:05 / 750 frames) */}
    <Sequence {...TIMING.segment5_validationRigor}>
      <ValidationRigor />
    </Sequence>

    {/* Segment 6: Honest Limitations & Mitigation (2:05 - 2:18 / 390 frames) */}
    <Sequence {...TIMING.segment6_honestLimitations}>
      <HonestLimitations />
    </Sequence>

    {/* Segment 7: Close & Team Lockup (2:18 - 2:36 / 540 frames) */}
    <Sequence {...TIMING.segment7_closingBranding}>
      <ClosingBranding />
    </Sequence>
  </AbsoluteFill>
);
