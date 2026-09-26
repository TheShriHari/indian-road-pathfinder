import React from "react";
import { AbsoluteFill, Sequence } from "remotion";
import { TIMING, COLORS } from "./theme";
import { FluidCameraRig } from "./components/FluidCameraRig";
import { TitleCard } from "./scenes/segment1/TitleCard";
import { ProblemStatement } from "./scenes/segment1/ProblemStatement";
import { PerceptionScan } from "./scenes/segment2/PerceptionScan";
import { MotionPrediction } from "./scenes/segment2/MotionPrediction";
import { CostmapBuild } from "./scenes/segment2/CostmapBuild";
import { CorridorCheck } from "./scenes/segment2/CorridorCheck";
import { PathPlanning } from "./scenes/segment2/PathPlanning";
import { BehaviorFSM } from "./scenes/segment2/BehaviorFSM";
import { SteeringControl } from "./scenes/segment2/SteeringControl";
import { BugsFoundFixed } from "./scenes/segment3/BugsFoundFixed";
import { RigorKPIs } from "./scenes/segment3/RigorKPIs";
import { ClosingBranding } from "./scenes/segment4/ClosingBranding";

export const PS26037Explainer: React.FC = () => (
  <AbsoluteFill style={{ backgroundColor: COLORS.bg }}>
    <FluidCameraRig>
      {/* Beat 1: Title Card (Camera pulls through into road) */}
      <Sequence {...TIMING.beat1_titleCard}>
        <TitleCard />
      </Sequence>

      {/* Beat 2: Problem Statement (Merged Reality + Assumptions -> dolly into roof sensor pod) */}
      <Sequence {...TIMING.beat2_problemStatement}>
        <ProblemStatement />
      </Sequence>

      {/* Beat 3: Perception Scan (Starts docked on roof sensor, bounding boxes remain) */}
      <Sequence {...TIMING.beat3_perceptionScan}>
        <PerceptionScan />
      </Sequence>

      {/* Beat 4: Motion Prediction (Covariance ribbons sprout, camera pulls to bird's-eye) */}
      <Sequence {...TIMING.beat4_motionPrediction}>
        <MotionPrediction />
      </Sequence>

      {/* Beat 5: Costmap Build (Bird's-eye cellular heat bloom, pinch setup) */}
      <Sequence {...TIMING.beat5_costmapBuild}>
        <CostmapBuild />
      </Sequence>

      {/* Beat 6: Corridor Check (Spring calipers, stop line wipe, particle shatter) */}
      <Sequence {...TIMING.beat6_corridorCheck}>
        <CorridorCheck />
      </Sequence>

      {/* Beat 7: Path Planning (Particles coalesce, spline draw, turning radius) */}
      <Sequence {...TIMING.beat7_pathPlanning}>
        <PathPlanning />
      </Sequence>

      {/* Beat 8: Behavior FSM (Vehicle tracks spline, speed-driven chassis physics) */}
      <Sequence {...TIMING.beat8_behaviorFSM}>
        <BehaviorFSM />
      </Sequence>

      {/* Beat 9: Steering Control (Cockpit gauge synced to wheel turn, max 25°/s) */}
      <Sequence {...TIMING.beat9_steeringControl}>
        <SteeringControl />
      </Sequence>

      {/* Beat 10: Bugs Found & Fixed (Diagnostic zoom, corner cutting + EKF fixes) */}
      <Sequence {...TIMING.beat10_bugsFoundFixed}>
        <BugsFoundFixed />
      </Sequence>

      {/* Beat 11: Rigor KPIs (Pull-back, 1,000 trials, zero toolboxes, 2.3ms replan) */}
      <Sequence {...TIMING.beat11_rigorKpis}>
        <RigorKPIs />
      </Sequence>

      {/* Beat 12: Closing Branding (Ego forward acceleration, real gaps remain) */}
      <Sequence {...TIMING.beat12_closingBranding}>
        <ClosingBranding />
      </Sequence>
    </FluidCameraRig>
  </AbsoluteFill>
);
