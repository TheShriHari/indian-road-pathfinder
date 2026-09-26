import React from "react";
import { AbsoluteFill, Sequence } from "remotion";
import { TIMING } from "./theme";
import { RealityIntro } from "./scenes/segment1/RealityIntro";
import { AssumptionBreakdown } from "./scenes/segment1/AssumptionBreakdown";
import { TitleCard } from "./scenes/segment1/TitleCard";
import { PerceptionScan } from "./scenes/segment2/PerceptionScan";
import { MotionPrediction } from "./scenes/segment2/MotionPrediction";
import { CostmapBuild } from "./scenes/segment2/CostmapBuild";
import { CorridorCheck } from "./scenes/segment2/CorridorCheck";
import { PathPlanning } from "./scenes/segment2/PathPlanning";
import { BehaviorFSM } from "./scenes/segment2/BehaviorFSM";
import { SteeringControl } from "./scenes/segment2/SteeringControl";
import { ScenarioGrid } from "./scenes/segment3/ScenarioGrid";
import { RigorKPIs } from "./scenes/segment3/RigorKPIs";
import { ClosingBranding } from "./scenes/segment4/ClosingBranding";

export const PS26037Explainer: React.FC = () => (
  <AbsoluteFill style={{ backgroundColor: "#0B0F19" }}>
    {/* ── SEGMENT 1: Introduction ── */}
    <Sequence {...TIMING.scene1_1_realityIntro}>
      <RealityIntro />
    </Sequence>
    <Sequence {...TIMING.scene1_2_assumptionBreakdown}>
      <AssumptionBreakdown />
    </Sequence>
    <Sequence {...TIMING.scene1_3_titleCard}>
      <TitleCard />
    </Sequence>

    {/* ── SEGMENT 2: 7-Step Workflow ── */}
    <Sequence {...TIMING.step1_perceptionScan}>
      <PerceptionScan />
    </Sequence>
    <Sequence {...TIMING.step2_motionPrediction}>
      <MotionPrediction />
    </Sequence>
    <Sequence {...TIMING.step3_costmapBuild}>
      <CostmapBuild />
    </Sequence>
    <Sequence {...TIMING.step4_corridorCheck}>
      <CorridorCheck />
    </Sequence>
    <Sequence {...TIMING.step5_pathPlanning}>
      <PathPlanning />
    </Sequence>
    <Sequence {...TIMING.step6_behaviorFSM}>
      <BehaviorFSM />
    </Sequence>
    <Sequence {...TIMING.step7_steeringControl}>
      <SteeringControl />
    </Sequence>

    {/* ── SEGMENT 3: Engineering Rigor ── */}
    <Sequence {...TIMING.scene3_1_scenarioGrid}>
      <ScenarioGrid />
    </Sequence>
    <Sequence {...TIMING.scene3_2_rigorKpis}>
      <RigorKPIs />
    </Sequence>

    {/* ── SEGMENT 4: Closing ── */}
    <Sequence {...TIMING.scene4_1_closingBranding}>
      <ClosingBranding />
    </Sequence>
  </AbsoluteFill>
);
