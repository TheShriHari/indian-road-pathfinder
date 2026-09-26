import "./index.css";
import React from "react";
import { Composition } from "remotion";
import { PS26037Explainer } from "./PS26037Explainer";
import { WIDTH, HEIGHT, FPS, TOTAL_FRAMES } from "./theme";

export const RemotionRoot: React.FC = () => {
  return (
    <>
      <Composition
        id="PS26037Explainer"
        component={PS26037Explainer}
        durationInFrames={TOTAL_FRAMES}
        fps={FPS}
        width={WIDTH}
        height={HEIGHT}
      />
    </>
  );
};
