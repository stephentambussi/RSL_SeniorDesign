import { Card } from "@mui/material";
import { useEffect } from "react";
import { useSelector } from "react-redux";
import { useState } from "react";
import AutoNav from "./AutoNav";
import ManualControl from "./ManualControl";

export default function ControlArea({ displayedComponentType }) {
  const displayedComponentDict = {
    "autoNav": <AutoNav />,
    "manualControl": <ManualControl />,
  };
  const controlAreaPaneType = "controlAreaPane";
  const [displayedComponent, setDisplayedComponet] = useState(
    displayedComponentDict[controlAreaPaneType]
  );
  const [isAutoNavActive, setIsAutoNavActive] = useState(true);
  const [isManualControlActive, setIsManualControlActive] = useState(false);

  const changeDisplayedComponent = (displayedComponentType) => {
      debugger
    setDisplayedComponet(displayedComponentDict[displayedComponentType]);
    switch (displayedComponentType) {
      case "autoNav":
        setIsAutoNavActive(true);
        setIsManualControlActive(false);
        break;
      case "manualControl":
        setIsAutoNavActive(false);
        setIsManualControlActive(true);
        break;
      default:
        break;
    }
  };

  useEffect(() => {
    console.log(displayedComponentType);
    changeDisplayedComponent(displayedComponentType);
  }, [displayedComponentType]);

  return (
    <Card>
      {displayedComponent}
      {isAutoNavActive === true ? <AutoNav /> : null}
      {isManualControlActive === true ? <ManualControl /> : null}
    </Card>
  );
}
