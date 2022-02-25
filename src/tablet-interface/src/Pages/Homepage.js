import React from "react";
import { useState } from "react";
import { Box } from "@mui/system";
import NavBar from "../Components/layout/NavBar";
import ModeSelect from "../Components/ModeSelect";
import ControlArea from "../Components/ControlArea";
import AutoNav from "../Components/AutoNav";

export default function Homepage() {
  const [displayedComponent, setDisplayedComponent] = useState("mainNav");

  const handleSelectionClick = (selection) => {
    console.log("Mode selection clicked: " + selection);
    switch (selection) {
      case "autoNav":
        setDisplayedComponent("autoNav");
        break;
      case "manualControl":
        setDisplayedComponent("manualControl");
        break;
    }
  };

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "Column",
        alignItems: "center",

      }}
    >
      <NavBar/>
      <Box
        sx={{
          display: "flex",
          flexDirection: "row",
          columnGap: "355px",
            marginTop: "100px",

        }}
      >
        <Box
          sx={{
            display: "flex",
            width: "50%",
            flexDirection: "column",
            justifyItems: "center",
          }}
        >
          <ModeSelect modeSelectionCallback={handleSelectionClick}/>
        </Box>
        <Box
          sx={{
            display: "flex",
            width: "50%",
            justifyItems: "center",
          }}
        >
          <ControlArea displayedComponentType={displayedComponent}/>
        </Box>
      </Box>
    </Box>
  );
}
