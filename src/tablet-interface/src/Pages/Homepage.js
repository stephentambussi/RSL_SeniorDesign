import React from "react";
import { useState } from "react";
import { Box } from "@mui/system";
import MainNav from "../Components/layout/MainNav";
import ModeSelect from "../Components/ModeSelect";
import ControlArea from "../Components/ControlArea";

export default function Homepage() {

  const [displayedComponent, setDisplayedComponent] = useState("mainNav");

  const handleSelectionClick = (selection) => {
    console.log("Mode selection clicked: " + selection)
    switch(selection) {
      case "autoNav":
        setDisplayedComponent("autoNav");
        break;
      case "manualControl":
        setDisplayedComponent("manualControl");
        break;
    }
  }
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "Column",
        alignItems: "center",
        
      }}
    >
      <MainNav />
      <Box
        sx={{
            display: "flex",
            flexDirection: "row"
        }}>
        <Box
          sx={{
            display: "flex",
            width: "50%",
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
