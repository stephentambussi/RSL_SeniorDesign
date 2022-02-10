import React from "react";
import { useState } from "react";
import { Box } from "@mui/system";
import AutoNav from "../Components/AutoNav";
import MainNav from "../Components/layout/MainNav";
import ModeSelect from "../Components/ModeSelect";
import ManualControl from "../Components/ManualControl";

export default function Homepage() {


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
          <ModeSelect />
        </Box>
        <Box
          sx={{
            display: "flex",
            width: "50%",
            justifyItems: "center",
          }}
        >
          {displayedComponent}

        </Box>
      </Box>
    </Box>
  );
}
