import React from "react";
import { Box } from "@mui/system";
import AutoNav from "../Components/AutoNav";
import ModeSelect from "../Components/ModeSelect";

export default function Homepage() {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "Row",
      }}
    >
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
        <AutoNav />
      </Box>
    </Box>
  );
}
