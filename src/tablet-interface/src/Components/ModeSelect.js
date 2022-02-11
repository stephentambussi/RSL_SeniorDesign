import React from "react";
import { Container, Button } from "@mui/material";
import { publishTwishMessage } from "../Helpers/ROSLink";



export default function ModeSelect({modeSelectionCallback}) {

  const handleSelectionClick = (selection) => {
    modeSelectionCallback(selection)
  }


  return (
    <Container>
      <header>Mode selector</header>
      <Button onClick={handleSelectionClick("manualControl")}>Manual mode</Button>
      <Button>Follow mode</Button>
      <Button onClick={handleSelectionClick("autonav")}>AutoNav</Button>
    </Container>
  );
}
