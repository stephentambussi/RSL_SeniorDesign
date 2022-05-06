import React from "react";
import { Container, Button, Card } from "@mui/material";
// import { publishTwishMessage } from "../Helpers/ROSLink";

export default function ModeSelect({ modeSelectionCallback }) {
  const handleSelectionClick = (selection) => {
    //   console.log("Mode selection clicked: " + selection);
    modeSelectionCallback(selection);
  };

  return (
    <Container>
      <Card>
        <header>Mode selector</header>
        <Button
          onClick={() => {
            handleSelectionClick("manualControl");
          }}
        >
          Manual mode
        </Button>
        <Button
          onClick={() => {
            handleSelectionClick("followMode");
          }}
        >
          Follow mode
        </Button>
        <Button
          onClick={() => {
            handleSelectionClick("autoNav");
          }}
        >
          AutoNav
        </Button>
      </Card>
    </Container>
  );
}
