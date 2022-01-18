import React from "react";
import { Container, Button } from "@mui/material";

export default function ModeSelect() {
  return (
    <Container>
      <header>Mode selector</header>
      <Button>Manual mode</Button>
      <Button>Follow mode</Button>
    </Container>
  );
}
