import { Container } from "@mui/material";
import React from "react";
import { Button } from "@mui/material";

export default function AutoNav() {
  return (
    <Container>
      <header>Destination select</header>
      <Button>Home</Button>
      <Button>Kitchen</Button>
    </Container>
  );
}
