import React from "react";
import { Container, Button } from "@mui/material";



export default function ModeSelect() {



const handleOnClick = () => {
  console.log("Publishing cmd_vel")
}




  return (
    <Container>
      <header>Mode selector</header>
      <Button onClick={handleOnClick}>Manual mode</Button>
      <Button>Follow mode</Button>
      <Button>AutoNav</Button>
    </Container>
  );
}
