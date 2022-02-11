// import { Route, Routes } from "react-router-dom";
import { Box } from "@mui/system";
import Homepage from "./Pages/Homepage";
import { Route, Router } from "react-router-dom";

function App() {
  return (
    <Router>
      <div>
        <Route path="/" component={Homepage} />
      </div>
    </Router>
  );
}

export default App;
