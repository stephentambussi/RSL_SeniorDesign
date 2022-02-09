import { Card } from '@mui/material'

export default function ControlArea() {
const [isAutoNavActive, setIsAutoNavActive] = useState(false);
  const [isManualControlActive, setIsManualControlActive] = useState(false);

  const changeDisplayedComponent = (component) => {
    setDisplayedComponet(displayedComponentDict[component])
    switch(component){
      case "autoNav":
        setIsAutoNavActive(true);
        setIsManualControlActive(false);
        break;
      case "manualControl":
        setIsAutoNavActive(false);
        setIsManualControlActive(true);
        break;
      default:
        break
    }
  }

  const displayedComponentDict ={
    "autoNav":<AutoNav/>,
    "manualControl":<ManualControl/>
  }

  const [displayedComponent, setDisplayedComponet] = usedState(displayedComponentDict.autoNav)


}