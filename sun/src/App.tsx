import sunImage from "./assets/sun.png";
import starsImage from "./assets/stars.jpg";
import oceanImage from "./assets/ocean.png";
import { useEffect, useState } from "react";
import { Slider } from "@mui/material";
import styled from "@emotion/styled";
import io from "socket.io-client";

export default function App() {
  const [position, setPosition] = useState(sunPosition(0));
  const [time, setTime] = useState(0);
  const socket = io("http://localhost:8888");

  useEffect(() => {
    socket.on("serial", ({ data }) => {
      console.log("Saida serial arduino: ", data);
      const parsedNumber = Number(data.replace("\r", ""));
      setTime(parsedNumber);
      setPosition(sunPosition(parsedNumber));
    });
  }, []);

  function sunPosition(time: number): { x: number; y: number } {
    const maxWidth = window.innerWidth + 500;
    const width = (maxWidth / 24) * time;
    const maxHeight = window.innerHeight;

    return {
      x: width - 450,
      y:
        (((2 * Math.sqrt(maxHeight)) / maxWidth) * width -
          Math.sqrt(maxHeight)) **
        2,
    };
  }

  function sliderHandler(value: number) {
    setTime(value);
    setPosition(sunPosition(value));
  }

  function opacityMap(): number {
    if (time < 5) {
      return 1;
    } else if (time >= 5 && time < 7) {
      return 1 - (time - 5) / 2;
    } else if (time >= 7 && time < 17) {
      return 0;
    } else if (time >= 17 && time < 19) {
      return (time - 17) / 2;
    }

    return 1;
  }

  return (
    <Container>
      <Hours>{time} horas</Hours>
      <Slider
        sx={{ zIndex: "3", width: "500px" }}
        min={0}
        max={24}
        step={0.1}
        value={time}
        onChange={(_, value) => sliderHandler(value as number)}
      />
      <Stars
        src={starsImage}
        style={{
          opacity: opacityMap(),
        }}
      />
      <Sun
        src={sunImage}
        style={{ top: position.y + "px", left: position.x + "px" }}
      />
      <SunShadow
        style={{
          top: position.y + 100 + "px",
          left: position.x + 177.777 + "px",
        }}
      />
      <Ocean src={oceanImage} />
    </Container>
  );
}

const Container = styled.div`
  position: relative;
  height: 100vh;
  padding: 16px;
  background-color: #c3bbe8;
`;

const Hours = styled.p`
  position: absolute;
  top: 16px;
  right: 16px;
  color: #fff;
  z-index: 1;
`;

const Stars = styled.img`
  position: absolute;
  top: 0;
  left: 0;
  width: 100vw;
  z-index: 0;
`;

const Sun = styled.img`
  position: absolute;
  height: 200px;
  z-index: 1;
`;

const SunShadow = styled.div`
  position: absolute;
  box-shadow: 0px 0px 300px 200px #ffde8b;
  z-index: 0;
`;

const Ocean = styled.img`
  position: absolute;
  z-index: 2;
  left: -32px;
  width: calc(100vw + 64px);
  top: -75vh;
`;
