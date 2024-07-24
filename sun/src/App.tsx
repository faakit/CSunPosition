import sunImage from "./assets/sun.png";
import starsImage from "./assets/stars.jpg";
import { useState } from "react";
import { Slider } from "@mui/material";
import styled from "@emotion/styled";
import io from 'socket.io-client';

export default function App() {
  const [position, setPosition] = useState(sunPosition(0));
  const [time, setTime] = useState(0);
  const socket = io('http://localhost:8888');

  socket.on('serial', (data) => {
    console.log('parser: ', data)
    setTime(Number(data))
  });

  function sunPosition(time: number): { x: number; y: number } {
    const maxWidth = window.innerWidth - 200;
    const width = (maxWidth / 24) * time;
    const maxHeight = window.innerHeight;

    return {
      x: width - 70,
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

  return (
    <Container>
      <Hours>{time} horas</Hours>
      <Slider
        sx={{ zIndex: "2", width: "500px" }}
        min={0}
        max={24}
        step={0.1}
        value={time}
        onChange={(_, value) => sliderHandler(value as number)}
      />
      <Stars
        src={starsImage}
        style={{
          opacity:
            time >= 0 && time <= 6
              ? 1 - time / 6
              : time >= 18 && time <= 25
              ? (time - 18) / 6
              : 0,
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
      ></SunShadow>
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
  box-shadow: 0px 0px 500px 300px #ffde8b;
  z-index: 0;
`;
