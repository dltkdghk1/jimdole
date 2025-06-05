import React from "react";
import styled, { keyframes } from "styled-components";
import robotIcon from "../assets/robot-assistant.png";
import boxIcon from "../assets/box.png";

// 로봇과 박스가 함께 움직이는 애니메이션
const moveToGate = keyframes`
  0% { transform: translateX(20px); }
  50% { transform: translateX(400px); }
  100% { transform: translateX(20px); }
`;

// 로봇과 박스를 묶은 컨테이너
const RobotWithBoxContainer = styled.div`
  display: flex;
  align-items: center;
  animation: ${moveToGate} 6s infinite;
`;

// 로봇 아이콘 스타일
const RobotIcon = styled.img`
  width: 100px;
  height: 100px;
`;

// 박스 아이콘 스타일
const BoxIcon = styled.img`
  width: 60px;
  height: 60px;
  margin-left: -20px;
`;

const MovingAnimation = () => {
  return (
    <RobotWithBoxContainer>
      <RobotIcon src={robotIcon} alt="Robot Icon" />
      <BoxIcon src={boxIcon} alt="Box Icon" />
    </RobotWithBoxContainer>
  );
};

export default MovingAnimation;
