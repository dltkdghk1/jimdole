import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { useNavigate } from "react-router-dom";
import { useSpeechSynthesis } from 'react-speech-kit';
import gateIcon from '../assets/boarding-gate.png';
import MovingAnimation from "../components/MovingAnimation";

const Home = () => {
  const navigate = useNavigate();
  const { t } = useTranslation();
  const { speak, cancel, speaking } = useSpeechSynthesis();
  const [isSpeaking, setIsSpeaking] = useState(false);

  const playWelcomeMessage = () => {
    const welcomeMessage = t("home.welcome") + " " + t("home.description");
    speak({
      text: welcomeMessage,
      onEnd: () => {
        setIsSpeaking(false);
      },
    });
    setIsSpeaking(true);
  };

  useEffect(() => {
    playWelcomeMessage();

    const intervalId = setInterval(() => {
      if (!isSpeaking) {
        playWelcomeMessage();
      }
    }, 30000);

    return () => {
      clearInterval(intervalId);
      cancel();
    };
  }, [isSpeaking, cancel]);

  const handleClick = () => {
    navigate("/reservation");
  };

  return (
    <div 
      onClick={handleClick}
      style={{
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        alignItems: "center",
        height: "100vh",
        backgroundColor: "#f5f5f5",
        cursor: "pointer",
      }}
    >
      {/* 제목 섹션 */}
      <div style={{ textAlign: "center", marginBottom: "20px", color: "rgb(67, 67, 67):" }}>
        <h1 style={{ fontSize: "350%", marginBottom: "10px" }}>
          {t("home.welcome")}
        </h1>
        <h1 style={{ fontSize: "350%" }}>
          {t("home.description")}
        </h1>
      </div>

      {/* 애니메이션 & 게이트 섹션 */}
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          justifyContent: "space-between",
          alignItems: "center",
          width: "80%",
          maxWidth: "600px",
          gap: "20px",
          marginTop: "40px"
        }}
      >
        <MovingAnimation />
        <img
          src={gateIcon}
          alt="Gate Icon"
          style={{ width: '100px', height: '100px' }}
        />
      </div>
    </div>
  );
};

export default Home;
