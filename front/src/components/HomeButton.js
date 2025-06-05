import React from "react";
import { useNavigate } from "react-router-dom";
import { useTranslation } from "react-i18next";
import HomeIcon from "../assets/building.png"

const HomeButton = ({ setReservationNumber }) => {
  const navigate = useNavigate();
  const { t } = useTranslation();
  const goToHome = () => {
    navigate("/"); // Home.js로 이동
    setReservationNumber(""); // 예약번호 초기화
  };

  return (
    <div
      onClick={goToHome}
      style={{
        position: "fixed",
        top: "10px",
        right: "280px",
        zIndex: 1000, // 다른 요소 위에 표시
        fontSize: "40px",
        cursor: "pointer",
      }}
    >
        <img 
          src={HomeIcon} 
          alt="Home"
          style={{ width: "80px", height: "auto" }} // 아이콘 크기 조정
        />
    </div>
  );
};

export default HomeButton;
