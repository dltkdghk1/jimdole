import React from "react";
import { useNavigate } from "react-router-dom";
import robotIcon from "../assets/user-setting (1).png"

const RobotIcon = () => {
  const navigate = useNavigate();

  // 로봇 아이콘 클릭 시 관리자 비밀번호 입력 페이지로 이동
  const handleClick = () => {
    navigate("/admin-login"); // '/admin-login' 경로로 이동
  };

  return (
    <div
      style={{
        position: "fixed", // 모든 페이지에서 고정 위치
        bottom: "20px",
        left: "20px",
        cursor: "pointer",
      }}
      onClick={handleClick}
    >
      <img
        src={robotIcon}
        alt="Robot Icon"
        style={{ width: "70px", height: "auto" }}
      />
    </div>
  );
};

export default RobotIcon;
