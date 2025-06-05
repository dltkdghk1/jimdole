import React, { useState, useEffect } from "react";

const CurrentDateTime = () => {
  const [currentTime, setCurrentTime] = useState(new Date()); // 현재 시간 상태

  // 1초마다 시간을 업데이트
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => clearInterval(timer); // 컴포넌트 언마운트 시 타이머 정리
  }, []);

  return (
    <div
      style={{
        position: "fixed", // 고정 위치
        top: "20px",
        right: "20px",
        fontSize: "30px",
        color: "#555",
        zIndex: 1000, // 다른 요소 위에 표시되도록 설정
      }}
    >
      {currentTime.toLocaleDateString()} (
      {["일", "월", "화", "수", "목", "금", "토"][currentTime.getDay()]})<br />
      {currentTime.toLocaleTimeString()} {/* 실시간 시간 표시 */}
    </div>
  );
};

export default CurrentDateTime;
