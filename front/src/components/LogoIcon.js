import React from 'react';
import { useNavigate } from 'react-router-dom';
import logoImage from '../assets/image (18).png'; // 로고 이미지 파일 경로

const LogoIcon = () => {
  const navigate = useNavigate();

  const goToHome = () => {
    navigate('/'); // 홈 화면으로 이동
  };

  return (
    <img 
      src={logoImage} 
      alt="Logo"
      onClick={goToHome}
      style={{
        position: 'fixed', // 고정 위치
        top: '20px', // 화면 상단에서 20px 떨어짐
        left: '20px', // 화면 우측에서 20px 떨어짐
        width: '120px', // 로고 너비
        height: 'auto', // 비율 유지
        zIndex: 1000, // 다른 요소 위에 표시되도록 설정
        cursor: "pointer",
      }}
    />
  );
};

export default LogoIcon;
