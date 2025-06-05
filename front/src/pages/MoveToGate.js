import React, { useState, useEffect } from 'react';
import { useParams, useNavigate, useLocation } from 'react-router-dom';
import { useSpeechSynthesis } from 'react-speech-kit';
import useRobotWebSocket from '../hooks/useRobotWebSocket';
import MovingAnimation from "../components/MovingAnimation";
import { useTranslation } from 'react-i18next';

const MoveToGate = () => {
  const { t, i18n } = useTranslation();
  const { reservationNumber } = useParams();
  const navigate = useNavigate();
  const location = useLocation();
  const [gateInfo, setGateInfo] = useState(null);
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0 });
  const { speak } = useSpeechSynthesis();

  // 반응형 스타일 객체 수정
  const responsiveStyles = {
    container: {
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      justifyContent: "space-evenly", // 공간 분배 개선
      height: "100vh",
      backgroundColor: "#f5f5f5",
      padding: "clamp(3vw, 4vw, 30px)",
      boxSizing: "border-box",
      overflow: "hidden",
    },
    title: {
      fontSize: "clamp(36px, 6vw, 90px)", // 폰트 크기 조정
      marginBottom: "clamp(8px, 1vw, 14px)", // H1과 H2 사이 간격 축소
      color: "rgb(67, 67, 67)",
      textAlign: "center",
      width: "90%", // 너비 제한
    },
    subtitle: {
      fontSize: "clamp(20px, 4vw, 65px)", // 부제목 크기 조정
      marginTop: "0", // H2의 상단 여백 제거
      marginBottom: "clamp(8px, 1vw, 10px)", // H1과 H2 사이 간격 축소
      color: "rgb(100, 100, 100)",
      textAlign: "center",
    },
    contentWrapper: {
      display: "flex",
      flexDirection: "row",
      justifyContent: "space-between",
      alignItems: "center",
      width: "90%",
      maxWidth: "600px",
      marginTop: "clamp(20px, 4vh, 40px)",
    },
    gateImage: {
      width: "clamp(60px, 10vw, 100px)",
      height: "clamp(60px, 10vw, 100px)",
    },
    movingAnimationWrapper: {
      transform: `translate(${robotPosition.x}px, ${robotPosition.y}px)`,
      transition: 'transform 0.5s ease-out',
    },
  };

  // 게이트 정보 초기화
  useEffect(() => {
    console.log('[게이트] 위치 정보 초기화');
    if (!location.state?.gateNumber) {
      console.error('[오류] 유효하지 않은 접근');
      navigate('/', { replace: true });
      return;
    }
    setGateInfo({
      gateNumber: location.state.gateNumber,
      ...location.state.initialData,
    });
  }, [location, navigate]);

  // WebSocket 메시지 처리
  const handleRobotMessage = (message) => {
    console.log('[로봇] 메시지 수신:', message);
    if (message.reservationCode !== reservationNumber) return;

    setRobotPosition({
      x: message.currentX,
      y: message.currentY,
    });

    if (message.status === 'STOPPED') {
      navigate(`/`);
    }
  };

  useRobotWebSocket(handleRobotMessage);

  // 음성 안내
  const playMessage = (message) => {
    speak({
      text: message,
      lang: i18n.language === 'ko' ? 'ko-KR' : 'en-US',
    });
  };
  console.log('Current language:', i18n.language);
  console.log('Translation:', t('moveToGate.homeRedirect'));

  return (
    <div style={responsiveStyles.container}>
      <h1 style={responsiveStyles.title}>
        {gateInfo
          ? t('moveToGate.moving', { destination: `GATE ${gateInfo.gateNumber}` })
          : t('moveToGate.loading')}
      </h1>
      <h2 style={responsiveStyles.subtitle}>
        {t('moveToGate.homeRedirect')}
      </h2>

      <div style={responsiveStyles.contentWrapper}>
        {/* 로봇 애니메이션 */}
        <div style={responsiveStyles.movingAnimationWrapper}>
          <MovingAnimation />
        </div>

        {/* 게이트 이미지 */}
        <img
          src={require('../assets/boarding-gate.png')}
          alt="Gate"
          style={responsiveStyles.gateImage}
        />
      </div>
    </div>
  );
};

export default MoveToGate;
