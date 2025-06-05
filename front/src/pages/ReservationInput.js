import React, { useState, useCallback, useEffect, useRef } from "react";
import { useNavigate } from "react-router-dom";
import { useTranslation } from "react-i18next";
import VoiceButton from '../components/VoiceButton';
import { Client } from '@stomp/stompjs';
import SockJS from 'sockjs-client';
import api from '../components/api';

const ReservationInput = () => {
  const navigate = useNavigate();
  const [reservationNumber, setReservationNumber] = useState("");
  const { t } = useTranslation();
  const [stompClient, setStompClient] = useState(null);

  // 페이지 렌더링 시 상태 초기화
  useEffect(() => {
    console.log('[ReservationInput] Component mounted, resetting reservationNumber');
    setReservationNumber(""); // 상태 초기화
  }, []);

  // WebSocket 연결 초기화
  useEffect(() => {
    console.log('[WebSocket] 연결 시작...');
    const socket = new SockJS('https://j12e104.p.ssafy.io/ws');
    const client = new Client({
      webSocketFactory: () => socket,
      reconnectDelay: 5000,
      debug: (str) => console.log('[STOMP]', str),
    });

    client.activate();
    setStompClient(client);

    return () => {
      console.log('[WebSocket] 연결 해제...');
      if (client.active) {
        client.deactivate();
      }
    };
  }, []);

  // 키 입력 처리
  const handleKeyPress = (key) => {
    if (reservationNumber.length < 6 && /^[A-Z0-9]$/.test(key)) {
      setReservationNumber(prev => prev + key);
    }
  };

  // 전체 삭제
  const handleAllDelete = () => setReservationNumber("");

  // 마지막 삭제
  const handleDelete = () => setReservationNumber(prev => prev.slice(0, -1));

  // 취소
  const handleCancel = () => {
    navigate("/");
    setReservationNumber("");
  };

  // API 호출 함수
  const fetchReservationData = async (code) => {
    try {
      console.log('[API] 예약 검증 요청:', code);
      const response = await api.post('/reservations/verify', { 
        reservationCode: code 
      });
      console.log('[API] 응답 데이터:', response.data);
      return response.data;
    } catch (error) {
      console.error('[API] 오류:', error);
      if (error.response?.status === 500) {
        alert(t('reservationInput.notFound'));
      } else {
        alert(t('reservationInput.serverError'));
      }
      return null;
    }
  };

  // 확인 버튼 핸들러
  const handleConfirm = async () => {
    if (reservationNumber.length !== 6) {
      alert(t('reservationInput.invalidLength'));
      return;
    }
  
    // 1. API 검증
    const apiData = await fetchReservationData(reservationNumber);
    if (!apiData) return;
  
    // ✅ 추가된 시작 API 호출 부분
    try {
      await api.post('/reservations/start', {
        reservationCode: reservationNumber
      });
      console.log('[API] 로봇 출발 요청 성공');
    } catch (error) {
      console.error('[API] 로봇 출발 오류:', error);
      alert(t('reservationInput.serverError'));
      return;
    }
  
    // 2. WebSocket 전송
    if (stompClient?.connected) {
      console.log('[WebSocket] 예약 코드 전송:', reservationNumber);
      stompClient.publish({
        destination: '/app/verifyReservation',
        body: JSON.stringify({
          type: "verifyReservation",
          reservationCode: reservationNumber
        })
      });
    }
  
    // 3. 페이지 이동
    navigate(`/move-to-gate/${reservationNumber}`, {
      state: { 
        gateNumber: apiData.gateNumber,
        initialData: apiData 
      }
    });
    setReservationNumber("");
  };

  // 음성 입력 처리
  const handleVoiceInput = useCallback((input) => {
    const formattedInput = input
      .toUpperCase()
      .replace(/[^A-Z0-9]/g, '')
      .slice(0, 6);
    console.log('[음성입력] 변환 결과:', formattedInput);
    setReservationNumber(formattedInput);
  }, []);

  // 키보드 이벤트 핸들러
  useEffect(() => {
    const handleKeyDown = (e) => {
      const key = e.key.toUpperCase();
      if (/^[A-Z0-9]$/.test(key)) handleKeyPress(key);
      if (key === "BACKSPACE") handleDelete();
      if (key === "ENTER") handleConfirm();
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, [handleKeyPress, handleDelete, handleConfirm]);

  // 키보드 이벤트 핸들러
  useEffect(() => {
    const handleKeyDown = (e) => {
      const key = e.key.toUpperCase();
      if (/^[A-Z0-9]$/.test(key)) handleKeyPress(key);
      if (key === "BACKSPACE") handleDelete();
      if (key === "ENTER") handleConfirm();
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, [handleKeyPress, handleDelete, handleConfirm]);

  // 반응형 스타일 객체
  const responsiveStyles = {
    container: {
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      justifyContent: "center", // 중앙 정렬
      height: "100vh", // 화면 전체 높이
      backgroundColor: "#f5f5f5",
      padding: "clamp(3vw, 4vw, 30px)", // 반응형 패딩
      boxSizing: "border-box", // 패딩 포함 높이 계산
      overflow: "hidden", // 스크롤 방지
    },
    title: {
      fontSize: "clamp(24px, 4vw, 42px)",
      margin: "clamp(10px, 2vw, 20px) 0",
      color: "rgb(67, 67, 67)",
      textAlign: "center"
    },
    codeBox: (index) => ({
      width: "clamp(40px, 8vw, 55px)",
      height: "clamp(40px, 8vw, 55px)",
      borderRadius: "8px",
      border: index < reservationNumber.length 
        ? "2px solid rgb(166, 215, 255)" 
        : "2px solid rgb(166, 215, 255)",
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      backgroundColor: "#FFFFFF",
      fontSize: "clamp(16px, 3vw, 23px)",
      fontWeight: "bold",
      position: "relative"
    }),
    keyboardGrid: {
      display: "grid",
      gridTemplateColumns: "repeat(10, 1fr)",
      gap: "clamp(5px, 1vw, 10px)",
      width: "90%",
      maxWidth: "800px",
      margin: "clamp(10px, 2vh, 20px) 0"
    },
    keyButton: {
      padding: "clamp(8px, 1.5vw, 15px)",
      fontSize: "clamp(14px, 2.5vw, 24px)",
      borderRadius: "8px",
      border: "2px solid #ffffff",
      backgroundColor: "#ffffff",
      color: "#555",
      fontWeight: "bold",
      cursor: reservationNumber.length < 6 ? "pointer" : "not-allowed",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)",
      transition: "background-color 0.2s"
    },
    controlButton: (isActive) => ({
      padding: "clamp(10px, 2vw, 20px)",
      fontSize: "clamp(14px, 2vw, 20px)",
      borderRadius: "8px",
      border: "none",
      backgroundColor: isActive ? "rgb(111, 168, 248)" : "#E0E0E0",
      color: isActive ? "#FFFFFF" : "#9E9E9E",
      fontWeight: "bold",
      cursor: isActive ? "pointer" : "not-allowed",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)",
      transition: "all 0.2s"
    }),
    deleteButton: {
      padding: "clamp(10px, 2vw, 20px)",
      fontSize: "clamp(14px, 2vw, 20px)",
      borderRadius: "8px",
      border: "none",
      backgroundColor: "rgb(255, 223, 223)",
      color: "#555",
      fontWeight: "bold",
      cursor: "pointer",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)"
    }
  };

  return (
    <div style={responsiveStyles.container}>
      <h2 style={responsiveStyles.title}>{t('reservationInput.title')}</h2>
      
      <div style={{ display: "flex", gap: "clamp(5px, 1vw, 10px)", marginBottom: "2vh" }}>
        {[...Array(6)].map((_, index) => (
          <div
            key={index}
            style={responsiveStyles.codeBox(index)}
          >
            {reservationNumber[index] || ""}
          </div>
        ))}
        <VoiceButton onVoiceInput={handleVoiceInput} />
      </div>

      <div style={responsiveStyles.keyboardGrid}>
        {"1234567890QWERTYUIOPASDFGHJKLZXCVBNM".split("").map((key) => (
          <button
            key={key}
            onClick={() => handleKeyPress(key)}
            onMouseDown={(e) => e.target.style.backgroundColor = "#BBDEFB"}
            onMouseUp={(e) => e.target.style.backgroundColor = "#ffffff"}
            style={responsiveStyles.keyButton}
            disabled={reservationNumber.length >= 6}
          >
            {key}
          </button>
        ))}
      </div>

      <div style={{ 
        display: "flex", 
        justifyContent: "center", 
        gap: "clamp(10px, 2vw, 20px)", 
        margin: "clamp(10px, 2vh, 20px) 0"
      }}>
        <button
          onClick={handleDelete}
          style={{
            ...responsiveStyles.deleteButton,
            backgroundColor: reservationNumber ? "rgb(255, 223, 223)" : "#E0E0E0",
            cursor: reservationNumber ? 'pointer' : 'not-allowed'
          }}
          disabled={!reservationNumber}
        >
          {t('reservationInput.delete')}
        </button>

        <button
          onClick={handleAllDelete}
          style={{
            ...responsiveStyles.deleteButton,
            backgroundColor: reservationNumber ? "rgb(255, 223, 223)" : "#E0E0E0",
            cursor: reservationNumber ? 'pointer' : 'not-allowed'
          }}
          disabled={!reservationNumber}
        >
          {t('reservationInput.deleteAll')}
        </button>
      </div>

      <div style={{ display: "flex", gap: "clamp(10px, 2vw, 20px)" }}>
        <button
          onClick={handleCancel}
          style={{
            ...responsiveStyles.controlButton(true),
            backgroundColor: "rgb(255, 209, 209)",
            color: "#555"
          }}
        >
          {t('reservationInput.cancel')}
        </button>

        <button
          onClick={handleConfirm}
          style={responsiveStyles.controlButton(reservationNumber.length === 6)}
          disabled={reservationNumber.length !== 6}
        >
          {t('reservationInput.confirm')}
        </button>
      </div>
    </div>
  );
};

export default ReservationInput;