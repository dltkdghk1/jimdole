import React, { useState } from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import Home from "./pages/Home";
import ReservationInput from "./pages/ReservationInput";
import AdminLogin from "./pages/AdminLogin";
import MoveToGate from "./pages/MoveToGate";
import RobotIcon from "./components/RobotIcon"; // 하단 로봇 아이콘
import CurrentDateTime from "./components/CurrentDateTime"; // 상단 날짜 및 시간
import HomeButton from "./components/HomeButton"; // 좌측 상단 초기화면 버튼
import LanguageSwitch from "./components/LanguageSwitch"; // 우측 하단 언어 변경 버튼
import Admin from "./pages/Admin"; // 관리자 페이지
import Chatbot from "./components/ChatBot";
import ChatbotIcon from "./components/ChatbotIcon";
import LogoIcon from "./components/LogoIcon"; // 로고
import './index.css';
import './App.css';

function App() {
  console.log('[App] Component rendered'); // App 컴포넌트 렌더링 확인
  const [reservationNumber, setReservationNumber] = useState("");
  const [language, setLanguage] = useState("ko"); // 전체 애플리케이션의 언어 상태

  const handleLanguageChange = (newLanguage) => {
    setLanguage(newLanguage); // 언어 상태 변경
  };

  const [showChatbot, setShowChatbot] = useState(false);
  const toggleChatbot = () => {
    setShowChatbot(!showChatbot);
  };

  return (
    <Router>
      <div style={{ position: "relative" }}>
        {/* 좌측 상단 로고 */}
        <LogoIcon />

        {/* 상단 현재 날짜 및 시간 */}
        <CurrentDateTime />

        {/* 좌측 상단 초기화면 버튼 */}
        <HomeButton setReservationNumber={setReservationNumber} />

        {/* 우측 하단 언어 변경 버튼 */}
        <LanguageSwitch onLanguageChange={handleLanguageChange} />

        {/* 페이지 라우팅 */}
        <Routes>
          <Route path="/" element={<Home language={language} />} /> {/* 초기 화면 */}
          <Route path="/reservation"
            element={<ReservationInput 
            reservationNumber={reservationNumber}
            setReservationNumber={setReservationNumber}
            language={language} />} /> {/* 예약번호 입력 화면 */}

          <Route path="/admin-login" element={<AdminLogin language={language} />} /> {/* 관리자 비밀번호 입력 화면 */}
          <Route path="/move-to-gate" element={<MoveToGate language={language} />} /> {/* 이동 안내 페이지 */}
          <Route path="/move-to-gate/:reservationNumber" element={<MoveToGate/>} /> {/* 게이트 이동 페이지 */}
          <Route path="/admin" element={<Admin/>} /> {/* 관리자 페이지 */}
        </Routes>
        
        {/* 챗봇 */}
        <ChatbotIcon onClick={toggleChatbot} />
        {showChatbot && <Chatbot onClose={() => setShowChatbot(false)} />}

        {/* 하단 로봇 아이콘 */}
        <RobotIcon />
      </div>
    </Router>
  );
}

export default App;
