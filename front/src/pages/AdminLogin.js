import React, { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import api from "../components/api"; // Axios 인스턴스

const AdminLogin = () => {
  const [password, setPassword] = useState("");
  const [errorMessage, setErrorMessage] = useState("");
  const navigate = useNavigate();

  const handleKeyPress = (key) => {
    if (password.length < 12 && /^[A-Z0-9]$/.test(key)) {
      setPassword((prevPassword) => prevPassword + key);
    }
  };

  const handleDelete = () => {
    setPassword((prevPassword) => prevPassword.slice(0, -1));
  };

  const handleAllDelete = () => {
    setPassword("");
  };

  const handleSubmit = async (e) => {
    if (e) e.preventDefault(); // e가 존재할 경우에만 preventDefault 호출
    try {
      console.log(`API 호출: /admin/verify`);
      const response = await api.post("/admin/verify", { password });
      console.log("API 응답:", response.data);

      if (response.data.code === "SU") { 
        navigate("/admin"); // 성공 시 관리자 페이지로 이동
      } else {
        setErrorMessage("비밀번호가 올바르지 않습니다.");
      }
    } catch (error) {
      console.error("API 오류:", error);
      setErrorMessage("서버 오류가 발생했습니다.");
    }
  };

  // 키보드 입력 이벤트 리스너 추가
  useEffect(() => {
    const handleKeyDown = (event) => {
      const key = event.key.toUpperCase(); // 키 값을 대문자로 변환

      if (/^[A-Z0-9]$/.test(key)) {
        handleKeyPress(key); // 알파벳 또는 숫자 입력 처리
      } else if (key === "BACKSPACE") {
        handleDelete(); // 백스페이스로 마지막 문자 삭제
      } else if (key === "ENTER") {
        handleSubmit(); // 엔터로 확인 버튼 동작 실행
      }
    };

    document.addEventListener("keydown", handleKeyDown); // 키보드 이벤트 리스너 등록

    return () => {
      document.removeEventListener("keydown", handleKeyDown); // 컴포넌트 언마운트 시 리스너 제거
    };
  }, [handleKeyPress, handleDelete]);


  // 반응형 스타일 객체
  const responsiveStyles = {
    container: {
      display: "flex",
      flexDirection: "column",
      justifyContent: "center",
      alignItems: "center",
      height: "100vh",
      backgroundColor: "#f5f5f5",
      padding: "clamp(20px, 4vw, 40px)", // 반응형 패딩
      boxSizing: "border-box",
    },
    title: {
      marginBottom: "clamp(20px, 2vh, 40px)",
      fontSize: "clamp(30px, 4vw, 50px)", // 반응형 폰트 크기
      color: "rgb(67, 67, 67)",
    },
    passwordDisplay: {
      padding: "clamp(10px, 3vw, 18px) 90px 15px",
      fontSize: "clamp(20px, 2.5vw, 30px)", // 반응형 폰트 크기
      marginBottom: "clamp(15px, 2vh, 30px)",
      borderRadius: "8px",
      textAlign: "center",
      minWidth: "clamp(200px, 50vw, 400px)", // 반응형 너비
      backgroundColor: "#fff",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)",
    },
    errorMessage: {
      color: "rgb(184, 0, 0)",
      marginBottom: "clamp(15px, 2vh, 30px)",
    },
    keyboardGrid: {
      display: "grid",
      gridTemplateColumns: "repeat(10, 1fr)",
      gap: "clamp(5px, 1vw, 10px)",
      width: "90%",
      maxWidth: "800px",
      margin: "clamp(10px, 2vh, 20px) 0"
    },
    keyButton: {
      padding: "clamp(10px, 2vw, 20px)",
      fontSize: "clamp(16px, 2.5vw, 24px)", // 반응형 폰트 크기
      borderRadius: "8px",
      border: "2px solid #ffffff",
      backgroundColor: "#ffffff",
      color: "#555",
      fontWeight: "bold",
      cursor: password.length < 12 ? "pointer" : "not-allowed",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)",
    },
    deleteButtonWrapper: {
      display: "flex",
      justifyContent: "center",
      gap: "clamp(10px, 2vw, 20px)",
      marginBottom: "clamp(20px, 3vh, 40px)",
      marginTop: "clamp(20px, 3vh, 40px)",
    },
    deleteButton: {
      padding:"clamp(10px,2vw,15px)",
      fontSize:"clamp(16px,2.5vw,24px)",
      borderRadius:"8px",
      border:"2px solid rgb(255,223,223)",
      backgroundColor:"rgb(255,223,223)",
      color:"#555",
      fontWeight:"bold",
      cursor:"pointer",
    },
    actionButtonWrapper: {
      display: "flex",
      gap: "clamp(10px, 2vw, 20px)", // 버튼 간 간격
    },
    actionButton: {
      padding: "clamp(14px, 3vw, 20px)",
      fontSize: "clamp(18px, 3vw, 26px)", // 반응형 폰트 크기
      borderRadius: "8px",
      border: "none", // 테두리 제거
      fontWeight: "bold",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)", // 그림자 추가
    },
    cancelButton: {
      backgroundColor: "rgb(184,184,184)", // 회색 배경
      color: "#fff", // 흰색 텍스트
      cursor: password ? "pointer" : "not-allowed", // 비밀번호 없으면 비활성화
    },
    confirmButton: {
      backgroundColor: "rgb(111,168,248)", // 파란색 배경
      color: "#fff", // 흰색 텍스트
      cursor: password ? "pointer" : "not-allowed", // 비밀번호 없으면 비활성화
    },
  };

  return (
    <div style={responsiveStyles.container}>
      {/* 제목 */}
      <h2 style={responsiveStyles.title}>관리자 로그인</h2>

      {/* 비밀번호 입력 표시 */}
      <div style={responsiveStyles.passwordDisplay}>
        {'*'.repeat(password.length)}
      </div>

      {/* 에러 메시지 */}
      {errorMessage && (
        <p style={responsiveStyles.errorMessage}>{errorMessage}</p>
      )}

      {/* 가상 키보드 */}
      <div style={responsiveStyles.keyboardGrid}>
        {"1234567890QWERTYUIOPASDFGHJKLZXCVBNM".split("").map((key) => (
          <button
            key={key}
            onClick={() => handleKeyPress(key)}
            type="button"
            onMouseDown={(e) => e.target.style.backgroundColor = "#BBDEFB"}
            onMouseUp={(e) => e.target.style.backgroundColor = "#ffffff"}
            style={responsiveStyles.keyButton}
          >
            {key}
          </button>
        ))}
      </div>

      {/* 삭제 버튼 */}
      <div style={responsiveStyles.deleteButtonWrapper}>
        <button
          onClick={handleDelete}
          type="button"
          style={responsiveStyles.deleteButton}
        >
          ⌫ 삭 제
        </button>
        <button
          onClick={handleAllDelete}
          type="button"
          style={responsiveStyles.deleteButton}
        >
          전체 삭제
        </button>
      </div>

      {/* 취소 및 확인 버튼 */}
      <div style={responsiveStyles.actionButtonWrapper}>
        <button
          type="button"
          onClick={() => navigate("/")} // 취소 버튼 클릭 시 홈으로 이동
          disabled={!password} // 비밀번호 없으면 비활성화
          style={{
            ...responsiveStyles.actionButton,
            ...responsiveStyles.cancelButton,
          }}
        >
          취 소
        </button>
        <button
          type="submit"
          onClick={handleSubmit} // 확인 버튼 클릭 시 제출 처리
          disabled={!password} // 비밀번호 없으면 비활성화
          style={{
            ...responsiveStyles.actionButton,
            ...responsiveStyles.confirmButton,
          }}
        >
          확 인
        </button>
      </div>
    </div>
  );
};

export default AdminLogin;