
import React, { useState, useEffect } from "react";
import api from "../components/api"; // Axios 인스턴스

const Admin = () => {
  const [logs, setLogs] = useState([]); // 로봇 로그 상태
  const [errorMessage, setErrorMessage] = useState(""); // 에러 메시지 상태
  const [sortOrder, setSortOrder] = useState("asc"); // 시간 정렬 상태 (asc: 오름차순, desc: 내림차순)
  const [searchTerm, setSearchTerm] = useState(""); // 검색어 상태
  const [filteredLogs, setFilteredLogs] = useState([]); // 검색 결과 필터링된 로그 상태

  // 컴포넌트 마운트 시 로봇 로그 조회
  useEffect(() => {
    const fetchLogs = async () => {
      try {
        console.log("API 호출: /robot/logs");
        const response = await api.get("/robot/logs");
        console.log("API 응답 데이터:", response.data);
        setLogs(response.data); // 로그 데이터 저장
        setFilteredLogs(response.data); // 초기 필터링된 로그 설정
      } catch (error) {
        console.error("API 호출 중 오류 발생:", error);
        setErrorMessage("오류가 발생했습니다.\n잠시 후 다시 시도해주세요.");
      }
    };

    fetchLogs();
  }, []);

  /**
   * 이벤트 타입에 따라 로그 메시지 렌더링
   * @param {Object} log - 단일 로그 객체
   * @returns {String} 렌더링할 메시지 문자열
   */
  const renderLogMessage = (log) => {
    switch (log.eventType) {
      case "MOVED":
        return `게이트 ${log.gateNumber}로 이동 중 (${log.reservationCode || ""})`;
      case "STOPPED":
        return `게이트 ${log.gateNumber} 도착 (${log.reservationCode || ""})\n사진 촬영 및 전송 완료`;
      case "PHOTO_SENT":
        return `사진 촬영 및 전송 완료 (${log.reservationCode || ""})`;
      case "EMERGENCY_STOP":
        return `긴급 정지 발생`;
      case "MANUAL_RESUMED":
        return `수동 재가동됨 (${log.reservationCode || ""})`;
      default:
        return `알 수 없는 이벤트`;
    }
  };

  /**
   * 시간 정렬 함수
   * @param {String} order - 정렬 순서 ("asc" 또는 "desc")
   */
  const sortLogs = (order) => {
    const sortedLogs = [...filteredLogs].sort((a, b) =>
      order === "asc"
        ? new Date(a.timestamp) - new Date(b.timestamp)
        : new Date(b.timestamp) - new Date(a.timestamp)
    );
    setFilteredLogs(sortedLogs);
    setSortOrder(order); // 현재 정렬 상태 업데이트
  };

  /**
   * 검색 필터링 함수
   */
  const handleSearch = () => {
    const filtered = logs.filter((log) =>
      log.reservationCode?.toLowerCase().includes(searchTerm.toLowerCase())
    );
    setFilteredLogs(filtered);
  };

  // 반응형 스타일 객체
  const responsiveStyles = {
    container: {
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      height: "100vh",
      backgroundColor: "#f5f5f5",
      paddingTop: "clamp(20px, 4vw, 60px)",
      paddingBottom: "clamp(10px, 2vw, 20px)",
      overflow: "hidden",
    },
    title: {
      marginTop: "clamp(20px, 7vh, 70px)",
      marginBottom: "clamp(20px, 2vh, 60px)",
      fontSize: "clamp(28px, 4.5vw, 40px)",
      color: "rgb(67, 67, 67)",
    },
    errorMessage: {
      color: "red",
      marginBottom: "clamp(15px, 1.5vh, 30px)",
      fontSize: "clamp(16px, 2vw, 24px)",
    },
    searchWrapper: {
      display: "flex",
      justifyContent: "flex-end", // 검색창을 오른쪽 끝으로 정렬
      alignItems: "center",
      width: "90%",
      marginBottom: "clamp(15px, 2vh, 30px)",
    },
    sortButton: {
      padding: "5px", // 버튼 크기 조정
      marginLeft: "10px",
      fontSize: "14px", // 버튼 텍스트 크기 조정
      borderRadius: "8px",
      border: "2px solid #ffffff",
      backgroundColor: "#ffffff",
      color: "#555",
      fontWeight: "bold",
      cursor: "pointer",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)", // 그림자 추가
    },
    searchInput: {
      width: "200px", // 검색 입력란의 너비를 작게 설정
      padding: "8px",
      fontSize: "16px",
      borderRadius: "8px",
      border: "1px solid #ccc",
    },
    searchButton: {
      marginLeft: "10px", // 입력란과 버튼 사이 간격 추가
      padding: "8px",
      fontSize: "16px",
      borderRadius: "8px",
      cursor: "pointer",
      backgroundColor: "#BBDEFB", // 검색 버튼 배경색
      color: "#555", // 텍스트 색상
      fontWeight: "bold",
      border: "2px solid #BBDEFB",
      boxShadow: "0px 4px 6px rgba(0,0,0,0.1)", // 그림자 추가
    },
    tableWrapper: {
      width: "90%",
      maxHeight: "55%",
      overflowY: "scroll", // 테이블 내부 스크롤 활성화
      borderRadius: "8px",
      backgroundColor: "#ffffff", // 테이블 배경 흰색
      boxShadow: "0 4px 8px rgba(0,0,0,0.1)", // 약간의 그림자 추가
    },
    table: {
      width: "100%",
      borderCollapse: "collapse",
    },
    tableHeadRow: {
      position: "sticky", // 헤더 고정 설정
      top: 0,
      backgroundColor: "#d9f0ff", // 연한 하늘색 헤더 배경색
      borderBottom: "1px solid #ccc",
    },
    tableCellHeadWithButtonsWrapper:{
       display:"flex", 
       justifyContent:"center", 
       alignItems:"center"
     }, 
     tableCellHead: {
      padding: "clamp(10px, 1vw, 15px)",
      textAlign: "center",
      fontSize: "clamp(16px, 2vw, 24px)", // 반응형 폰트 크기
    },
     tableCellBody:{
       padding:"clamp(10px, 1vw, 20px)",
       textAlign:"center",
       fontSize:"clamp(14px, 3vw, 20px)", 
     },
     tableRowBody:{
        borderBottom:"1 px solid #ccc"
     }
  };

    return (
      <div style={responsiveStyles.container}>
        <h2 style={responsiveStyles.title}>짐돌이 로그</h2>
    
        {/* 에러 메시지 표시 */}
        {errorMessage && (
          <p style={responsiveStyles.errorMessage}>{errorMessage}</p>
        )}
    
        {/* 검색 입력란 */}
        <div style={responsiveStyles.searchWrapper}>
          <input
            type="text"
            placeholder="예약번호 검색"
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            style={responsiveStyles.searchInput}
          />
          <button
            onClick={handleSearch}
            style={responsiveStyles.searchButton}
          >
            검색
          </button>
        </div>
    
        {/* 로그 테이블 */}
        <div style={responsiveStyles.tableWrapper}>
          <table style={{ width: "100%", borderCollapse: "collapse" }}>
            <thead>
              <tr style={responsiveStyles.tableHeadRow}>
                <th style={responsiveStyles.tableCellHead}>
                  <div style={responsiveStyles.tableCellHeadWithButtonsWrapper}>
                    시간
                    <div style={responsiveStyles.sortButtonsWrapper}>
                      <button
                        onClick={() => sortLogs("asc")}
                        style={{
                          ...responsiveStyles.sortButton,
                          backgroundColor: sortOrder === "asc" ? "rgb(255, 215, 215)" : "rgb(208, 208, 208)",
                        }}
                      >
                        🔺
                      </button>
                      <button
                        onClick={() => sortLogs("desc")}
                        style={{
                          ...responsiveStyles.sortButton,
                          backgroundColor: sortOrder === "desc" ? "rgb(255, 215, 215)" : "rgb(208, 208, 208)",
                        }}
                      >
                        🔻
                      </button>
                    </div>
                  </div>
                </th>
                <th style={responsiveStyles.tableCellHead}>내용</th>
              </tr>
            </thead>
            <tbody>
              {filteredLogs.map((log, index) => (
                <tr key={index} style={responsiveStyles.tableRowBody}>
                  <td style={responsiveStyles.tableCellBody}>
                    {new Date(log.timestamp).toLocaleString()} {/* 시간 포맷팅 */}
                  </td>
                  <td style={responsiveStyles.tableCellBody}>
                    {renderLogMessage(log)} {/* 이벤트 메시지 렌더링 */}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    );
};

export default Admin;