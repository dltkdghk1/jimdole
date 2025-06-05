import React from "react";
import { useTranslation } from "react-i18next";

const LanguageSwitch = ({ onLanguageChange }) => {
  const { i18n } = useTranslation();

  const toggleLanguage = () => {
    const newLanguage = i18n.language === "ko" ? "en" : "ko";
    i18n.changeLanguage(newLanguage);
  };

  return (
    <div
      style={{
        position: "fixed",
        bottom: "30px",
        right: "30px",
        zIndex: 1000, // 다른 요소 위에 표시
      }}
    >
      <button
        onClick={toggleLanguage}
        style={{
          padding: "5px 10px",
          fontSize: "25px",
          borderRadius: "4px",
          border: "1px solid #ccc",
          backgroundColor: "#f5f5f5",
          cursor: "pointer",
        }}
      >
        {i18n.language === "ko" ? "Eng" : "한글"} {/* 현재 언어에 따라 버튼 텍스트 변경 */}
      </button>
    </div>
  );
};

export default LanguageSwitch;
