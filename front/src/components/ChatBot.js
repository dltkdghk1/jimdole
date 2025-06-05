import React, { useState, useEffect, useRef } from "react";
import { useLocation } from "react-router-dom";
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  ConversationHeader
} from "@chatscope/chat-ui-kit-react";
import "@chatscope/chat-ui-kit-styles/dist/default/styles.min.css";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import Keyboard from "react-simple-keyboard";
import "react-simple-keyboard/build/css/index.css";
import api from "./api";

const Chatbot = () => {
  const location = useLocation();
  const [isOpen, setIsOpen] = useState(true);
  const [prevPathname, setPrevPathname] = useState(location.pathname);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [inputValue, setInputValue] = useState("");
  const [showKeyboard, setShowKeyboard] = useState(false);

  const {
    transcript,
    listening,
    resetTranscript,
    browserSupportsSpeechRecognition
  } = useSpeechRecognition();

  useEffect(() => {
    if (transcript) {
      // 기존 값과 새 transcript 비교 후 업데이트
      setInputValue(prev => {
        const newValue = transcript.replace(prev, '');
        return prev + newValue;
      });
    }
  }, [transcript]); // resetTranscript 제거

  const handleMicClick = () => {
    if (listening) {
      SpeechRecognition.stopListening();
      console.log('[AUDIO] 음성 인식 종료');
    } else {
      setInputValue(""); // 새 음성 입력 전 초기화
      resetTranscript();
      SpeechRecognition.startListening({
        continuous: true,
        interimResults: false, // 🔥 중간 결과 비활성화
        language: "ko-KR",
      });
      console.log('[AUDIO] 음성 인식 시작');
    }
  };

  useEffect(() => {
    if (location.pathname === "/" && prevPathname !== "/") {
      setIsOpen(false);
      setShowKeyboard(false);
      setInputValue("");
    }
    setPrevPathname(location.pathname);
  }, [location.pathname, prevPathname]);

  const chatContainerRef = useRef(null);
  const [keyboardHeight, setKeyboardHeight] = useState(0);

  // 키보드 높이 계산
  useEffect(() => {
    if (chatContainerRef.current && showKeyboard) {
      const containerHeight = chatContainerRef.current.offsetHeight;
      setKeyboardHeight(containerHeight * 0.4); // 컨테이너 높이의 40%로 키보드 설정
    }
  }, [showKeyboard]);
  
  const handleClose = () => {
    SpeechRecognition.stopListening();
    setIsOpen(false);
    setShowKeyboard(false);
    setMessages([]);
  };

  const handleKeyboardInput = (input) => {
    setInputValue(input);
  };

  // 메시지 전송 핸들러
  const handleSend = async (userMessage) => {
    if (!userMessage.trim()) return;
  
    const newUserMessage = { 
      message: userMessage, 
      sender: "user", 
      direction: "outgoing" 
    };
    setMessages((prev) => [...prev, newUserMessage]);
    setIsLoading(true);
  
    try {
      const response = await api.post('/chatbot', {
        model: "sonar",
        messages: [
          { 
            role: "system", 
            content: `친근한 말투로 답변. 이모티콘 1-2개 사용 🚀. 최대 2문장만 작성. 최대한 짧게 답변.`
          },
          ...messages.map((msg) => ({
            role: msg.sender === "user" ? "user" : "assistant",
            content: msg.message,
          })),
          { role: "user", content: userMessage },
        ],
        max_tokens: 200,
        temperature: 0.7,
        top_p: 0.9,
        stream: false
      });
  
      const data = response.data;
      console.log("[API] Response:", data);
      
      // 🔥 수정된 부분: choices[0] 추가
      const botMessage = { 
        message: data.choices[0].message.content.trim(), 
        sender: "bot", 
        direction: "incoming" 
      };
      setMessages((prev) => [...prev, botMessage]);
  
    } catch (error) {
      setMessages((prev) => [...prev, { 
        message: `⚠️ 오류: ${error.message}`, 
        sender: "bot", 
        direction: "incoming" 
      }]);
    } finally {
      setIsLoading(false);
    }
    // 전송 후 초기화
    setInputValue("");
    resetTranscript();
    SpeechRecognition.stopListening();
    setShowKeyboard(false);
  };

  if (!isOpen) return null;

  return (
    <>
      {/* 메인 컨테이너 */}
      <MainContainer
        style={{
          position: "fixed",
          bottom: "70px",
          right: "20px",
          width: "26%",
          height: showKeyboard ? `calc(100% - 200px)` : "75%", // 키보드 높이에 따라 동적 조정
          zIndex: "1001",
          overflow: "visible", // 키보드 가림 방지
        }}
      >
        <ChatContainer>
          <ConversationHeader>
            <ConversationHeader.Content>
              <span style={{ fontWeight: "600", fontSize: "18px" }}>✈️ AI 챗봇</span>
            </ConversationHeader.Content>
            <ConversationHeader.Actions>
              <button
                onClick={handleClose}
                style={{
                  backgroundColor: "rgb(247, 110, 110)",
                  color: "#fff",
                  borderRadius: "10px",
                  padding: "5px 10px",
                  cursor: "pointer",
                  border: "none",
                }}
              >
                닫기
              </button>
            </ConversationHeader.Actions>
          </ConversationHeader>
  
          <MessageList style={{ paddingRight: "15px" }}>
            {messages.map((msg, index) => (
              <Message key={index} model={msg} />
            ))}
            {isLoading && (
              <Message
                model={{
                  message: "✏️ 답변을 작성 중입니다...",
                  sender: "bot",
                  direction: "incoming",
                }}
              />
            )}
          </MessageList>
  
          <MessageInput
            style={{
              bottom: showKeyboard ? "250px" : "0px", // 입력창 위치 조정
            }}
            placeholder="궁금한 점을 입력하세요..."
            value={inputValue}
            onChange={(value) => setInputValue(value)}
            onSend={() => {
              handleSend(inputValue);
              console.log("[UI] Hiding keyboard after send");
              setInputValue("");
              setShowKeyboard(false);
            }}
          />
        </ChatContainer>
      </MainContainer>
  
      {/* 툴박스 */}
      <div
        style={{
          position: "absolute",
          bottom: showKeyboard ? `calc(16% + 300px)` : "170px", // 키보드 높이에 따라 동적 조정
          right: "30px",
          display: "flex",
          gap: "10px",
          zIndex: 2000, // 높은 z-index 설정
        }}
      >
        {/* 키보드 버튼 */}
        <button
          onClick={() => {
            console.log("[BUTTON] Toggling keyboard");
            setShowKeyboard((prev) => !prev);
          }}
          style={{
            width: "40px",
            height: "40px",
            borderRadius: "50%",
            backgroundColor: showKeyboard ? "#2890ff" : "#eee",
            border: "none",
            cursor: "pointer",
            fontSize: "20px",
          }}
        >
          ⌨️
        </button>
  
        {/* 마이크 버튼 */}
        <button
          onClick={handleMicClick}
          style={{
            width: "40px",
            height: "40px",
            borderRadius: "50%",
            backgroundColor: listening ? "#ff4d4d" : "#eee", // isListening → listening
            borderWidth: 0,
            cursor: "pointer",
            fontSize: "20px",
          }}
        >
          🎙️
        </button>
      </div>
  
      {/* 키보드 컨테이너 */}
      {showKeyboard && (
        <div
          style={{
            position: "fixed", // 화면 하단에 고정
            bottom: "70px", // 키보드가 항상 화면 하단에 위치하도록 설정
            right: "2%",
            width: "25%",
            height: "250px", // 키보드 높이 고정
            backgroundColor: "#fff",
            // boxShadow: "0px -5px 15px rgba(0,0,0,0.2)",
            zIndex: 1003,
            borderRadius: "10px",
          }}
        >
          <Keyboard
            onChange={(input) => setInputValue(input)}
            onKeyPress={(button) => {
              if (button === "{enter}") {
                handleSend(inputValue);
                setInputValue("");
                setShowKeyboard(false);
              }
            }}
            layout={{
              default: [
                "1 2 3 4 5 6 7 8 9 0 {bksp}",
                "ㅂ ㅈ ㄷ ㄱ ㅅ ㅛ ㅕ ㅑ ㅐ ㅔ",
                "ㅁ ㄴ ㅇ ㄹ ㅎ ㅗ ㅓ ㅏ ㅣ {enter}",
                "ㅋ ㅌ ㅊ ㅍ ㅠ ㅜ ㅡ , . ? {shift}",
                "{space}",
              ],
            }}
            display={{
              "{bksp}": "⌫",
              "{enter}": "⏎",
              "{shift}": "⇧",
              "{space}": "스페이스",
            }}
          />
        </div>
      )}
    </>
  );
}; 

export default Chatbot;