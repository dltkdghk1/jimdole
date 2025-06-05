// hooks/useSpeechHandler.js
import { useState, useEffect } from 'react';
import SpeechRecognition, { useSpeechRecognition } from 'react-speech-recognition';

const useSpeechHandler = ({ 
  continuous = false,  // 🔥 단일 발화 모드로 변경
  interimResults = false, 
  language = 'en-US', 
  onResult 
}) => {
  const [isListening, setIsListening] = useState(false);
  const { transcript, listening, resetTranscript, browserSupportsSpeechRecognition } = useSpeechRecognition();

  const startListening = () => {
    if (!browserSupportsSpeechRecognition) return;
    resetTranscript();  // 🔥 시작 전 초기화 추가
    SpeechRecognition.startListening({ 
      continuous, 
      interimResults, 
      language 
    });
    setIsListening(true);
  };

  const stopListening = () => {
    SpeechRecognition.stopListening();
    setIsListening(false);
  };

  useEffect(() => {
    if (transcript && onResult) {
      onResult(transcript);
      resetTranscript();  // 🔥 결과 처리 후 초기화
    }
  }, [transcript, onResult, resetTranscript]);

  // 음성 인식 상태 동기화
  useEffect(() => {
    setIsListening(listening);
  }, [listening]);

  return { 
    isListening, 
    startListening, 
    stopListening,
    resetTranscript 
  };
};

export default useSpeechHandler;
