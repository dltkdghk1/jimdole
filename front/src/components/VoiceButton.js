import React, { useEffect } from 'react';
import SpeechRecognition, { useSpeechRecognition } from 'react-speech-recognition';
import { useTranslation } from 'react-i18next';
import styled, { keyframes } from 'styled-components';

// 음성 파동 애니메이션
const wave = keyframes`
  0% { height: 10px; }
  50% { height: 30px; }
  100% { height: 10px; }
`;

const VoiceButtonContainer = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-left: clamp(10px, 2vw, 20px);
`;

const MicButton = styled.button`
  width: clamp(40px, 8vw, 60px);
  height: clamp(40px, 8vw, 60px);
  border-radius: 50%;
  background: ${props => props.$listening ? '#f17272' : '#70aff3'};
  border: none;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;

  &:hover {
    transform: scale(1.05);
  }
`;

const WaveAnimation = styled.div`
  display: flex;
  gap: 4px;
  height: 40px;
  align-items: flex-end;
  margin-top: clamp(8px, 1.5vh, 15px);
`;

const Bar = styled.div`
  width: 6px;
  background: #62aaf7;
  animation: ${wave} 1s ease infinite;
  animation-delay: ${props => props.$delay};
`;

const FeedbackMessage = styled.div`
  position: fixed;
  bottom: clamp(100px, 15vh, 200px);
  right: clamp(20px, 3vw, 30px);
  background: rgba(0,0,0,0.9);
  color: white;
  padding: clamp(10px, 2vw, 15px);
  border-radius: 12px;
  max-width: min(300px, 90vw);
  font-size: clamp(14px, 2vw, 16px);
  text-align: center;
`;

const VoiceButton = ({ onVoiceInput = () => {}, maxLength }) => {
  const { t, i18n } = useTranslation();
  const { transcript, listening, resetTranscript, browserSupportsSpeechRecognition } = useSpeechRecognition();

  // 음성 입력 처리
  useEffect(() => {
    if (transcript) {
      const formatted = transcript
        .toUpperCase()
        .replace(/[^A-Z0-9]/g, '')
        .slice(0, maxLength || 100); // maxLength 프로퍼티 지원

      if (formatted) {
        onVoiceInput(formatted);
        if (maxLength && formatted.length >= maxLength) {
          resetTranscript();
        }
      }
    }
  }, [transcript, maxLength, onVoiceInput, resetTranscript]);

  // 음성 인식 토글
  const toggleListening = () => {
    if (listening) {
      SpeechRecognition.stopListening();
    } else {
      SpeechRecognition.startListening({ 
        continuous: true, // 지속적 음성 인식
        interimResults: true, // 중간 결과 수신
        language: i18n.language === 'ko' ? 'ko-KR' : 'en-US',
      });
    }
  };

  if (!browserSupportsSpeechRecognition) {
    return <div style={{ color: 'red' }}>{t('voice.unsupported')}</div>;
  }

  return (
    <VoiceButtonContainer>
      <MicButton 
        onClick={toggleListening}
        $listening={listening}
        aria-label={listening ? t('voice.stop') : t('voice.start')}
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="white">
          <path d="M12 14c1.66 0 3-1.34 3-3V5c0-1.66-1.34-3-3-3S9 3.34 9 5v6c0 1.66 1.34 3 3 3z"/>
          <path d="M17 11c0 2.76-2.24 5-5 5s-5-2.24-5-5H5c0 3.53 2.61 6.43 6 6.92V21h2v-3.08c3.39-.49 6-3.39 6-6.92h-2z"/>
        </svg>
      </MicButton>

      {listening && (
        <WaveAnimation>
          {[...Array(5)].map((_, i) => (
            <Bar key={i} $delay={`${i * 0.2}s`} />
          ))}
        </WaveAnimation>
      )}
    </VoiceButtonContainer>
  );
};

export default VoiceButton;