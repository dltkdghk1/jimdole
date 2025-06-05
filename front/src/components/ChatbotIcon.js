import React from 'react';
import chatbotIcon from '../assets/chatbot (1).png';

const ChatbotIcon = ({ onClick }) => (
  <img
    src={chatbotIcon}
    alt="Chatbot"
    onClick={onClick}
    style={{
      position: 'fixed',
      bottom: '20px',
      right: '130px',
      width: '80px',
      height: 'auto',
      cursor: 'pointer',
      zIndex: 1000
    }}
  />
);

export default ChatbotIcon;