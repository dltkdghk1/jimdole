import { useEffect, useRef } from 'react';
import SockJS from 'sockjs-client';
import { Client } from '@stomp/stompjs';

export default function useRobotWebSocket(onMessage) {
  const clientRef = useRef(null);

  useEffect(() => {
    console.log('[WebSocket] Initializing connection...');
    
    const stompClient = new Client({
      webSocketFactory: () => new SockJS('https://j12e104.p.ssafy.io/ws'),
      reconnectDelay: 5000,
      debug: (str) => console.log('[STOMP]', str),
      onConnect: (frame) => {
        console.log('[WebSocket] Connected:', frame);
        stompClient.subscribe('/topic/robot/status', (message) => {
          try {
            const data = JSON.parse(message.body);
            console.log('[WebSocket] Received:', data);
            onMessage(data);
          } catch (error) {
            console.error('[WebSocket] Message parse error:', error);
          }
        });
      },
      onStompError: (frame) => {
        console.error('[WebSocket] Protocol Error:', frame.headers.message);
      }
    });

    stompClient.activate();
    clientRef.current = stompClient;

    return () => {
      console.log('[WebSocket] Disconnecting...');
      if (clientRef.current?.active) {
        clientRef.current.deactivate();
      }
    };
  }, [onMessage]);
}