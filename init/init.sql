SET NAMES utf8mb4;

CREATE DATABASE IF NOT EXISTS carrybot;
CREATE USER IF NOT EXISTS 'robot'@'%' IDENTIFIED BY 'rotbot104';
GRANT ALL PRIVILEGES ON carrybot.* TO 'robot'@'%';
FLUSH PRIVILEGES;

USE carrybot;

-- 🛬 게이트
CREATE TABLE gate (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    gate_number VARCHAR(10) UNIQUE NOT NULL,
    x DECIMAL(9,6) NOT NULL,
    y DECIMAL(9,6) NOT NULL
);

-- ✈️ 항공편
CREATE TABLE flight (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    airline VARCHAR(100) NOT NULL,
    destination VARCHAR(100) NOT NULL,
    time DATETIME NOT NULL,
    gate_id BIGINT NOT NULL,
    FOREIGN KEY (gate_id) REFERENCES gate(id)
);

-- 🧾 예약
CREATE TABLE reservation (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    reservation_code VARCHAR(50) UNIQUE NOT NULL,
    phone_number VARCHAR(20) NOT NULL,
    is_expired BOOLEAN DEFAULT FALSE,
    flight_id BIGINT NOT NULL,
    FOREIGN KEY (flight_id) REFERENCES flight(id)
);

-- 🕓 만료 예약 이력
CREATE TABLE expired_reservation (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    reservation_id BIGINT UNIQUE,
    reservation_code VARCHAR(50) UNIQUE NOT NULL,
    flight_id BIGINT NOT NULL,
    moved_at DATETIME NOT NULL,
    FOREIGN KEY (reservation_id) REFERENCES reservation(id) ON DELETE SET NULL,
    FOREIGN KEY (flight_id) REFERENCES flight(id)
);

-- 📷 사진
CREATE TABLE photo (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    reservation_id BIGINT NOT NULL,
    image_url VARCHAR(255) NOT NULL,
    FOREIGN KEY (reservation_id) REFERENCES reservation(id) ON DELETE CASCADE
);

-- 🤖 로봇
CREATE TABLE robot (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    current_x DECIMAL(9,6) NOT NULL,
    current_y DECIMAL(9,6) NOT NULL,
    start_x DECIMAL(9,6) NOT NULL,
    start_y DECIMAL(9,6) NOT NULL,
    target_gate VARCHAR(10),
    status ENUM('IDLE', 'MOVING', 'STOPPED', 'EMERGENCY_STOP') NOT NULL,
    last_updated DATETIME NOT NULL
);

-- 📋 로봇 로그
CREATE TABLE robot_log (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    robot_id BIGINT NOT NULL,
    event_type ENUM(
        'MOVED',
        'STOPPED',
        'PHOTO_SENT',
        'EMERGENCY_STOP',
        'MANUAL_RESUMED'
    ) NOT NULL,
    event VARCHAR(255),
    timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    reservation_code VARCHAR(50),
    gate_number VARCHAR(10),
    from_x DECIMAL(9,6),
    from_y DECIMAL(9,6),
    to_x DECIMAL(9,6),
    to_y DECIMAL(9,6),
    FOREIGN KEY (robot_id) REFERENCES robot(id) ON DELETE CASCADE
);

-- ⛔ 멈춤 기록
CREATE TABLE stop (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    robot_id BIGINT NOT NULL,
    timestamp DATETIME NOT NULL,
    reason VARCHAR(255) NOT NULL,
    resolved_time DATETIME,
    resume_action ENUM('RESUME_GATE', 'RESUME_START') NOT NULL,
    FOREIGN KEY (robot_id) REFERENCES robot(id) ON DELETE CASCADE
);

-- 🔐 관리자
CREATE TABLE admin (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    password VARCHAR(20) NOT NULL
);

-- 📍 초기 더미 데이터
INSERT INTO gate (gate_number, x, y) VALUES
('A', -61.974000, -53.235000),
('B', -62.758000, -43.768000);

INSERT INTO flight (airline, destination, time, gate_id)
VALUES ('Korean Air', 'Jeju', '2025-04-01 08:30:00', 1),
       ('Asiana', 'Busan', '2025-04-01 10:00:00', 2);

INSERT INTO reservation (reservation_code, phone_number, is_expired, flight_id)
VALUES ('ABC123', '01011112222', false, 1),
       ('DEF456', '01033334444', false, 2);

-- 🤖 로봇 시작 위치
INSERT INTO robot (current_x, current_y, start_x, start_y, target_gate, status, last_updated)
VALUES (-50.000000, -50.001000, -50.000000, -50.001000, NULL, 'IDLE', NOW());

-- 🔐 관리자 기본 계정
INSERT INTO admin (password) VALUES ('1234');

-- 📄 로봇 로그: 게이트 A 도착 로그
INSERT INTO robot_log (
    robot_id, event_type, event, timestamp,
    reservation_code, gate_number,
    from_x, from_y, to_x, to_y
)
VALUES (
    1, 'STOPPED', '게이트 A 도착 완료', NOW(),
    'ABC123', 'A',
    -50.000000, -50.001000, -61.974000, -53.235000
);

-- 📄 로봇 로그: 게이트 B 도착 로그
INSERT INTO robot_log (
    robot_id, event_type, event, timestamp,
    reservation_code, gate_number,
    from_x, from_y, to_x, to_y
)
VALUES (
    1, 'STOPPED', '게이트 B 도착 완료', NOW(),
    'DEF456', 'B',
    -50.000000, -50.001000, -62.758000, -43.768000
);
