package com.ssafy.carrybot.robot.spec;

import com.ssafy.carrybot.robot.entity.RobotLog;
import com.ssafy.carrybot.robot.enums.EventType;
import org.springframework.data.jpa.domain.Specification;

import java.time.LocalDate;

//검색 조건 처리
public class RobotLogSpecification {

    public static Specification<RobotLog> search(
            String reservationCode,
            EventType eventType,
            LocalDate startDate,
            LocalDate endDate
    ) {
        return (root, query, cb) -> {
            var predicate = cb.conjunction();

            // 예약번호 필터링
            if (reservationCode != null && !reservationCode.isEmpty()) {
                predicate = cb.and(predicate,
                        cb.equal(root.get("reservationCode"), reservationCode));
            }

            // 이벤트 타입 필터링
            if (eventType != null) {
                predicate = cb.and(predicate,
                        cb.equal(root.get("eventType"), eventType));
            }


            // 시작 날짜 필터링
            if (startDate != null) {
                predicate = cb.and(predicate,
                        cb.greaterThanOrEqualTo(root.get("timestamp"), startDate.atStartOfDay()));
            }

            // 종료 날짜 필터링
            if (endDate != null) {
                predicate = cb.and(predicate,
                        cb.lessThanOrEqualTo(root.get("timestamp"), endDate.atTime(23, 59, 59)));
            }

            return predicate;
        };
    }
}
//필터링 여기서 구현