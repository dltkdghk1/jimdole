package com.ssafy.carrybot.flight.service;

import com.ssafy.carrybot.flight.dto.request.FlightRequestDTO;
import com.ssafy.carrybot.flight.dto.response.FlightResponseDTO;
import com.ssafy.carrybot.flight.entity.Flight;
import com.ssafy.carrybot.flight.repository.FlightRepository;
import com.ssafy.carrybot.gate.entity.Gate;
import com.ssafy.carrybot.gate.repository.GateRepository;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
public class FlightServiceImpl implements FlightService {

    private final FlightRepository flightRepository;
    private final GateRepository gateRepository;

    @Override
    public FlightResponseDTO registerFlight(FlightRequestDTO dto) {

        //JPA가 내부적으로 SQL을 날려서 DB에서 해당 ID를 가진 Gate 객체를 직접 찾아서 꺼냄
        //즉, DB에서 직접 조회해서 가져온 진짜 Gate 엔티티 객체이다.
        Gate gate = gateRepository.findById(dto.getGateId())
                .orElseThrow(() -> new EntityNotFoundException("게이트를 찾을 수 없습니다."));

        Flight flight = Flight.builder()
                .airline(dto.getAirline())
                .destination(dto.getDestination())
                .time(dto.getTime())
                .gate(gate) // DB에서 조회한 실제 Gate 객체를 넣음
                .build();

//gate(gate)인 이유: FlightRequestDTO에는 gate가 없고 gateId만 있음.
// dto에는 Gate 객체가 없기 때문에 → dto.getGate()가 아니라,
// dto.getGateId()로 Gate ID만 꺼내서
// 그 ID로 DB에서 Gate를 조회한 뒤 → Flight에 연결

        // DB에 저장하고, ID 등 모든 값이 채워진 상태의 Admin 객체를 saved에 담은 거
        //save(flight)->jpa에서 자동으로 해주는건데, JPA가 DB에 flight 객체를 저장하고, 저장된 결과를 반환
        Flight saved = flightRepository.save(flight);

        return new FlightResponseDTO(
                saved.getId(),
                saved.getAirline(),
                saved.getDestination(),
                saved.getTime(),
                saved.getGate().getId(), //응답 DTO에는 private Long gateId;만 있어서 Gate 다 안보내고 id만 보내려고
                saved.getGate().getGateNumber()
        );
    }

    @Override
    public List<FlightResponseDTO> getAllFlights() {
        return flightRepository.findAll().stream()
                .map(f -> new FlightResponseDTO(
                        f.getId(), f.getAirline(), f.getDestination(), f.getTime(),
                        f.getGate().getId(), f.getGate().getGateNumber()))
                .collect(Collectors.toList());
    }

    @Override
    public FlightResponseDTO getFlightById(Long id) {
        Flight f = flightRepository.findById(id)
                .orElseThrow(() -> new EntityNotFoundException("항공편을 찾을 수 없습니다."));
        return new FlightResponseDTO(
                f.getId(), f.getAirline(), f.getDestination(), f.getTime(),
                f.getGate().getId(), f.getGate().getGateNumber()
        );
    }
}
