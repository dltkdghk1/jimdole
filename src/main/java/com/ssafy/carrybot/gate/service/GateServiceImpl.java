package com.ssafy.carrybot.gate.service;

import com.ssafy.carrybot.gate.dto.request.GateRequestDTO;
import com.ssafy.carrybot.gate.dto.response.GateResponseDTO;
import com.ssafy.carrybot.gate.entity.Gate;
import com.ssafy.carrybot.gate.repository.GateRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor //이건 서비스 말고 Impl에서 사용, gateRepository를 자동 주입해줌
public class GateServiceImpl implements GateService{
    private final GateRepository gateRepository; //DB 작업 처리하는 JPA 인터페이스

    @Override
    public GateResponseDTO registerGate(GateRequestDTO dto){
        Gate gate = Gate.builder()
                .gateNumber(dto.getGateNumber())
                .x(dto.getX())
                .y(dto.getY())
                .build();//Builder로 Gate 객체 생성

        Gate saved = gateRepository.save(gate); //JPA가 객체를 감지하고 실제 DB에 저장하려면 save()가 필요

        return new GateResponseDTO(saved.getId(), saved.getGateNumber(), saved.getX(), saved.getY());
    }

    @Override
    public List<GateResponseDTO> getAllGates() {
        return gateRepository.findAll().stream()
                .map(g -> new GateResponseDTO(g.getId(), g.getGateNumber(), g.getX(), g.getY()))
                .collect(Collectors.toList());
    }

    @Override
    public GateResponseDTO getGateById(Long id) {
        Gate gate = gateRepository.findById(id)
                .orElseThrow(() -> new IllegalArgumentException("게이트를 찾을 수 없습니다."));

        return new GateResponseDTO(gate.getId(), gate.getGateNumber(), gate.getX(), gate.getY());
    }

}
