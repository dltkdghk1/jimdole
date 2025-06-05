package com.ssafy.carrybot.gate.service;

import com.ssafy.carrybot.gate.dto.request.GateRequestDTO;
import com.ssafy.carrybot.gate.dto.response.GateResponseDTO;

import java.util.List;

public interface GateService {

    GateResponseDTO registerGate(GateRequestDTO dto);//게이트 등록, 요청을 보낼 때 데이터 받음, 저장 후 응답
    List<GateResponseDTO> getAllGates(); //전체 게이트 조회니깐 입력값 필요없음 => (). 여러 개 게이트 가져오니깐 리스트로 반환
    GateResponseDTO getGateById(Long id); //어떤 게이트 조회할지 id로 결정
}

//registerGate 메서드 이름: 게이트 등록 기능
//(GateRequestDTO dto) 입력: 클라이언트가 보낸 요청 데이터(dto에 담김)
//GateResponseDTO 출력: 등록 결과를 담은 응답 DTO
//GateResponseDTO registerGate(GateRequestDTO dto) 예시로 들면 클라이언트가 보낸 게이트 정보를 받아서 DB에 등록하고 결과를 GateResponseDTO로 반환
//인터페이스는 이 서비스는 이런 기능을 제공해야해라는 기능 설계 목록일 뿐