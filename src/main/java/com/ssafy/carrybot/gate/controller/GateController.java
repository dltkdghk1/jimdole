package com.ssafy.carrybot.gate.controller;

import com.ssafy.carrybot.gate.dto.request.GateRequestDTO;
import com.ssafy.carrybot.gate.dto.response.GateResponseDTO;
import com.ssafy.carrybot.gate.service.GateService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/gates")
public class GateController {

    private final GateService gateService;

    @PostMapping //게이트 등록
    public ResponseEntity<GateResponseDTO> registerGate(@RequestBody GateRequestDTO dto) {
        GateResponseDTO response = gateService.registerGate(dto);
        return ResponseEntity.ok(response);
    }

    @GetMapping //모든 게이트 조회
    public ResponseEntity<List<GateResponseDTO>> getAllGates(){
        return ResponseEntity.ok(gateService.getAllGates());
    }

    @GetMapping("/{id}")
    public ResponseEntity<GateResponseDTO> getGateById(@PathVariable Long id){
        return ResponseEntity.ok(gateService.getGateById(id));
    }
}
