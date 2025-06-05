package com.ssafy.carrybot.flight.controller;

import com.ssafy.carrybot.flight.dto.request.FlightRequestDTO;
import com.ssafy.carrybot.flight.dto.response.FlightResponseDTO;
import com.ssafy.carrybot.flight.service.FlightService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/flights")
public class FlightController {

    private final FlightService flightService;

    @PostMapping
    public ResponseEntity<FlightResponseDTO> registerFlight(@RequestBody FlightRequestDTO dto) {
        return ResponseEntity.ok(flightService.registerFlight(dto));
    }

    @GetMapping
    public ResponseEntity<List<FlightResponseDTO>> getAllFlights() {
        return ResponseEntity.ok(flightService.getAllFlights());
    }

    @GetMapping("/{id}")
    public ResponseEntity<FlightResponseDTO> getFlightById(@PathVariable Long id) {
        return ResponseEntity.ok(flightService.getFlightById(id));
    }
}

