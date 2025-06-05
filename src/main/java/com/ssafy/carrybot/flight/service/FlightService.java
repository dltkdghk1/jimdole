package com.ssafy.carrybot.flight.service;

import com.ssafy.carrybot.flight.dto.request.FlightRequestDTO;
import com.ssafy.carrybot.flight.dto.response.FlightResponseDTO;

import java.util.List;

public interface FlightService {

    FlightResponseDTO registerFlight(FlightRequestDTO dto); // 항공편 등록
    List<FlightResponseDTO> getAllFlights(); //전체 항공편
    FlightResponseDTO getFlightById(Long id); //특정 항공편
}
