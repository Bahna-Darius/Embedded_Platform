#pragma once
#include "mbed.h"
#include "utils/task.hpp"
#include "drivers/hcsr04.hpp"
#include "brain/robotstatemachine.hpp"
#include "brain/globalsv.hpp"

namespace periodics {

class CUltrasonic : public utils::CTask {
public:
    CUltrasonic(std::chrono::milliseconds period,
                UnbufferedSerial& serial,
                brain::CRobotStateMachine& rsm,
                PinName trigF, PinName echoF,
                PinName trigR, PinName echoR);

    // Brain command:
    //   #usFovr:1;;  -> override ON (temporary)
    //   #usFovr:0;;  -> override OFF
    void serialCallbackUSFrontOverride(const char* msg, char* resp);
    void serialCallbackPARKcommand(const char* msg, char* resp);

private:
    void _run() override;

    bool kl_ultra_enabled() const;
    bool rear_publish_allowed() const;

    void publish_rear_mm(int mm);
    void publish_front_mm(int mm); // optional debug

    UnbufferedSerial& m_serial;
    brain::CRobotStateMachine& m_rsm;

    drivers::CHCSR04 m_front;
    drivers::CHCSR04 m_rear;

    enum Phase { TRIG_FRONT, TRIG_REAR } m_phase {TRIG_FRONT};
    uint32_t m_next_trigger_ms {0};

    bool m_rear_enabled {false};   // controlled by #park command

    // Front braking thresholds
    int m_front_brake_mm   {150};   // brake if <= 150mm
    int m_front_release_mm {200};  // release if >= 200mm (hysteresis)
    bool m_front_stop_req {false};

    // Brain override
    bool m_front_override {false};

    // Trigger spacing (avoid cross-talk)
    const uint32_t m_gap_ms {60};
};

} // namespace periodics
