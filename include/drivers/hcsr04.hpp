#pragma once
#include "mbed.h"

namespace drivers {

class CHCSR04 {
public:
    CHCSR04(PinName trigPin, PinName echoPin, int timeout_ms = 20);

    void trigger();      // starts a measurement (10us pulse)
    void poll_timeout(); // call periodically to detect timeout

    bool has_new() const { return m_new; }
    void clear_new() { m_new = false; }

    // returns -1 on timeout/invalid
    int distance_mm() const;

    bool measuring() const { return m_measuring; }

private:
    void on_rise();
    void on_fall();

    DigitalOut   m_trig;
    InterruptIn  m_echo;

    volatile uint32_t m_rise_us {0};
    volatile uint32_t m_pulse_us {0};
    volatile bool     m_got_rise {false};
    volatile bool     m_new {false};
    volatile bool     m_valid {false};
    volatile bool     m_measuring {false};

    int      m_timeout_ms;
    uint32_t m_trigger_ms {0};
};

} // namespace drivers
