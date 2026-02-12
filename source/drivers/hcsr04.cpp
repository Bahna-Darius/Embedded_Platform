#include "drivers/hcsr04.hpp"

namespace drivers {

static inline uint32_t now_us() { return us_ticker_read(); }
static inline uint32_t now_ms() { return Kernel::get_ms_count(); }

CHCSR04::CHCSR04(PinName trigPin, PinName echoPin, int timeout_ms)
    : m_trig(trigPin)
    , m_echo(echoPin, PullDown)
    , m_timeout_ms(timeout_ms)
{
    m_trig = 0;
    m_echo.rise(callback(this, &CHCSR04::on_rise));
    m_echo.fall(callback(this, &CHCSR04::on_fall));
}

void CHCSR04::trigger()
{
    if (m_measuring) return;

    m_measuring = true;
    m_got_rise  = false;
    m_new       = false;
    m_valid     = false;
    m_pulse_us  = 0;
    m_trigger_ms = now_ms();

    m_trig = 0; wait_us(2);
    m_trig = 1; wait_us(10);
    m_trig = 0;
}

void CHCSR04::poll_timeout()
{
    if (!m_measuring) return;

    if ((uint32_t)(now_ms() - m_trigger_ms) > (uint32_t)m_timeout_ms) {
        m_measuring = false;
        m_valid = false;
        m_new = true;     // “new result” = timeout
    }
}

void CHCSR04::on_rise()
{
    if (!m_measuring) return;
    if (m_got_rise) return;

    m_rise_us = now_us();
    m_got_rise = true;
}

void CHCSR04::on_fall()
{
    if (!m_measuring) return;
    if (!m_got_rise) return;

    uint32_t fall = now_us();
    m_pulse_us = fall - m_rise_us;

    m_valid = true;
    m_measuring = false;
    m_new = true;
}

int CHCSR04::distance_mm() const
{
    if (!m_valid) return -1;

    // distance_mm = pulse_us * 0.1715 (integer)
    // = pulse_us * 343 / 2000
    return (int)((m_pulse_us * 343u) / 2000u);
}

} // namespace drivers
