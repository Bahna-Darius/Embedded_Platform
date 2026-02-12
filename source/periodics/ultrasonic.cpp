#include "periodics/ultrasonic.hpp"

namespace periodics {

static inline uint32_t now_ms() { return Kernel::get_ms_count(); }

CUltrasonic::CUltrasonic(std::chrono::milliseconds period,
                         UnbufferedSerial& serial,
                         brain::CRobotStateMachine& rsm,
                         PinName trigF, PinName echoF,
                         PinName trigR, PinName echoR)
    : utils::CTask(period)
    , m_serial(serial)
    , m_rsm(rsm)
    , m_front(trigF, echoF, 20)
    , m_rear(trigR, echoR, 20)
{
    m_next_trigger_ms = now_ms() + 200;
}

bool CUltrasonic::kl_ultra_enabled() const
{
    // IMPORTANT:
    // - If you use ONLY KL=15, change this to: (uint8_globalsV_value_of_kl == 15)
    return (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30);
}

bool CUltrasonic::rear_publish_allowed() const
{
    // Rear only for parking
    return m_rear_enabled && (uint8_globalsV_value_of_kl == 30);
}

void CUltrasonic::publish_rear_mm(int mm)
{
    char buf[64];
    if (mm < 0) {
        snprintf(buf, sizeof(buf), "@usRear:timeout;;\r\n");
    } else {
        snprintf(buf, sizeof(buf), "@usRear:%d;;\r\n", mm);
    }
    m_serial.write(buf, strlen(buf));
}

void CUltrasonic::publish_front_mm(int mm)
{
    // Optional debug; comment out if you don't want spam
    char buf[64];
    if (mm < 0) snprintf(buf, sizeof(buf), "@usFront:timeout;;\r\n");
    else        snprintf(buf, sizeof(buf), "@usFront:%d;;\r\n", mm);
    m_serial.write(buf, strlen(buf));
}

void CUltrasonic::serialCallbackUSFrontOverride(const char* msg, char* resp)
{
    int v = 0;
    (void)sscanf(msg, "%d", &v);

    m_front_override = (v == 1);

    sprintf(resp, "%d", m_front_override ? 1 : 0);
}

void CUltrasonic::serialCallbackPARKcommand(const char* msg, char* resp)
{
    int v = 0;
    (void)sscanf(msg, "%d", &v);

    m_rear_enabled = (v == 1);

    // Optional: if disabling, clear last measurement state
    // (not required, but keeps things clean)
    if (!m_rear_enabled) {
        // nothing special needed; we just stop triggering
    }

    sprintf(resp, "%d", m_rear_enabled ? 1 : 0);
}

void CUltrasonic::_run()
{
    if (!kl_ultra_enabled()) {
        m_front_stop_req = false;
        m_front_override = false;
        m_rsm.setSafetyStop(false);
        return;
    }

    // timeout polling (non-blocking)
    m_front.poll_timeout();
    m_rear.poll_timeout();

    // trigger scheduling (alternate to avoid cross-talk)
    uint32_t t = now_ms();
    if ((int32_t)(t - m_next_trigger_ms) >= 0) {

        // Don't start a new measurement while any is in progress
        if (!m_front.measuring() && !m_rear.measuring()) {

            if (m_phase == TRIG_FRONT) {
                m_front.trigger();

                // If rear is enabled, next phase is rear; otherwise keep doing front only
                m_phase = m_rear_enabled ? TRIG_REAR : TRIG_FRONT;

            } else { // TRIG_REAR
                // Only trigger rear if parking enabled (and optionally KL=15)
                if (rear_publish_allowed()) {
                    m_rear.trigger();
                }
                m_phase = TRIG_FRONT;
            }

            m_next_trigger_ms = t + m_gap_ms;
        }
    }

    // FRONT result
    if (m_front.has_new()) {
        int mm = m_front.distance_mm();
        m_front.clear_new();

        // publish_front_mm(mm); // enable if you want to see it live

        // Hysteresis stop request
        if (mm > 0) {
            if (!m_front_stop_req && mm <= m_front_brake_mm) m_front_stop_req = true;
            if ( m_front_stop_req && mm >= m_front_release_mm) m_front_stop_req = false;
        } else {
            // timeout -> treat as "no obstacle"
            m_front_stop_req = false;
        }
    }

    // Apply front safety stop (unless overridden)
    if (m_front_override) {
        m_rsm.setSafetyStop(false);
    } else {
        m_rsm.setSafetyStop(m_front_stop_req, 0);
    }

    // REAR result (publish only when parking enabled)
    if (m_rear.has_new()) {
        int mm = m_rear.distance_mm();
        m_rear.clear_new();

        if (rear_publish_allowed()) {
            publish_rear_mm(mm);
        }
    }
}

} // namespace periodics
