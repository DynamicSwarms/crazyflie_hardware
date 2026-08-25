#pragma once

#include <algorithm>
#include <cstdint>

namespace libcrtp {

class TickTimer
{
public:
    explicit TickTimer(uint32_t durationMs)
        : m_durationMs(durationMs)
        , m_elapsedMs(durationMs)
    {}

    void reset()
    {
        m_elapsedMs = 0;
    }

    void reset(uint32_t durationMs)
    {
        m_durationMs = durationMs;
        m_elapsedMs = 0;
    }

    void tick(uint32_t ms)
    {
        m_elapsedMs += std::min(ms, m_durationMs - m_elapsedMs);
    }

    bool isReady() const
    {
        return m_elapsedMs == m_durationMs;
    }

private:
    uint32_t m_durationMs;
    uint32_t m_elapsedMs;
};

} // namespace libcrtp
