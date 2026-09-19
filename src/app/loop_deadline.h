#pragma once
#include <stdint.h>

/* Keep the original clock phase after a short overrun. Drop only whole
 * missed slots, so recovery never runs a backlog of stale control cycles.
 * All timer comparisons must be within half a 32-bit wrap. */
static inline uint32_t loop_deadline_recover(uint32_t deadline, uint32_t now,
                                             uint32_t period,
                                             uint32_t *missed_slots)
{
    const int32_t late = (int32_t)(now - deadline);
    if (late >= 0 && (uint32_t)late >= period) {
        const uint32_t skipped = (uint32_t)late / period;
        *missed_slots += skipped;
        deadline += skipped * period;
    }
    return deadline;
}

/* Fractional microsecond periods: 16 kHz alternates 62 and 63 us exactly.
 * Advancing multiple slots is needed only after a genuine missed deadline. */
static inline uint32_t loop_period_advance(uint32_t hz, uint32_t slots,
                                           uint32_t *remainder)
{
    const uint64_t numerator = (uint64_t)slots * 1000000U + *remainder;
    *remainder = (uint32_t)(numerator % hz);
    return (uint32_t)(numerator / hz);
}
