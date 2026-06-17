#include "tick.h"
#include "ch32v20x.h"

#define TICK_IRQ_PERIOD_MS 1000u
#define TICK_IRQ_PERIOD_US 1000000u

static volatile uint32_t tick_base_ms_;
static volatile uint32_t tick_base_us_;
static uint32_t tick_cycles_per_ms_;
static uint32_t tick_cycles_per_us_;

/*********************************************************************
 * @fn      Tick_Init
 *
 * @brief   Resets tick counter and starts SysTick at 1 second interval.
 *
 * @return  none
 */
void Tick_Init(void) {
    tick_base_ms_ = 0;
    tick_base_us_ = 0;
    tick_cycles_per_ms_ = SystemCoreClock / 1000u;
    tick_cycles_per_us_ = SystemCoreClock / 1000000u;

    SysTick->CMP = SystemCoreClock;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xf;
    SysTick->SR = 0;
    NVIC_EnableIRQ(SysTick_IRQn);
}

/*********************************************************************
 * @fn      Tick_GetMs
 *
 * @brief   Returns the current time in milliseconds.
 *
 * @return  time in milliseconds
 */
uint32_t Tick_GetMs(void) {
    uint32_t base_before;
    uint32_t base_after;
    uint32_t cnt;
    uint32_t sr;

    do {
        base_before = tick_base_ms_;
        cnt = (uint32_t)SysTick->CNT;
        sr = SysTick->SR;
        base_after = tick_base_ms_;
    } while (base_before != base_after);

    if (tick_cycles_per_ms_ == 0) {
        return base_before;
    }

    if (sr & 1u) {
        base_before += TICK_IRQ_PERIOD_MS;
    }

    return base_before + (cnt / tick_cycles_per_ms_);
}

/*********************************************************************
 * @fn      Tick_GetUs
 *
 * @brief   Returns the current time in microseconds.
 *
 * @return  time in microseconds
 */
uint32_t Tick_GetUs(void) {
    uint32_t base_before;
    uint32_t base_after;
    uint32_t sr;
    uint32_t cnt;

    do {
        base_before = tick_base_us_;
        cnt = (uint32_t)SysTick->CNT;
        sr = SysTick->SR;
        base_after = tick_base_us_;
    } while (base_before != base_after);

    if (tick_cycles_per_us_ == 0) {
        return base_before;
    }

    if (sr & 1u) {
        base_before += TICK_IRQ_PERIOD_US;
    }

    return base_before + (cnt / tick_cycles_per_us_);
}

/*********************************************************************
 * @fn      Tick_Get
 *
 * @brief   Returns the current time in milliseconds.
 *
 * @return  time in milliseconds
 */
uint32_t Tick_Get(void) {
    return Tick_GetMs();
}

/*********************************************************************
 * @fn      Tick_OnSysTick
 *
 * @brief   Called from SysTick_Handler once per tick period.
 *
 * @return  none
 */
void Tick_OnSysTick(void) {
    tick_base_ms_ += TICK_IRQ_PERIOD_MS;
    tick_base_us_ += TICK_IRQ_PERIOD_US;
}
