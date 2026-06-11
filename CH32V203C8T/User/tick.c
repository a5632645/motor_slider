#include "tick.h"
#include "ch32v20x.h"

static volatile uint32_t tick_;

/*********************************************************************
 * @fn      Tick_Init
 *
 * @brief   Resets tick counter and starts SysTick at 1 ms interval.
 *
 * @return  none
 */
void Tick_Init(void)
{
    tick_ = 0;
    SysTick->CMP = SystemCoreClock / 1000; 
    SysTick->CNT = 0;
    SysTick->CTLR = 0xf;
    SysTick->SR = 0;
    NVIC_EnableIRQ(SysTick_IRQn);
}

/*********************************************************************
 * @fn      Tick_Get
 *
 * @brief   Returns the current 1ms tick value.
 *
 * @return  tick count in milliseconds
 */
uint32_t Tick_Get(void)
{
    return tick_;
}

/*********************************************************************
 * @fn      Tick_Increment
 *
 * @brief   Called from SysTick_Handler once per 1ms.
 *
 * @return  none
 */
void Tick_Increment(void)
{
    tick_++;
}
