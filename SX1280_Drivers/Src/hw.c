#include "hw.h"


extern void Error_Handler( void );


void HwSetLowPower( void )
{
    SpiDeInit( );
    
    /* Enter Stop Mode */
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFE );
}


/**
  * @brief Provide accurate delay (in milliseconds) based on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */

