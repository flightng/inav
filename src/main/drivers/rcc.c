#include "build/debug.h"
#include "platform.h"
#include "rcc.h"

#define RCC_BIT_CMD(ptr, mask, state)       do { if (state != DISABLE) { ptr |= mask; } else { ptr &= ~mask; } } while(0)


void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
//ahben1 crm
//ahben2
//apb1en
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(AT32F4) 
    case RCC_AHB1:
        RCC_BIT_CMD(RCC->AHB1ENR, mask, NewState);
        break;

    case RCC_AHB2:
        RCC_BIT_CMD(RCC->AHB2ENR, mask, NewState);
        break;
#endif

#if defined(STM32H7)
    case RCC_AHB3:
        RCC_BIT_CMD(RCC->AHB3ENR, mask, NewState);
        break;

    case RCC_AHB4:
        RCC_BIT_CMD(RCC->AHB4ENR, mask, NewState);
        break;

    case RCC_APB1L:
        RCC_BIT_CMD(RCC->APB1LENR, mask, NewState);
        break;

    case RCC_APB1H:
        RCC_BIT_CMD(RCC->APB1HENR, mask, NewState);
        break;

    case RCC_APB3:
        RCC_BIT_CMD(RCC->APB3ENR, mask, NewState);
        break;

    case RCC_APB4:
        RCC_BIT_CMD(RCC->APB4ENR, mask, NewState);
        break;
#endif
//apb1en
//apb2en
#if !(defined(STM32H7) || defined(STM32G4))
    case RCC_APB1:
        RCC_BIT_CMD(RCC->APB1ENR, mask, NewState);
        break;
#endif

    case RCC_APB2:
        RCC_BIT_CMD(RCC->APB2ENR, mask, NewState);
        break;

    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
//ahbrst1 apb1rst 
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    case RCC_AHB1:
        RCC_BIT_CMD(RCC->AHB1RSTR, mask, NewState);
        break;

    case RCC_AHB2:
        RCC_BIT_CMD(RCC->AHB2RSTR, mask, NewState);
        break;
#endif

#if defined(STM32H7)
    case RCC_AHB3:
        RCC_BIT_CMD(RCC->AHB3RSTR, mask, NewState);
        break;

    case RCC_AHB4:
        RCC_BIT_CMD(RCC->AHB4RSTR, mask, NewState);
        break;

    case RCC_APB1L:
        RCC_BIT_CMD(RCC->APB1LRSTR, mask, NewState);
        break;

    case RCC_APB1H:
        RCC_BIT_CMD(RCC->APB1HRSTR, mask, NewState);
        break;

    case RCC_APB3:
        RCC_BIT_CMD(RCC->APB3RSTR, mask, NewState);
        break;

    case RCC_APB4:
        RCC_BIT_CMD(RCC->APB4RSTR, mask, NewState);
        break;
#endif

#if !(defined(STM32H7) || defined(STM32G4))
    case RCC_APB1:
        RCC_BIT_CMD(RCC->APB1RSTR, mask, NewState);
        break;
#endif
//  crm_periph_reset(CRM_ADC_PERIPH_RESET, TRUE);
<<<<<<< HEAD

=======
>>>>>>> 初始化At32

   
    case RCC_APB2:
        RCC_BIT_CMD(RCC->APB2RSTR, mask, NewState);
        break;
    }
}
