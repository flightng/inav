/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "resource.h"

struct dmaChannelDescriptor_s;

typedef uint32_t dmaTag_t;                          // Packed DMA adapter/channel/stream
typedef struct dmaChannelDescriptor_s * DMA_t;

#if defined(UNIT_TEST)
typedef uint32_t DMA_TypeDef;
#endif

#define DMA_TAG(dma, stream, channel)   ( (((dma) & 0x03) << 12) | (((stream) & 0x0F) << 8) | (((channel) & 0xFF) << 0) )
#define DMA_NONE                        (0)

#define DMATAG_GET_DMA(x)               ( ((x) >> 12) & 0x03 )
#define DMATAG_GET_STREAM(x)            ( ((x) >> 8)  & 0x0F )
#define DMATAG_GET_CHANNEL(x)           ( ((x) >> 0)  & 0xFF )

typedef void (*dmaCallbackHandlerFuncPtr)(DMA_t channelDescriptor);

#if defined(AT32F43x)
    
    //#define EDMA_IT_TCIF                         ((uint32_t)0x00000010) /* edma full data transfer intterrupt EDMA_FDT_INT*/
    //#define EDMA_IT_HTIF                         ((uint32_t)0x00000008) /* edma half data transfer intterrupt EDMA_HDT_INT*/
    //#define EDMA_IT_TEIF                         ((uint32_t)0x00000004) /* edma data transfer error intterrupt EDMA_DTERR_INT*/
    //#define EDMA_IT_DMEIF                        ((uint32_t)0x00000002) /* edma direct mode error intterrupt EDMA_DMERR_INT*/
    //#define EDMA_IT_FEIF                         ((uint32_t)0x00000080) /* edma fifo error interrupt EDMA_FERR_INT*/

    #define DMA_IT_TCIF                         ((uint32_t)0x00000002) /*!< dma full data transfer interrupt */
    #define DMA_IT_HTIF                         ((uint32_t)0x00000004) /*!< dma half data transfer interrupt */
    #define DMA_IT_DMEIF                        ((uint32_t)0x00000008) /*!< dma errorr interrupt */

    // dma_type edma
    typedef struct dmaChannelDescriptor_s {
        dmaTag_t                    tag;
        dma_type*                   dma;
        dma_channel_type*           ref;
        // edma_type*                  edma;
        // edma_stream_type*           sref;
        dmaCallbackHandlerFuncPtr   irqHandlerCallback;
        uint32_t                    flagsShift;
        IRQn_Type                   irqNumber;
        uint32_t                    userParam;
        resourceOwner_e             owner;
        uint8_t                     resourceIndex;
    } dmaChannelDescriptor_t;

#else
    #define DMA_IT_TCIF                         ((uint32_t)0x00000020)
    #define DMA_IT_HTIF                         ((uint32_t)0x00000010)
    #define DMA_IT_TEIF                         ((uint32_t)0x00000008)
    #define DMA_IT_DMEIF                        ((uint32_t)0x00000004)
    #define DMA_IT_FEIF                         ((uint32_t)0x00000001)


    typedef struct dmaChannelDescriptor_s {
        dmaTag_t                    tag;
        DMA_TypeDef*                dma;
    #if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
        DMA_Stream_TypeDef*         ref;
    #else
        DMA_Channel_TypeDef*        ref;
    #endif
        dmaCallbackHandlerFuncPtr   irqHandlerCallback;
        uint32_t                    flagsShift;
        IRQn_Type                   irqNumber;
        uint32_t                    userParam;
        resourceOwner_e             owner;
        uint8_t                     resourceIndex;
    } dmaChannelDescriptor_t;

#endif


#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)

#define DEFINE_DMA_CHANNEL(d, s, f) { \
                                        .tag = DMA_TAG(d, s, 0), \
                                        .dma = DMA##d, \
                                        .ref = DMA##d##_Stream##s, \
                                        .irqHandlerCallback = NULL, \
                                        .flagsShift = f, \
                                        .irqNumber = DMA##d##_Stream##s##_IRQn, \
                                        .userParam = 0 \
                                    }

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                if (dmaDescriptors[i].irqHandlerCallback)\
                                                                    dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))



DMA_t dmaGetByRef(const DMA_Stream_TypeDef * ref);

#elif defined(AT32F43x)
// todo   DMA_TAG(d, s, 0) =>  DMA_TAG(d, s, s)
#define DEFINE_DMA_CHANNEL(d, s, f) { \
                                        .tag = DMA_TAG(d, s, 0), \
                                        .dma = DMA##d, \
                                        .ref = DMA##d##_CHANNEL##s, \
                                        .irqHandlerCallback = NULL, \
                                        .flagsShift = f, \
                                        .irqNumber = DMA##d##_CHANNEL##s##_IRQn, \
                                        .userParam = 0 \
                                    }
/* 
EDMA
#define DEFINE_EDMA_CHANNEL(d, s, f) { \
                                        .tag = DMA_TAG(d, s, 0), \
                                        .edma = EDMA, \
                                        .sref = EDMA##_STREAM##s, \
                                        .irqHandlerCallback = NULL, \
                                        .flagsShift = f, \
                                        .irqNumber = EDMA##_Stream##s##_IRQn, \
                                        .userParam = 0 \
                                    } 
*/

//DMA1_Channel1_IRQHandler
#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Channel ## s ## _IRQHandler(void) {\
                                                                if (dmaDescriptors[i].irqHandlerCallback)\
                                                                    dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
                                                            }
//todo HIFCR LIFCR
//dma_flag_clear dma_flag_get
#define DMA_CLEAR_FLAG(d, flag) d->dma->clr = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->sts & (flag << d->flagsShift))

DMA_t dmaGetByRef(const dma_channel_type * ref);


//EDMA_Stream1_IRQHandler
/*
#define DEFINE_EDMA_IRQ_HANDLER(d, s, i) void EDMA ## _Stream ## s ## _IRQHandler(void) {\
                                                                if (dmaDescriptors[i].irqHandlerCallback)\
                                                                    dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
                                                            }

#define EDMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->edma->clr2 = (flag << (d->flagsShift - 32)); else d->edma->clr1 = (flag << d->flagsShift)
#define EDMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->edma->sts2 & (flag << (d->flagsShift - 32)): d->edma->sts1 & (flag << d->flagsShift))
 
DMA_t dmaGetByRef(const edma_stream_type * ref);
*/

#endif

DMA_t dmaGetByTag(dmaTag_t tag);
uint32_t dmaGetChannelByTag(dmaTag_t tag);
resourceOwner_e dmaGetOwner(DMA_t dma);
void dmaInit(DMA_t dma, resourceOwner_e owner, uint8_t resourceIndex);
void dmaEnableClock(DMA_t dma);
void dmaSetHandler(DMA_t dma, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);
void dmaCleanInterrupts(DMA_t dma);
