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

#include <stdint.h>

#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "build/atomic.h"

#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include "usb_io.h"

#include "drivers/time.h"
#include "at32f435_437_clock.h"  
#include "serial.h"
#include "serial_usb_vcp_at32f43x.h"
#include "nvic.h"
#include "at32f435_437_tmr.h"  
#include "stddef.h"

otg_core_type otg_core_struct;
#define USB_TIMEOUT  50

static vcpPort_t vcpPort;

// hw_config 涉及改动较多，这里使用二木BF的VCP代码，定时器实现

/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* enable the acc calibration ready interrupt */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    /* update the c1\c2\c3 value */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
#if (USB_ID == 0)
    acc_sof_select(ACC_SOF_OTG1);
#else
    acc_sof_select(ACC_SOF_OTG2);
#endif
    /* open acc calibration */
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  }
  else
  {
    switch(system_core_clock)
    {
      /* 48MHz */
      case 48000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1);
        break;

      /* 72MHz */
      case 72000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1_5);
        break;

      /* 96MHz */
      case 96000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2);
        break;

      /* 120MHz */
      case 120000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2_5);
        break;

      /* 144MHz */
      case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;

      /* 168MHz */
      case 168000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3_5);
        break;

      /* 192MHz */
      case 192000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4);
        break;

      /* 216MHz */
      case 216000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4_5);
        break;

      /* 240MHz */
      case 240000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5);
        break;

      /* 264MHz */
      case 264000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5_5);
        break;

      /* 288MHz */
      case 288000000:
        crm_usb_clock_div_set(CRM_USB_DIV_6);
        break;

      default:
        break;

    }
  }
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* dp and dm */
  gpio_init_struct.gpio_pins = OTG_PIN_DP | OTG_PIN_DM;
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);

  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DP_SOURCE, OTG_PIN_MUX);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DM_SOURCE, OTG_PIN_MUX);

#ifdef USB_SOF_OUTPUT_ENABLE
  crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
  gpio_init_struct.gpio_pins = OTG_PIN_SOF;
  gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_SOF_SOURCE, OTG_PIN_MUX);
#endif

  /* otgfs use vbus pin */
#ifndef USB_VBUS_IGNORE
  gpio_init_struct.gpio_pins = OTG_PIN_VBUS;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_VBUS_SOURCE, OTG_PIN_MUX);
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);
#endif


}
#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  usb low power wakeup interrupt config
  * @param  none
  * @retval none
  */
void usb_low_power_wakeup_config(void)
{
  exint_init_type exint_init_struct;

  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  exint_default_para_init(&exint_init_struct);

  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = OTG_WKUP_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  nvic_irq_enable(OTG_WKUP_IRQ, NVIC_PRIO_USB_WUP,0);
}

/**
  * @brief  this function handles otgfs wakup interrupt.
  * @param  none
  * @retval none
  */
void OTG_WKUP_HANDLER(void)
{
  exint_flag_clear(OTG_WKUP_EXINT_LINE);
}

#endif


/********************************************
 * copy from cdc part
 */

uint32_t CDC_Send_FreeBytes(void)
{
    /*
        return the bytes free in the circular buffer

        functionally equivalent to:
        (APP_Rx_ptr_out > APP_Rx_ptr_in ? APP_Rx_ptr_out - APP_Rx_ptr_in : APP_RX_DATA_SIZE - APP_Rx_ptr_in + APP_Rx_ptr_in)
        but without the impact of the condition check.
    */
    uint32_t freeBytes;
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        freeBytes = ((UserTxBufPtrOut - UserTxBufPtrIn) + (-((int)(UserTxBufPtrOut <= UserTxBufPtrIn)) & APP_TX_DATA_SIZE)) - 1;
    }

    return freeBytes;
}

/**
 * @brief  CDC_Send_DATA
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  ptrBuffer: Buffer of data to be sent
 * @param  sendLength: Number of data to be sent (in bytes)
 * @retval Bytes sent
 */
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0) {
            // block until there is free space in the ring buffer
            delay(1);
        }

        ATOMIC_BLOCK(NVIC_PRIO_USB) {
            UserTxBuffer[UserTxBufPtrIn] = ptrBuffer[i];
            UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
        }
    }
    return sendLength;
}

void TxTimerConfig(void){
	  /* Initialize TIMx peripheral as follow:
	       + Period = CDC_POLLING_INTERVAL*1000 - 1  every 5ms
	       + Prescaler = ((SystemCoreClock/2)/10000) - 1
	       + ClockDivision = 0
	       + Counter direction = Up
	  */
  crm_periph_clock_enable(CRM_TMR20_PERIPH_CLOCK, TRUE);
	//timer, period, perscaler
	tmr_base_init(usbTxTmr,(CDC_POLLING_INTERVAL - 1),((system_core_clock)/10000 - 1));
	//TMR_CLOCK_DIV1 = 0X00 NO DIV
	tmr_clock_source_div_set(usbTxTmr,TMR_CLOCK_DIV1);
	//COUNT UP
	tmr_cnt_dir_set(usbTxTmr,TMR_COUNT_UP);

	tmr_period_buffer_enable(usbTxTmr,TRUE);

	tmr_interrupt_enable(usbTxTmr, TMR_OVF_INT, TRUE);

	nvic_irq_enable(TMR20_OVF_IRQn,NVIC_PRIO_USB,0);

	tmr_counter_enable(usbTxTmr,TRUE);

}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void TMR20_OVF_IRQHandler()
{

    uint32_t buffsize;
    static uint32_t lastBuffsize = 0;

    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;

    if (pcdc->g_tx_completed == 1) {
        // endpoint has finished transmitting previous block
        if (lastBuffsize) {
            bool needZeroLengthPacket = lastBuffsize % 64 == 0;

            // move the ring buffer tail based on the previous succesful transmission
            UserTxBufPtrOut += lastBuffsize;
            if (UserTxBufPtrOut == APP_TX_DATA_SIZE) {
                UserTxBufPtrOut = 0;
            }
            lastBuffsize = 0;

            if (needZeroLengthPacket) {
            	usb_vcp_send_data(&otg_core_struct.dev, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], 0);
                return;
            }
        }
        if (UserTxBufPtrOut != UserTxBufPtrIn) {
            if (UserTxBufPtrOut > UserTxBufPtrIn) { /* Roll-back */
                buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
            } else {
                buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
            }
            if (buffsize > APP_TX_BLOCK_SIZE) {
                buffsize = APP_TX_BLOCK_SIZE;
            }

            uint32_t txed=usb_vcp_send_data(&otg_core_struct.dev,(uint8_t*)&UserTxBuffer[UserTxBufPtrOut], buffsize);
            if (txed==SUCCESS) {
                lastBuffsize = buffsize;
            }
        }
    }
    tmr_flag_clear(usbTxTmr,TMR_OVF_FLAG);
}

/************************************************************/

//是否插入 add
uint8_t usbIsConnected(){
	return (USB_CONN_STATE_DEFAULT !=otg_core_struct.dev.conn_state);
}

//是否配置 add
uint8_t usbIsConfigured(){
	return (USB_CONN_STATE_CONFIGURED ==otg_core_struct.dev.conn_state);
}

bool usbVcpIsConnected(const serialPort_t *instance)
{
    (void)instance;
    return usbIsConnected() && usbIsConfigured();
}

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}


static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}
//set baudrate change callback
// vcp 收到 set_line_coding 命令时，设置波特率
static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    // Register upper driver baud rate callback routine with USB driver
    CDC_SetBaudRateCb((void (*)(void *context, uint32_t baud))cb, (void *)context);
}

static void usbVcpSetMode(serialPort_t *instance, portMode_t mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);
    uint32_t available=0;

    available=APP_Rx_ptr_in-APP_Rx_ptr_out;
    if(available ==0){//是否还有没copy到缓存里的
        cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
		if(pcdc->g_rx_completed == 1){
			available=pcdc->g_rxlen;
		}
    } 
    return available;
    //return CDC_Receive_BytesAvailable();
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    // uint8_t buf[1];

    // while (true) {
    //     if (CDC_Receive_DATA(buf, 1))
    //         return buf[0];
    // }

    //检查缓存是否非空，如空，增加一次读取
   if ((APP_Rx_ptr_in==0)||(APP_Rx_ptr_out == APP_Rx_ptr_in)){
	   APP_Rx_ptr_out=0;
	   APP_Rx_ptr_in=usb_vcp_get_rxdata(&otg_core_struct.dev,APP_Rx_Buffer);// usb 每次 最大64 字节，不会溢出
	   if(APP_Rx_ptr_in==0)
	   {
		   //没有读到数据,返回一个0 ，避免返回上次的脏数据
		   return 0;
	   }
   }
   return APP_Rx_Buffer[APP_Rx_ptr_out++];
}

// 分多次发送 todo 
//写buffer数据到vpc 需要实现
//这里有bug， 调用write 的时候是写到了缓存里，如果缓存满了仍然未发送，则会内存溢出死机，比如在Cli 初始化时，设置serialWriteBufShim
//重新理解函数名， 为想serial 写入一个缓存块，直接走vcp发出去
//但是这样修改仍然有问题，在msp 的处理中，报文头、数据、crc 3个包被分3次发送，导致接收方无法一次性接收、校验
static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (!usbVcpIsConnected(instance)) {
        return;
    }

    uint32_t start = millis();
    const uint8_t *p = data;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

static bool usbVcpFlush(vcpPort_t *port)
{
    uint32_t count = port->txAt;
    port->txAt = 0;

    if (count == 0) {
        return true;
    }

    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }

    uint32_t start = millis();
    uint8_t *p = port->txBuf;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
    return count == 0;
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .isConnected = usbVcpIsConnected,
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite,
        .isIdle = NULL,
    }
};

void usbVcpInitHardware(void)
{
    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, RESOURCE_INPUT, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, RESOURCE_OUTPUT, 0);

    /* usb gpio config */
     usb_gpio_config();

   #ifdef USB_LOW_POWER_WAKUP
     usb_low_power_wakeup_config();
   #endif

     /* enable otgfs clock */
     crm_periph_clock_enable(OTG_CLOCK, TRUE);

     /* select usb 48m clcok source */
     usb_clock48m_select(USB_CLK_HEXT);

     /* enable otgfs irq,不能设置太高的优先级，会干扰spi 的dma中断通信 */
     nvic_irq_enable(OTG_IRQ,NVIC_PRIO_USB,0);

     usbGenerateDisconnectPulse();

     /* init usb */
     usbd_init(&otg_core_struct,
               USB_FULL_SPEED_CORE_ID,
               USB_ID,
               &cdc_class_handler,
               &cdc_desc_handler); 

    //CONFIG TX TIMER
    TxTimerConfig();
}

serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;
    s = &vcpPort;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
    return pcdc->linecoding.bitrate;
   // return CDC_BaudRate();
}

#endif
