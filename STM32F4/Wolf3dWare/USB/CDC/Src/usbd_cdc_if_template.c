/**
    ******************************************************************************
    * @file    usbd_cdc_if_template.c
    * @author  MCD Application Team
    * @version V2.3.0
    * @date    04-November-2014
    * @brief   Generic media access Layer.
    ******************************************************************************
    * @attention
    *
    * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
    *
    * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
    * You may not use this file except in compliance with the License.
    * You may obtain a copy of the License at:
    *
    *        http://www.st.com/software_license_agreement_liberty_v2
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    *
    ******************************************************************************
    */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if_template.h"
#include "RingBuffer.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
    * @{
    */


/** @defgroup USBD_CDC
    * @brief usbd core module
    * @{
    */

/** @defgroup USBD_CDC_Private_TypesDefinitions
    * @{
    */
/**
    * @}
    */


/** @defgroup USBD_CDC_Private_Defines
    * @{
    */
/**
    * @}
    */


/** @defgroup USBD_CDC_Private_Macros
    * @{
    */

/**
    * @}
    */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
    * @{
    */

static int8_t TEMPLATE_Init     (void);
static int8_t TEMPLATE_DeInit   (void);
static int8_t TEMPLATE_Control  (uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t TEMPLATE_Receive  (uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_Template_fops = {
    TEMPLATE_Init,
    TEMPLATE_DeInit,
    TEMPLATE_Control,
    TEMPLATE_Receive
};

USBD_CDC_LineCodingTypeDef linecoding = {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
};


extern USBD_HandleTypeDef USBD_Device;
extern void setCDCEventFromISR();

static uint8_t rx_buffer[CDC_DATA_FS_OUT_PACKET_SIZE*2];
static uint8_t overflow_buffer[CDC_DATA_FS_OUT_PACKET_SIZE];
static uint16_t overflow_buffer_size;
static RingBuffer_t *ring_buffer;

char g_VCPInitialized;

void SetupVCP()
{
	ring_buffer = CreateRingBuffer(CDC_DATA_FS_OUT_PACKET_SIZE*8+1);
	overflow_buffer_size= 0;
}

/* Private functions ---------------------------------------------------------*/

/**
    * @brief  TEMPLATE_Init
    *         Initializes the CDC media low layer
    * @param  None
    * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
    */
static int8_t TEMPLATE_Init(void)
{
    /*
         Add your initialization code here
    */
    USBD_CDC_SetRxBuffer(&USBD_Device, rx_buffer);
    g_VCPInitialized = 1;
    return (0);
}

/**
    * @brief  TEMPLATE_DeInit
    *         DeInitializes the CDC media low layer
    * @param  None
    * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
    */
static int8_t TEMPLATE_DeInit(void)
{
    /*
         Add your deinitialization code here
    */
    DeleteRingBuffer(ring_buffer);
    return (0);
}


/**
    * @brief  TEMPLATE_Control
    *         Manage the CDC class requests
    * @param  Cmd: Command code
    * @param  Buf: Buffer containing command data (request parameters)
    * @param  Len: Number of data to be sent (in bytes)
    * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
    */
static int8_t TEMPLATE_Control  (uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            /* Add your code here */
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            /* Add your code here */
            break;

        case CDC_SET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_GET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_CLEAR_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_SET_LINE_CODING:
            linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | \
                                               (pbuf[2] << 16) | (pbuf[3] << 24));
            linecoding.format     = pbuf[4];
            linecoding.paritytype = pbuf[5];
            linecoding.datatype   = pbuf[6];

            /* Add your code here */
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(linecoding.bitrate);
            pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
            pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
            pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
            pbuf[4] = linecoding.format;
            pbuf[5] = linecoding.paritytype;
            pbuf[6] = linecoding.datatype;

            /* Add your code here */
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            //printf("Got control line state: len %d, %p\n", length, pbuf);
            /* Add your code here */
            break;

        case CDC_SEND_BREAK:
            /* Add your code here */
            break;

        default:
            break;
    }

    return (0);
}

/**
    * @brief  TEMPLATE_Receive
    *         Data received over USB OUT endpoint are sent over CDC interface
    *         through this function.
    *
    *         @note
    *         This function will issue a NAK packet on any OUT packet received on
    *         USB endpoint untill exiting this function. If you exit this function
    *         before transfer is complete on CDC interface (ie. using DMA controller)
    *         it will result in receiving more data while previous ones are still
    *         not sent.
    *
    * @param  Buf: Buffer of data to be received
    * @param  Len: Number of data received (in bytes)
    * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
    */

static volatile bool buffer_full= false;
static int8_t TEMPLATE_Receive (uint8_t *Buf, uint32_t *Len)
{
	if(overflow_buffer_size > 0) {
		// feed overflow buffer first
		for (int i = 0; i < overflow_buffer_size; ++i) {
		    if(!RingBufferPut(ring_buffer, overflow_buffer[i])){
		    	// Now what?? FIXME this should not happen as buffer is bigger than overflow buffer
		    	break;
		    }
		}
		overflow_buffer_size= 0;
	}

	uint32_t n= *Len;
	for (int i = 0; i < n; ++i) {
		if(!RingBufferPut(ring_buffer, Buf[i])){
			// dump overflow into the overflow buffer, as we will not call USBD_CDC_ReceivePacket
			// we should not get any more packets until these have all been cleared
			overflow_buffer_size= n-i;
			memcpy(overflow_buffer, &Buf[i], overflow_buffer_size);
    		buffer_full= true;
			break;
		}
    }
    // signal thread that we have something to process
	setCDCEventFromISR();

    if (!buffer_full) {
        USBD_CDC_ReceivePacket(&USBD_Device);
    }

    return (USBD_OK);
}

/**
    * @}
    */

/**
    * @}
    */

/**
    * @}
    */

bool VCP_get(uint8_t *c)
{
	bool r= RingBufferGet(ring_buffer, c);

	if(buffer_full && RingBufferEmpty(ring_buffer)) {
		buffer_full= false;
    	// we drained the buffer, so we can start receiving packets again
    	USBD_CDC_ReceivePacket(&USBD_Device);
    }

    return r;
}

int VCP_read(void *pBuffer, int size)
{
    if (RingBufferEmpty(ring_buffer))
        return 0;

    uint8_t *p= pBuffer;
    int cnt= 0;
    while(cnt < size && !RingBufferEmpty(ring_buffer)) {
    	if(!RingBufferGet(ring_buffer, p)) break;
    	++p;
    	++cnt;
    }

    if(buffer_full && RingBufferEmpty(ring_buffer)) {
    	// we drained the buffer, so we can start recieving packets again
    	USBD_CDC_ReceivePacket(&USBD_Device);
    }

    return cnt;
}

// TODO make non-blocking and use circular queue
int VCP_write(const void *pBuffer, int size)
{
    if (size > CDC_DATA_FS_OUT_PACKET_SIZE) {
        int offset;
        for (offset = 0; offset < size; offset++) {
            int todo = MIN(CDC_DATA_FS_OUT_PACKET_SIZE,
                           size - offset);
            int done = VCP_write(((char *)pBuffer) + offset, todo);
            if (done != todo)
                return offset + done;
        }

        return size;
    }

    USBD_CDC_HandleTypeDef *pCDC =
        (USBD_CDC_HandleTypeDef *)USBD_Device.pClassData;
    while(pCDC->TxState) { } //Wait for previous transfer

    USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)pBuffer, size);
    if (USBD_CDC_TransmitPacket(&USBD_Device) != USBD_OK)
        return 0;

    while(pCDC->TxState) { } //Wait until transfer is done
    return size;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
