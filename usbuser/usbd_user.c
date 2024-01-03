

#include "main.h"

#include "usbd_core.h"
#include "cdc_uart.h"

#include "DAP_config.h"
#include "DAP.h"

/******************************************************************************/


#define USBD_VID           0x0d28
#define USBD_PID           0x0204
#define USBD_MAX_POWER     500


#define DAP_IN_EP  0x81
#define DAP_OUT_EP 0x01
#define DAP_INTF   0
#define CMSIS_DAP_INTERFACE_SIZE (9 + 7 + 7)


#define CDC0_IN_EP  0x82
#define CDC0_OUT_EP 0x02
#define CDC0_INT_EP 0x83
#define CDC0_INTF   1


#define CDC1_IN_EP  0x84
#define CDC1_OUT_EP 0x04
#define CDC1_INT_EP 0x85
#define CDC1_INTF   3


#define USB_CONFIG_SIZE (9 + CMSIS_DAP_INTERFACE_SIZE + CDC_ACM_DESCRIPTOR_LEN*2)
#define INTF_NUM        5

/*!< global descriptor */
static const uint8_t cmsisdap_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xef, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
	USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 1, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

 	/* Interface 0 : DAPLINK */
	USB_INTERFACE_DESCRIPTOR_INIT(DAP_INTF, 0, 2, 0xFF, 0x00, 0x00, 2),
	USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP,  USB_ENDPOINT_TYPE_BULK, USB_PACKET_SIZE, 0x00),

	/* Interface 1,2 : CDC0 */
	CDC_ACM_DESCRIPTOR_INIT(CDC0_INTF, CDC0_INT_EP, CDC0_OUT_EP, CDC0_IN_EP, USB_PACKET_SIZE, 4),
 	/* Interface 3,4 : CDC1 */
	CDC_ACM_DESCRIPTOR_INIT(CDC1_INTF, CDC1_INT_EP, CDC1_OUT_EP, CDC1_IN_EP, USB_PACKET_SIZE, 5),

#ifdef CONFIG_USB_HS
    /* device qualifier descriptor */
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
#endif
    0x00
};


static const struct usb_string_descriptor str0 = USB_ARRAY_DESC(0x0409);
static const struct usb_string_descriptor str1 = USB_STRING_DESC("CherryUSB");
static const struct usb_string_descriptor str2 = USB_STRING_DESC("T-LINK CMSIS-DAP");
static const struct usb_string_descriptor str3 = USB_STRING_DESC("2023121800");
static const struct usb_string_descriptor str4 = USB_STRING_DESC("T-LINK UART1");
static const struct usb_string_descriptor str5 = USB_STRING_DESC("T-LINK UART2");

static uint8_t *string_desc[] = {
	(uint8_t*)&str0,
	(uint8_t*)&str1,
	(uint8_t*)&str2,
	(uint8_t*)&str3,
	(uint8_t*)&str4,
	(uint8_t*)&str5,
};
static int string_cnt = sizeof(string_desc)/sizeof(uint8_t*);


/******************************************************************************/

#define USBD_WINUSB_VENDOR_CODE 0x20

/* WinUSB Microsoft OS 2.0 descriptor sizes */
#define WINUSB_DESCRIPTOR_SET_HEADER_SIZE  10
#define WINUSB_FUNCTION_SUBSET_HEADER_SIZE 8
#define WINUSB_FEATURE_COMPATIBLE_ID_SIZE  20

#define FUNCTION_SUBSET_LEN                160
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132

#define USBD_WINUSB_DESC_SET_LEN (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + FUNCTION_SUBSET_LEN)

__ALIGN_BEGIN const uint8_t WinUSB_desc[] = {
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE), /* wDescriptorType */
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */  /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),          /* wDescriptorSetTotalLength */

    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), /* wDescriptorType */
    0,                                         /* bFirstInterface USBD_BULK_IF_NUM*/
    0,                                         /* bReserved */
    WBVAL(FUNCTION_SUBSET_LEN),                /* wSubsetLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  /* wLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  /* wDescriptorType */
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        /* CompatibleId*/
    0, 0, 0, 0, 0, 0, 0, 0,                    /* SubCompatibleId*/
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), /* wLength */
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   /* wDescriptorType */
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), /* wPropertyDataType */
    WBVAL(42),                                 /* wPropertyNameLength */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), /* wPropertyDataLength */
    '{', 0,
    'C', 0, 'D', 0, 'B', 0, '3', 0, 'B', 0, '5', 0, 'A', 0, 'D', 0, '-', 0,
    '2', 0, '9', 0, '3', 0, 'B', 0, '-', 0,
    '4', 0, '6', 0, '6', 0, '3', 0, '-', 0,
    'A', 0, 'A', 0, '3', 0, '6', 0, '-',
    0, '1', 0, 'A', 0, 'A', 0, 'E', 0, '4', 0, '6', 0, '4', 0, '6', 0, '3', 0, '7', 0, '7', 0, '6', 0,
    '}', 0, 0, 0, 0, 0
};

#define USBD_NUM_DEV_CAPABILITIES (1)
#define USBD_WINUSB_DESC_LEN 28
#define USBD_BOS_WTOTALLENGTH (5 + USBD_WINUSB_DESC_LEN)

__ALIGN_BEGIN const uint8_t BOS_desc[] = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    USBD_NUM_DEV_CAPABILITIES,    /* bNumDeviceCaps */

    USBD_WINUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,         /* PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D,
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */ /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),         /* wDescriptorSetTotalLength */
    USBD_WINUSB_VENDOR_CODE,                 /* bVendorCode */
    0,                                       /* bAltEnumCode */
};


struct usb_msosv2_descriptor msosv2_desc = {
	.vendor_code = USBD_WINUSB_VENDOR_CODE,
	.compat_id = (uint8_t*)WinUSB_desc,
	.compat_id_len = USBD_WINUSB_DESC_SET_LEN,
};

struct usb_bos_descriptor bos_desc = {
	.string = (uint8_t*)BOS_desc,
	.string_len = USBD_BOS_WTOTALLENGTH
};


static void dap_out_callback(uint8_t ep, uint32_t nbytes);
static void dap_in_callback(uint8_t ep, uint32_t nbytes);

static struct usbd_endpoint dap_out_ep = {
	.ep_addr = DAP_OUT_EP,
	.ep_cb = dap_out_callback
};

static struct usbd_endpoint dap_in_ep = {
	.ep_addr = DAP_IN_EP,
	.ep_cb = dap_in_callback
};


struct usbd_interface dap_intf;


/******************************************************************************/


static volatile int USB_RequestIndexI; // Request  Index In
static volatile int USB_RequestIndexO; // Request  Index Out
static volatile int USB_RequestCountI; // Request  Count In
static volatile int USB_RequestCountO; // Request  Count Out
static volatile int USB_RequestIdle;    // Request  Idle  Flag

static volatile int USB_ResponseIndexI; // Response Index In
static volatile int USB_ResponseIndexO; // Response Index Out
static volatile int USB_ResponseCountI; // Response Count In
static volatile int USB_ResponseCountO; // Response Count Out
static volatile int USB_ResponseIdle;    // Response Idle  Flag

static uint8_t USB_Request[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
static uint8_t USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE]; // Response Buffer
static int USB_RespSize[DAP_PACKET_COUNT];                      // Response Size


static void dap_out_callback(uint8_t ep, uint32_t nbytes)
{
//	printk("dap_out: %3d  %02x %02x\n", nbytes,
//			USB_Request[USB_RequestIndexI][0], USB_Request[USB_RequestIndexI][1]);

	if (USB_Request[USB_RequestIndexI][0] == ID_DAP_TransferAbort) {
		DAP_TransferAbort = 1U;
	} else {
		USB_RequestIndexI++;
		if (USB_RequestIndexI == DAP_PACKET_COUNT) {
			USB_RequestIndexI = 0U;
		}
		USB_RequestCountI++;
	}

	// Start reception of next request packet
	if ((uint16_t)(USB_RequestCountI - USB_RequestCountO) != DAP_PACKET_COUNT) {
		usbd_ep_start_read(DAP_OUT_EP, USB_Request[USB_RequestIndexI], DAP_PACKET_SIZE);
	} else {
		USB_RequestIdle = 1U;
	}
}


static void dap_in_callback(uint8_t ep, uint32_t nbytes)
{
	if (USB_ResponseCountI != USB_ResponseCountO) {
		// Load data from response buffer to be sent back
		usbd_ep_start_write(DAP_IN_EP, USB_Response[USB_ResponseIndexO], USB_RespSize[USB_ResponseIndexO]);
		USB_ResponseIndexO++;
		if (USB_ResponseIndexO == DAP_PACKET_COUNT) {
			USB_ResponseIndexO = 0U;
		}
		USB_ResponseCountO++;
	} else {
		USB_ResponseIdle = 1U;
    }
}


static void dap_out_start(void)
{
	usbd_ep_start_read(DAP_OUT_EP, USB_Request[USB_RequestIndexI], DAP_PACKET_SIZE);
}


static void dap_init(void)
{
	DAP_Setup();

	USB_RequestIndexI = 0;
	USB_RequestIndexO = 0;
	USB_RequestCountI = 0;
	USB_RequestCountO = 0;
	USB_RequestIdle   = 1;
	USB_ResponseIndexI = 0;
	USB_ResponseIndexO = 0;
	USB_ResponseCountI = 0;
	USB_ResponseCountO = 0;
	USB_ResponseIdle   = 1;
}


void dap_handle(void)
{
    uint32_t n;

    // Process pending requests
    while (USB_RequestCountI != USB_RequestCountO) {
        // Handle Queue Commands
        n = USB_RequestIndexO;
        while (USB_Request[n][0] == ID_DAP_QueueCommands) {
            USB_Request[n][0] = ID_DAP_ExecuteCommands;
            n++;
            if (n == DAP_PACKET_COUNT) {
                n = 0U;
            }
            if (n == USB_RequestIndexI) {
                // flags = osThreadFlagsWait(0x81U, osFlagsWaitAny, osWaitForever);
                // if (flags & 0x80U) {
                //     break;
                // }
            }
        }

        // Execute DAP Command (process request and prepare response)
        USB_RespSize[USB_ResponseIndexI] =
            (uint16_t)DAP_ExecuteCommand(USB_Request[USB_RequestIndexO], USB_Response[USB_ResponseIndexI]);

        // Update Request Index and Count
        USB_RequestIndexO++;
        if (USB_RequestIndexO == DAP_PACKET_COUNT) {
            USB_RequestIndexO = 0U;
        }
        USB_RequestCountO++;

        if (USB_RequestIdle) {
            if ((uint16_t)(USB_RequestCountI - USB_RequestCountO) != DAP_PACKET_COUNT) {
                USB_RequestIdle = 0U;
                usbd_ep_start_read(DAP_OUT_EP, USB_Request[USB_RequestIndexI], DAP_PACKET_SIZE);
            }
        }

        // Update Response Index and Count
        USB_ResponseIndexI++;
        if (USB_ResponseIndexI == DAP_PACKET_COUNT) {
            USB_ResponseIndexI = 0U;
        }
        USB_ResponseCountI++;

        if (USB_ResponseIdle) {
            if (USB_ResponseCountI != USB_ResponseCountO) {
                // Load data from response buffer to be sent back
                n = USB_ResponseIndexO++;
                if (USB_ResponseIndexO == DAP_PACKET_COUNT) {
                    USB_ResponseIndexO = 0U;
                }
                USB_ResponseCountO++;
                USB_ResponseIdle = 0U;
                usbd_ep_start_write(DAP_IN_EP, USB_Response[n], USB_RespSize[n]);
            }
        }
    }
}


/******************************************************************************/


static uint8_t cdc0_uart_txbuf[TXBUF_SIZE];
static uint8_t cdc0_usb_rxbuf[USB_PACKET_SIZE];
static uint8_t cdc0_uart_rxbuf[RXBUF_SIZE];
static uint8_t cdc0_usb_txbuf[TXBUF_SIZE];

static uint8_t cdc1_uart_txbuf[TXBUF_SIZE];
static uint8_t cdc1_usb_rxbuf[USB_PACKET_SIZE];
static uint8_t cdc1_uart_rxbuf[RXBUF_SIZE];
static uint8_t cdc1_usb_txbuf[TXBUF_SIZE];


CDC_UART cdc_uarts[2] = {
	{
		.uart = UART3,
		.pclk = APB1CLK_FREQ,
		.dma  = DMA1,
		.txch = 1,
		.txdma = &DMA1->CH[1],
		.rxch = 2,
		.rxdma = &DMA1->CH[2],
		.intf_num = CDC0_INTF,
		.out_ep = {
			.ep_addr = CDC0_OUT_EP,
		},
		.in_ep = {
			.ep_addr = CDC0_IN_EP,
		},
		.usb_rxbuf  = cdc0_usb_rxbuf,
		.uart_txbuf = cdc0_uart_txbuf,
		.usb_txbuf  = cdc0_usb_txbuf,
		.uart_rxbuf = cdc0_uart_rxbuf,
	},

	{
		.uart = UART1,
		.pclk = APB2CLK_FREQ,
		.dma  = DMA1,
		.txch = 3,
		.txdma = &DMA1->CH[3],
		.rxch = 4,
		.rxdma = &DMA1->CH[4],
		.intf_num = CDC1_INTF,
		.out_ep = {
			.ep_addr = CDC1_OUT_EP,
		},
		.in_ep = {
			.ep_addr = CDC1_IN_EP,
		},
		.usb_rxbuf  = cdc1_usb_rxbuf,
		.uart_txbuf = cdc1_uart_txbuf,
		.usb_txbuf  = cdc1_usb_txbuf,
		.uart_rxbuf = cdc1_uart_rxbuf,
	},
};


CDC_UART *get_cdcuart(int ep, int intf)
{
	if(ep){
		if(ep==cdc_uarts[0].out_ep.ep_addr || ep==cdc_uarts[0].in_ep.ep_addr){
			return &cdc_uarts[0];
		}
		if(ep==cdc_uarts[1].out_ep.ep_addr || ep==cdc_uarts[1].in_ep.ep_addr){
			return &cdc_uarts[1];
		}
	}

	if(intf){
		if(intf==cdc_uarts[0].intf_num || intf==cdc_uarts[0].intf_num+1){
			return &cdc_uarts[0];
		}
		if(intf==cdc_uarts[1].intf_num || intf==cdc_uarts[1].intf_num+1){
			return &cdc_uarts[1];
		}
	}

	return NULL;
}


void dma1_channel3_irqhandler(void)
{
	// UART3.RXDMA
	dma_irq_handle(&cdc_uarts[0]);
}


void dma1_channel5_irqhandler(void)
{
	// UART1.RXDMA
	dma_irq_handle(&cdc_uarts[1]);
}


void usart3_irqhandler(void)
{
	uart_irq_handle(&cdc_uarts[0]);
}


void usart1_irqhandler(void)
{
	uart_irq_handle(&cdc_uarts[1]);
}

/******************************************************************************/


void usb_dc_low_level_init(void)
{
    printk("usb_dc_low_level_init!\n");
	// b[27] USBPLL_SRC = HSE
	// b[26:24] USBHS_DIV = Div4
	// b[28:29] USBHSCLK = 4MHz(16/4)
	// b[31] USBHS_SRC = USB_PHY
	RCC->CFGR2 = 0;
	RCC->CFGR2 = 0x93000000;
	RCC->CFGR2 = 0xd3000000;
	RCC->AHBPCENR |= 0x0800;

	int_enable(USBHS_IRQn);
}


void usbd_event_handler(uint8_t event)
{
	USB_LOG_RAW("USBD Event: %d\n", event);
	switch (event) {
	case USBD_EVENT_RESET:
		cdcuart_reset(&cdc_uarts[0]);
		cdcuart_reset(&cdc_uarts[1]);
		break;
	case USBD_EVENT_CONNECTED:
		break;
	case USBD_EVENT_DISCONNECTED:
		break;
	case USBD_EVENT_RESUME:
		break;
	case USBD_EVENT_SUSPEND:
		break;
	case USBD_EVENT_CONFIGURED:
		cdc_recv_start(&cdc_uarts[0]);
		cdc_recv_start(&cdc_uarts[1]);
		dap_out_start();
		break;
	case USBD_EVENT_SET_REMOTE_WAKEUP:
		break;
	case USBD_EVENT_CLR_REMOTE_WAKEUP:
		break;
	default:
		break;
	}
}


void usb_dc_user_init(void)
{
	dap_init();

	usbd_desc_register(cmsisdap_descriptor, string_desc, string_cnt);
	usbd_bos_desc_register(&bos_desc);
	usbd_msosv2_desc_register(&msosv2_desc);

	usbd_add_interface(&dap_intf);
	usbd_add_endpoint(&dap_out_ep);
	usbd_add_endpoint(&dap_in_ep);

	cdcuart_init(&cdc_uarts[0]);
	int_enable(DMA1_Channel3_IRQn);
	int_enable(USART3_IRQn);

	cdcuart_init(&cdc_uarts[1]);
	int_enable(DMA1_Channel5_IRQn);
	int_enable(USART1_IRQn);

	usbd_initialize();
}


