#include "usb_standard.h"
#include "cdc_standard.h"
#include "usbd_conf.h"
#include "usbd_cdc_core.h"

#define INTERFACE_CDC_CONTROL 0
#define INTERFACE_CDC_DATA 1

typedef struct ConfigDesc {
  USB_ConfigurationDescriptor Config;
  
  USB_InterfaceDescriptor CDC_control_interface;
  CDC_FunctionalHeaderDescriptor CDC_functional_header;
  CDC_FunctionalACMDescriptor CDC_functional_ACM;
  CDC_FunctionalUnionDescriptor CDC_functional_union;
  USB_EndpointDescriptor CDC_notification_endpoint;

  USB_InterfaceDescriptor CDC_data_interface;
  USB_EndpointDescriptor CDC_out_endpoint;
  USB_EndpointDescriptor CDC_in_endpoint;
}  __attribute__((packed)) ConfigDesc;

const ConfigDesc configuration_descriptor = {
  .Config = {
    .bLength = sizeof(USB_ConfigurationDescriptor),
    .bDescriptorType = USB_DTYPE_Configuration,
    .wTotalLength  = sizeof(ConfigDesc),
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = USB_CONFIG_ATTR_BUSPOWERED,
    .bMaxPower = USB_CONFIG_POWER_MA(500)
  },
  .CDC_control_interface = {
    .bLength = sizeof(USB_InterfaceDescriptor),
    .bDescriptorType = USB_DTYPE_Interface,
    .bInterfaceNumber = INTERFACE_CDC_CONTROL,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = CDC_INTERFACE_CLASS,
    .bInterfaceSubClass = CDC_INTERFACE_SUBCLASS_ACM,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
  },
  .CDC_functional_header = {
    .bLength = sizeof(CDC_FunctionalHeaderDescriptor),
    .bDescriptorType = USB_DTYPE_CSInterface,
    .bDescriptorSubtype = CDC_SUBTYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .CDC_functional_ACM = {
    .bLength = sizeof(CDC_FunctionalACMDescriptor),
    .bDescriptorType = USB_DTYPE_CSInterface,
    .bDescriptorSubtype = CDC_SUBTYPE_ACM,
    .bmCapabilities = 0x00,
  },
  .CDC_functional_union = {
    .bLength = sizeof(CDC_FunctionalUnionDescriptor),
    .bDescriptorType = USB_DTYPE_CSInterface,
    .bDescriptorSubtype = CDC_SUBTYPE_UNION,
    .bMasterInterface = INTERFACE_CDC_CONTROL,
    .bSlaveInterface = INTERFACE_CDC_DATA,
  },
  .CDC_notification_endpoint = {
    .bLength = sizeof(USB_EndpointDescriptor),
    .bDescriptorType = USB_DTYPE_Endpoint,
    .bEndpointAddress = CDC_CMD_EP,
    .bmAttributes = (USB_EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .wMaxPacketSize = CDC_CMD_PACKET_SZE,
    .bInterval = 0xFF
  },
  .CDC_data_interface = {
    .bLength = sizeof(USB_InterfaceDescriptor),
    .bDescriptorType = USB_DTYPE_Interface,
    .bInterfaceNumber = INTERFACE_CDC_DATA,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = CDC_INTERFACE_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
  },
  .CDC_out_endpoint = {
    .bLength = sizeof(USB_EndpointDescriptor),
    .bDescriptorType = USB_DTYPE_Endpoint,
    .bEndpointAddress = CDC_OUT_EP,
    .bmAttributes = (USB_EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .wMaxPacketSize = CDC_DATA_MAX_PACKET_SIZE,
    .bInterval = 0x0
  },
  .CDC_in_endpoint = {
    .bLength = sizeof(USB_EndpointDescriptor),
    .bDescriptorType = USB_DTYPE_Endpoint,
    .bEndpointAddress = CDC_IN_EP,
    .bmAttributes = (USB_EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .wMaxPacketSize = CDC_DATA_MAX_PACKET_SIZE,
    .bInterval = 0x0
  },
};

static uint8_t usb_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t usb_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t usb_Setup       (void  *pdev, USB_SETUP_REQ *req);
static uint8_t usb_EP0_RxReady  (void *pdev);
static uint8_t usb_DataIn      (void *pdev, uint8_t epnum);
static uint8_t usb_DataOut     (void *pdev, uint8_t epnum);
static uint8_t usb_SOF         (void *pdev);
static uint8_t* usb_GetCfgDesc (uint8_t speed, uint16_t *length);

USBD_Class_cb_TypeDef custom_composite_cb = {
    .Init = usb_Init,
    .DeInit = usb_DeInit,
    .Setup = usb_Setup,
    .EP0_TxSent = NULL,
    .EP0_RxReady = usb_EP0_RxReady,
    .DataIn = usb_DataIn,
    .DataOut = usb_DataOut,
    .SOF = usb_SOF,
    .IsoINIncomplete = NULL,
    .IsoOUTIncomplete = NULL,
    .GetConfigDescriptor = usb_GetCfgDesc,
};

static uint8_t usb_Init        (void  *pdev, uint8_t cfgidx) {
  return USBD_CDC_cb.Init(pdev, cfgidx);
}

static uint8_t usb_DeInit      (void  *pdev, uint8_t cfgidx) {
  return USBD_CDC_cb.DeInit(pdev, cfgidx);
}

static uint8_t usb_Setup       (void  *pdev, USB_SETUP_REQ *req) {
  return USBD_CDC_cb.Setup(pdev, req);
}

static uint8_t usb_EP0_RxReady  (void *pdev) {
  return USBD_CDC_cb.EP0_RxReady(pdev);
}

static uint8_t usb_DataIn      (void *pdev, uint8_t epnum) {
  return USBD_CDC_cb.DataIn(pdev, epnum);
}

static uint8_t usb_DataOut     (void *pdev, uint8_t epnum) {
  return USBD_CDC_cb.DataOut(pdev, epnum);
}

static uint8_t usb_SOF         (void *pdev) {
  return USBD_CDC_cb.SOF(pdev);
}

static uint8_t* usb_GetCfgDesc (uint8_t speed, uint16_t *length) {
  *length = sizeof (configuration_descriptor);
  return (uint8_t*) &configuration_descriptor;
}
