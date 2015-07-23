#pragma once

#include <stdint.h>

// Device -> host
#define USB_IN 0x80
// Host -> device
#define USB_OUT 0

#define USB_REQTYPE_DIRECTION_MASK 0x80
#define USB_REQTYPE_TYPE_MASK 0x60
#define USB_REQTYPE_RECIPIENT_MASK 0x1F

#define USB_REQTYPE_STANDARD (0 << 5)
#define USB_REQTYPE_CLASS (1 << 5)
#define USB_REQTYPE_VENDOR (2 << 5)

#define USB_RECIPIENT_DEVICE (0 << 0)
#define USB_RECIPIENT_INTERFACE (1 << 0)
#define USB_RECIPIENT_ENDPOINT (2 << 0)
#define USB_RECIPIENT_OTHER (3 << 0)

typedef struct  {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__ ((packed)) USB_SetupPacket;

// Standard requests
enum {
	USB_REQ_GetStatus = 0,
	USB_REQ_ClearFeature = 1,
	USB_REQ_SetFeature = 3,
	USB_REQ_SetAddress = 5,
	USB_REQ_GetDescriptor = 6,
	USB_REQ_SetDescriptor = 7,
	USB_REQ_GetConfiguration = 8,
	USB_REQ_SetConfiguration = 9,
	USB_REQ_GetInterface = 10,
	USB_REQ_SetInterface = 11,
	USB_REQ_SynchFrame = 12,
};

enum {
	USB_FEATURE_EndpointHalt = 0x00,
	USB_FEATURE_DeviceRemoteWakeup = 0x01,
	USB_FEATURE_TestMode = 0x02,
};

enum {
	USB_DTYPE_Device = 0x01,
	USB_DTYPE_Configuration = 0x02,
	USB_DTYPE_String = 0x03,
	USB_DTYPE_Interface = 0x04,
	USB_DTYPE_Endpoint = 0x05,
	USB_DTYPE_DeviceQualifier = 0x06,
	USB_DTYPE_Other = 0x07,
	USB_DTYPE_InterfacePower = 0x08,
	USB_DTYPE_InterfaceAssociation = 0x0B,
	USB_DTYPE_CSInterface = 0x24,
	USB_DTYPE_CSEndpoint = 0x25,
} USB_dtype;

typedef enum {
	USB_CSCP_NoDeviceClass = 0x00,
	USB_CSCP_NoDeviceSubclass = 0x00,
	USB_CSCP_NoDeviceProtocol = 0x00,
	USB_CSCP_VendorSpecificClass = 0xFF,
	USB_CSCP_VendorSpecificSubclass = 0xFF,
	USB_CSCP_VendorSpecificProtocol = 0xFF,
	USB_CSCP_IADDeviceClass = 0xEF,
	USB_CSCP_IADDeviceSubclass = 0x02,
	USB_CSCP_IADDeviceProtocol = 0x01,
} USB_cscp;

#define USB_CONFIG_POWER_MA(mA) ((mA)/2)
#define USB_STRING_LEN(c) (sizeof(USB_DescriptorHeader) + ((c) * 2))

#define USB_LANGUAGE_EN_US 0x0409

#define ENDPOINT_DESCRIPTOR_DIR_IN 0x80
#define ENDPOINT_DESCRIPTOR_DIR_OUT 0x00

#define USB_CONFIG_ATTR_BUSPOWERED 0x80
#define USB_CONFIG_ATTR_SELFPOWERED 0x40
#define USB_CONFIG_ATTR_REMOTEWAKEUP 0x20

#define ENDPOINT_ATTR_NO_SYNC (0 << 2)
#define ENDPOINT_ATTR_ASYNC (1 << 2)
#define ENDPOINT_ATTR_ADAPTIVE (2 << 2)
#define ENDPOINT_ATTR_SYNC (3 << 2)

#define ENDPOINT_USAGE_DATA (0 << 4)
#define ENDPOINT_USAGE_FEEDBACK (1 << 4)
#define ENDPOINT_USAGE_IMPLICIT_FEEDBACK (2 << 4)

#define USB_EP_TYPE_CONTROL 0x00
#define USB_EP_TYPE_ISOCHRONOUS 0x01
#define USB_EP_TYPE_BULK 0x02
#define USB_EP_TYPE_INTERRUPT 0x03

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
} __attribute__ ((packed)) USB_DescriptorHeader;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} __attribute__ ((packed)) USB_DeviceDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceSubClass;
	uint8_t bMaxPacketSize0;
	uint8_t bNumConfigurations;
	uint8_t bReserved;
} __attribute__ ((packed)) USB_DeviceQualifierDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} __attribute__ ((packed)) USB_ConfigurationDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} __attribute__ ((packed)) USB_InterfaceDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
} __attribute__ ((packed)) USB_InterfaceAssociationDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} __attribute__ ((packed)) USB_EndpointDescriptor;

typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	__CHAR16_TYPE__ bString[];
} __attribute__ ((packed)) USB_StringDescriptor;

/// Microsoft WCID descriptor
typedef struct {
	uint8_t bFirstInterfaceNumber;
	uint8_t reserved1;
	uint8_t compatibleID[8];
	uint8_t subCompatibleID[8];
	uint8_t reserved2[6];
} __attribute__((packed)) USB_MicrosoftCompatibleDescriptor_Interface;

typedef struct {
	uint32_t dwLength;
	uint16_t bcdVersion;
	uint16_t wIndex;
	uint8_t bCount;
	uint8_t reserved[7];
	USB_MicrosoftCompatibleDescriptor_Interface interfaces[];
} __attribute__((packed)) USB_MicrosoftCompatibleDescriptor;
