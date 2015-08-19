#pragma once

#define CDC_INTERFACE_CLASS 0x02
#define CDC_INTERFACE_SUBCLASS_ACM 0x02
#define CDC_INTERFACE_CLASS_DATA 0x0A

#define CDC_SUBTYPE_HEADER 0x00
#define CDC_SUBTYPE_ACM 0x02
#define CDC_SUBTYPE_UNION 0x06

#define CDC_SEND_ENCAPSULATED_COMMAND 0x0
#define CDC_GET_ENCAPSULATED_RESPONSE 0x1
#define CDC_SET_LINE_ENCODING 0x20
#define CDC_GET_LINE_ENCODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK 0x23

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint16_t bcdCDC;
} __attribute__((packed)) CDC_FunctionalHeaderDescriptor;

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;
} __attribute__((packed)) CDC_FunctionalACMDescriptor;

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface;
} __attribute__((packed)) CDC_FunctionalUnionDescriptor;

typedef struct {
  uint32_t baud_rate;
  uint8_t char_format;
  uint8_t parity_type;
  uint8_t data_bits;
} __attribute__((packed)) CDC_LineEncoding;
