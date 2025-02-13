#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

// #define THERMAL_DATA_UART
// #define TMP007
// #define TMP007_OVERLAY
// #define MLX90614
// #define MLX90614_OVERLAY
// #define OVERLAY_FAHRENHEIT
// #define SPLASHSCREEN_OVERLAY
// #define OVERLAY_DEFAULT_ON
// #define Y16
// #define DEBUG_IN_USB_PRODUCT_STRING
// #define DEBUG_IN_SPLASHSCREEN_OVERLAY

// Values from LEP_PCOLOR_LUT_E in Middlewares/lepton_sdk/Inc/LEPTON_VID.h
#define PSUEDOCOLOR_LUT LEP_VID_FUSION_LUT

#ifdef USART_DEBUG // this happens by 'make USART_DEBUG=1' or when USART_DEBUG is enabled above
#undef THERMAL_DATA_UART
#define USART_DEBUG_SPEED (115200)
#else
#define USART_DEBUG_SPEED (921600)
#endif

#endif
