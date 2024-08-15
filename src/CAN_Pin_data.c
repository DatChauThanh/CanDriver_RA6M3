/* generated pin source file - do not edit */
#include "CAN_Pin_data.h"
// CAN0: Port 1, 2 ,4, 7.
// CAN1: Port 1, 5 ,6. 
// Select Correct Channel and Port 
#define CHANNEL0
#define PORT 4
const ioport_pin_cfg_t g_bsp_can_pin_cfg_data[] =
#ifdef CHANNEL0
#if PORT == 1
        {
          { .pin = BSP_IO_PORT_01_PIN_02, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_01_PIN_03, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#if PORT == 2 
        {
          { .pin = BSP_IO_PORT_02_PIN_02, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_02_PIN_03, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#if PORT == 4 
        {
          { .pin = BSP_IO_PORT_04_PIN_01, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_04_PIN_02, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#if PORT == 7
        {
          { .pin = BSP_IO_PORT_07_PIN_04, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_07_PIN_05, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#else
#if PORT == 1
        {
          { .pin = BSP_IO_PORT_01_PIN_09, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_01_PIN_10, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#if PORT == 5 
        {
          { .pin = BSP_IO_PORT_05_PIN_11, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_05_PIN_12, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#if PORT == 6 
        {
          { .pin = BSP_IO_PORT_06_PIN_09, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
          { .pin = BSP_IO_PORT_06_PIN_10, .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                  | (uint32_t) IOPORT_PERIPHERAL_CAN) },
        };
#endif
#endif
const ioport_cfg_t g_bsp_can_pin_cfg =
{ .number_of_pins = sizeof(g_bsp_can_pin_cfg_data) / sizeof(ioport_pin_cfg_t), .p_pin_cfg_data = &g_bsp_can_pin_cfg_data[0], };