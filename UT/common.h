/***********************************************************************************************************************
 * File Name    : common.h
 * Description  : Contains bit math macro and reg access.
 **********************************************************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_
#include <stdint-gcc.h>

/*=====Bit Math Macro=====*/
#define SET_BIT(REG, Pos)     ((REG) |= (1 << Pos))

#define CLEAR_BIT(REG, Pos)   ((REG) &= ~(1 << Pos))

#define READ_BIT(REG, Pos)    ((REG) & (1 << Pos))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))


#define     __IM     volatile //const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
/* =========================================================================================================================== */
/* ================                                          R_CAN0                                           ================ */
/* =========================================================================================================================== */
typedef enum IRQn
{
    Reset_IRQn            = -15,       /*  1 Reset Vector invoked on Power up and warm reset */
    NonMaskableInt_IRQn   = -14,       /*  2 Non maskable Interrupt cannot be stopped or preempted */
    HardFault_IRQn        = -13,       /*  3 Hard Fault all classes of Fault */
    MemoryManagement_IRQn = -12,       /*  4 Memory Management MPU mismatch, including Access Violation and No Match */
    BusFault_IRQn         = -11,       /*  5 Bus Fault Pre-Fetch-, Memory Access, other address/memory Fault */
    UsageFault_IRQn       = -10,       /*  6 Usage Fault i.e. Undef Instruction, Illegal State Transition */
    SecureFault_IRQn      = -9,        /*  7 Secure Fault Interrupt */
    SVCall_IRQn           = -5,        /* 11 System Service Call via SVC instruction */
    DebugMonitor_IRQn     = -4,        /* 12 Debug Monitor */
    PendSV_IRQn           = -2,        /* 14 Pendable request for system service */
    SysTick_IRQn          = -1,        /* 15 System Tick Timer */
} IRQn_Type;
/**
 * @brief R_CAN0_MB [MB] (Mailbox)
 */
typedef struct
{
    union
    {
        __IOM uint32_t ID;             /*!< (@ 0x00000000) Mailbox ID Register                                        */

        struct
        {
            __IOM uint32_t EID : 18;   /*!< [17..0] Extended ID                                                       */
            __IOM uint32_t SID : 11;   /*!< [28..18] Standard ID                                                      */
            uint32_t           : 1;
            __IOM uint32_t RTR : 1;    /*!< [30..30] Remote Transmission Request                                      */
            __IOM uint32_t IDE : 1;    /*!< [31..31] ID Extension                                                     */
        } ID_b;
    };

    union
    {
        __IOM uint16_t DL;             /*!< (@ 0x00000004) Mailbox DLC Register                                       */

        struct
        {
            __IOM uint16_t DLC : 4;    /*!< [3..0] Data Length Code                                                   */
            uint16_t           : 12;
        } DL_b;
    };

    union
    {
        __IOM uint8_t D[8];            /*!< (@ 0x00000006) Mailbox Data Register                                      */

        struct
        {
            __IOM uint8_t DATA : 8;    /*!< [7..0] DATA0 to DATA7 store the transmitted or received CAN
                                        *   message data. Transmission or reception starts from DATA0.
                                        *   The bit order on the CAN bus is MSB-first, and transmission
                                        *   or reception starts from bit 7                                            */
        } D_b[8];
    };

    union
    {
        __IOM uint16_t TS;             /*!< (@ 0x0000000E) Mailbox Timestamp Register                                 */

        struct
        {
            __IOM uint16_t TSL : 8;    /*!< [7..0] Time Stamp Higher ByteBits TSL[7:0] store the counter
                                        *   value of the time stamp when received messages are stored
                                        *   in the mailbox.                                                           */
            __IOM uint16_t TSH : 8;    /*!< [15..8] Time Stamp Lower ByteBits TSH[7:0] store the counter
                                        *   value of the time stamp when received messages are stored
                                        *   in the mailbox.                                                           */
        } TS_b;
    };
} R_CAN0_MB_Type;                      /*!< Size = 16 (0x10)       

/**
 * @brief Controller Area Network (CAN) Module (R_CAN0)
 */

typedef struct                         /*!< (@ 0x40050000) R_CAN0 Structure                                           */
{
    __IM uint32_t        RESERVED[128];
    __IOM R_CAN0_MB_Type MB[32];       /*!< (@ 0x00000200) Mailbox                                                    */

    union
    {
        __IOM uint32_t MKR[8];         /*!< (@ 0x00000400) Mask Register                                              */

        struct
        {
            __IOM uint32_t EID : 18;   /*!< [17..0] Extended ID                                                       */
            __IOM uint32_t SID : 11;   /*!< [28..18] Standard ID                                                      */
            uint32_t           : 3;
        } MKR_b[8];
    };

    union
    {
        __IOM uint32_t FIDCR[2];       /*!< (@ 0x00000420) FIFO Received ID Compare Registers                         */

        struct
        {
            __IOM uint32_t EID : 18;   /*!< [17..0] Extended ID                                                       */
            __IOM uint32_t SID : 11;   /*!< [28..18] Standard ID                                                      */
            uint32_t           : 1;
            __IOM uint32_t RTR : 1;    /*!< [30..30] Remote Transmission Request                                      */
            __IOM uint32_t IDE : 1;    /*!< [31..31] ID Extension                                                     */
        } FIDCR_b[2];
    };

    union
    {
        __IOM uint32_t MKIVLR;         /*!< (@ 0x00000428) Mask Invalid Register                                      */

        struct
        {
            __IOM uint32_t MB0  : 1;   /*!< [0..0] mailbox 0 Mask Invalid                                             */
            __IOM uint32_t MB1  : 1;   /*!< [1..1] mailbox 1 Mask Invalid                                             */
            __IOM uint32_t MB2  : 1;   /*!< [2..2] mailbox 2 Mask Invalid                                             */
            __IOM uint32_t MB3  : 1;   /*!< [3..3] mailbox 3 Mask Invalid                                             */
            __IOM uint32_t MB4  : 1;   /*!< [4..4] mailbox 4 Mask Invalid                                             */
            __IOM uint32_t MB5  : 1;   /*!< [5..5] mailbox 5 Mask Invalid                                             */
            __IOM uint32_t MB6  : 1;   /*!< [6..6] mailbox 6 Mask Invalid                                             */
            __IOM uint32_t MB7  : 1;   /*!< [7..7] mailbox 7 Mask Invalid                                             */
            __IOM uint32_t MB8  : 1;   /*!< [8..8] mailbox 8 Mask Invalid                                             */
            __IOM uint32_t MB9  : 1;   /*!< [9..9] mailbox 9 Mask Invalid                                             */
            __IOM uint32_t MB10 : 1;   /*!< [10..10] mailbox 10 Mask Invalid                                          */
            __IOM uint32_t MB11 : 1;   /*!< [11..11] mailbox 11 Mask Invalid                                          */
            __IOM uint32_t MB12 : 1;   /*!< [12..12] mailbox 12 Mask Invalid                                          */
            __IOM uint32_t MB13 : 1;   /*!< [13..13] mailbox 13 Mask Invalid                                          */
            __IOM uint32_t MB14 : 1;   /*!< [14..14] mailbox 14 Mask Invalid                                          */
            __IOM uint32_t MB15 : 1;   /*!< [15..15] mailbox 15 Mask Invalid                                          */
            __IOM uint32_t MB16 : 1;   /*!< [16..16] mailbox 16 Mask Invalid                                          */
            __IOM uint32_t MB17 : 1;   /*!< [17..17] mailbox 17 Mask Invalid                                          */
            __IOM uint32_t MB18 : 1;   /*!< [18..18] mailbox 18 Mask Invalid                                          */
            __IOM uint32_t MB19 : 1;   /*!< [19..19] mailbox 19 Mask Invalid                                          */
            __IOM uint32_t MB20 : 1;   /*!< [20..20] mailbox 20 Mask Invalid                                          */
            __IOM uint32_t MB21 : 1;   /*!< [21..21] mailbox 21 Mask Invalid                                          */
            __IOM uint32_t MB22 : 1;   /*!< [22..22] mailbox 22 Mask Invalid                                          */
            __IOM uint32_t MB23 : 1;   /*!< [23..23] mailbox 23 Mask Invalid                                          */
            __IOM uint32_t MB24 : 1;   /*!< [24..24] mailbox 24 Mask Invalid                                          */
            __IOM uint32_t MB25 : 1;   /*!< [25..25] mailbox 25 Mask Invalid                                          */
            __IOM uint32_t MB26 : 1;   /*!< [26..26] mailbox 26 Mask Invalid                                          */
            __IOM uint32_t MB27 : 1;   /*!< [27..27] mailbox 27 Mask Invalid                                          */
            __IOM uint32_t MB28 : 1;   /*!< [28..28] mailbox 28 Mask Invalid                                          */
            __IOM uint32_t MB29 : 1;   /*!< [29..29] mailbox 29 Mask Invalid                                          */
            __IOM uint32_t MB30 : 1;   /*!< [30..30] mailbox 30 Mask Invalid                                          */
            __IOM uint32_t MB31 : 1;   /*!< [31..31] mailbox 31 Mask Invalid                                          */
        } MKIVLR_b;
    };

    union
    {
        union
        {
            __IOM uint32_t MIER;         /*!< (@ 0x0000042C) Mailbox Interrupt Enable Register                          */

            struct
            {
                __IOM uint32_t MB0  : 1; /*!< [0..0] mailbox 0 Interrupt Enable                                         */
                __IOM uint32_t MB1  : 1; /*!< [1..1] mailbox 1 Interrupt Enable                                         */
                __IOM uint32_t MB2  : 1; /*!< [2..2] mailbox 2 Interrupt Enable                                         */
                __IOM uint32_t MB3  : 1; /*!< [3..3] mailbox 3 Interrupt Enable                                         */
                __IOM uint32_t MB4  : 1; /*!< [4..4] mailbox 4 Interrupt Enable                                         */
                __IOM uint32_t MB5  : 1; /*!< [5..5] mailbox 5 Interrupt Enable                                         */
                __IOM uint32_t MB6  : 1; /*!< [6..6] mailbox 6 Interrupt Enable                                         */
                __IOM uint32_t MB7  : 1; /*!< [7..7] mailbox 7 Interrupt Enable                                         */
                __IOM uint32_t MB8  : 1; /*!< [8..8] mailbox 8 Interrupt Enable                                         */
                __IOM uint32_t MB9  : 1; /*!< [9..9] mailbox 9 Interrupt Enable                                         */
                __IOM uint32_t MB10 : 1; /*!< [10..10] mailbox 10 Interrupt Enable                                      */
                __IOM uint32_t MB11 : 1; /*!< [11..11] mailbox 11 Interrupt Enable                                      */
                __IOM uint32_t MB12 : 1; /*!< [12..12] mailbox 12 Interrupt Enable                                      */
                __IOM uint32_t MB13 : 1; /*!< [13..13] mailbox 13 Interrupt Enable                                      */
                __IOM uint32_t MB14 : 1; /*!< [14..14] mailbox 14 Interrupt Enable                                      */
                __IOM uint32_t MB15 : 1; /*!< [15..15] mailbox 15 Interrupt Enable                                      */
                __IOM uint32_t MB16 : 1; /*!< [16..16] mailbox 16 Interrupt Enable                                      */
                __IOM uint32_t MB17 : 1; /*!< [17..17] mailbox 17 Interrupt Enable                                      */
                __IOM uint32_t MB18 : 1; /*!< [18..18] mailbox 18 Interrupt Enable                                      */
                __IOM uint32_t MB19 : 1; /*!< [19..19] mailbox 19 Interrupt Enable                                      */
                __IOM uint32_t MB20 : 1; /*!< [20..20] mailbox 20 Interrupt Enable                                      */
                __IOM uint32_t MB21 : 1; /*!< [21..21] mailbox 21 Interrupt Enable                                      */
                __IOM uint32_t MB22 : 1; /*!< [22..22] mailbox 22 Interrupt Enable                                      */
                __IOM uint32_t MB23 : 1; /*!< [23..23] mailbox 23 Interrupt Enable                                      */
                __IOM uint32_t MB24 : 1; /*!< [24..24] mailbox 24 Interrupt Enable                                      */
                __IOM uint32_t MB25 : 1; /*!< [25..25] mailbox 25 Interrupt Enable                                      */
                __IOM uint32_t MB26 : 1; /*!< [26..26] mailbox 26 Interrupt Enable                                      */
                __IOM uint32_t MB27 : 1; /*!< [27..27] mailbox 27 Interrupt Enable                                      */
                __IOM uint32_t MB28 : 1; /*!< [28..28] mailbox 28 Interrupt Enable                                      */
                __IOM uint32_t MB29 : 1; /*!< [29..29] mailbox 29 Interrupt Enable                                      */
                __IOM uint32_t MB30 : 1; /*!< [30..30] mailbox 30 Interrupt Enable                                      */
                __IOM uint32_t MB31 : 1; /*!< [31..31] mailbox 31 Interrupt Enable                                      */
            } MIER_b;
        };

        union
        {
            __IOM uint32_t MIER_FIFO;    /*!< (@ 0x0000042C) Mailbox Interrupt Enable Register for FIFO Mailbox
                                          *                  Mode                                                       */

            struct
            {
                __IOM uint32_t MB0  : 1; /*!< [0..0] mailbox 0 Interrupt Enable                                         */
                __IOM uint32_t MB1  : 1; /*!< [1..1] mailbox 1 Interrupt Enable                                         */
                __IOM uint32_t MB2  : 1; /*!< [2..2] mailbox 2 Interrupt Enable                                         */
                __IOM uint32_t MB3  : 1; /*!< [3..3] mailbox 3 Interrupt Enable                                         */
                __IOM uint32_t MB4  : 1; /*!< [4..4] mailbox 4 Interrupt Enable                                         */
                __IOM uint32_t MB5  : 1; /*!< [5..5] mailbox 5 Interrupt Enable                                         */
                __IOM uint32_t MB6  : 1; /*!< [6..6] mailbox 6 Interrupt Enable                                         */
                __IOM uint32_t MB7  : 1; /*!< [7..7] mailbox 7 Interrupt Enable                                         */
                __IOM uint32_t MB8  : 1; /*!< [8..8] mailbox 8 Interrupt Enable                                         */
                __IOM uint32_t MB9  : 1; /*!< [9..9] mailbox 9 Interrupt Enable                                         */
                __IOM uint32_t MB10 : 1; /*!< [10..10] mailbox 10 Interrupt Enable                                      */
                __IOM uint32_t MB11 : 1; /*!< [11..11] mailbox 11 Interrupt Enable                                      */
                __IOM uint32_t MB12 : 1; /*!< [12..12] mailbox 12 Interrupt Enable                                      */
                __IOM uint32_t MB13 : 1; /*!< [13..13] mailbox 13 Interrupt Enable                                      */
                __IOM uint32_t MB14 : 1; /*!< [14..14] mailbox 14 Interrupt Enable                                      */
                __IOM uint32_t MB15 : 1; /*!< [15..15] mailbox 15 Interrupt Enable                                      */
                __IOM uint32_t MB16 : 1; /*!< [16..16] mailbox 16 Interrupt Enable                                      */
                __IOM uint32_t MB17 : 1; /*!< [17..17] mailbox 17 Interrupt Enable                                      */
                __IOM uint32_t MB18 : 1; /*!< [18..18] mailbox 18 Interrupt Enable                                      */
                __IOM uint32_t MB19 : 1; /*!< [19..19] mailbox 19 Interrupt Enable                                      */
                __IOM uint32_t MB20 : 1; /*!< [20..20] mailbox 20 Interrupt Enable                                      */
                __IOM uint32_t MB21 : 1; /*!< [21..21] mailbox 21 Interrupt Enable                                      */
                __IOM uint32_t MB22 : 1; /*!< [22..22] mailbox 22 Interrupt Enable                                      */
                __IOM uint32_t MB23 : 1; /*!< [23..23] mailbox 23 Interrupt Enable                                      */
                __IOM uint32_t MB24 : 1; /*!< [24..24] Transmit FIFO Interrupt Enable                                   */
                __IOM uint32_t MB25 : 1; /*!< [25..25] Transmit FIFO Interrupt Generation Timing Control                */
                uint32_t            : 2;
                __IOM uint32_t MB28 : 1; /*!< [28..28] Receive FIFO Interrupt Enable                                    */
                __IOM uint32_t MB29 : 1; /*!< [29..29] Receive FIFO Interrupt Generation Timing Control                 */
                uint32_t            : 2;
            } MIER_FIFO_b;
        };
    };
    __IM uint32_t RESERVED1[252];

    union
    {
        union
        {
            __IOM uint8_t MCTL_TX[32];       /*!< (@ 0x00000820) Message Control Register for Transmit                      */

            struct
            {
                __IOM uint8_t SENTDATA  : 1; /*!< [0..0] Transmission Complete Flag                                         */
                __IM uint8_t  TRMACTIVE : 1; /*!< [1..1] Transmission-in-Progress Status Flag (Transmit mailbox
                                              *   setting enabled)                                                          */
                __IOM uint8_t TRMABT : 1;    /*!< [2..2] Transmission Abort Complete Flag (Transmit mailbox setting
                                              *   enabled)                                                                  */
                uint8_t               : 1;
                __IOM uint8_t ONESHOT : 1;   /*!< [4..4] One-Shot Enable                                                    */
                uint8_t               : 1;
                __IOM uint8_t RECREQ  : 1;   /*!< [6..6] Receive Mailbox Request                                            */
                __IOM uint8_t TRMREQ  : 1;   /*!< [7..7] Transmit Mailbox Request                                           */
            } MCTL_TX_b[32];
        };

        union
        {
            __IOM uint8_t MCTL_RX[32];       /*!< (@ 0x00000820) Message Control Register for Receive                       */

            struct
            {
                __IOM uint8_t NEWDATA   : 1; /*!< [0..0] Reception Complete Flag                                            */
                __IM uint8_t  INVALDATA : 1; /*!< [1..1] Reception-in-Progress Status Flag (Receive mailbox setting
                                              *   enabled)                                                                  */
                __IOM uint8_t MSGLOST : 1;   /*!< [2..2] Message Lost Flag(Receive mailbox setting enabled)                 */
                uint8_t               : 1;
                __IOM uint8_t ONESHOT : 1;   /*!< [4..4] One-Shot Enable                                                    */
                uint8_t               : 1;
                __IOM uint8_t RECREQ  : 1;   /*!< [6..6] Receive Mailbox Request                                            */
                __IOM uint8_t TRMREQ  : 1;   /*!< [7..7] Transmit Mailbox Request                                           */
            } MCTL_RX_b[32];
        };
    };

    union
    {
        __IOM uint16_t CTLR;           /*!< (@ 0x00000840) Control Register                                           */

        struct
        {
            __IOM uint16_t MBM  : 1;   /*!< [0..0] CAN Mailbox Mode Select                                            */
            __IOM uint16_t IDFM : 2;   /*!< [2..1] ID Format Mode Select                                              */
            __IOM uint16_t MLM  : 1;   /*!< [3..3] Message Lost Mode Select                                           */
            __IOM uint16_t TPM  : 1;   /*!< [4..4] Transmission Priority Mode Select                                  */
            __IOM uint16_t TSRC : 1;   /*!< [5..5] Time Stamp Counter Reset Command                                   */
            __IOM uint16_t TSPS : 2;   /*!< [7..6] Time Stamp Prescaler Select                                        */
            __IOM uint16_t CANM : 2;   /*!< [9..8] CAN Operating Mode Select                                          */
            __IOM uint16_t SLPM : 1;   /*!< [10..10] CAN Sleep Mode                                                   */
            __IOM uint16_t BOM  : 2;   /*!< [12..11] Bus-Off Recovery Mode by a program request                       */
            __IOM uint16_t RBOC : 1;   /*!< [13..13] Forcible Return From Bus-Off                                     */
            uint16_t            : 2;
        } CTLR_b;
    };

    union
    {
        __IM uint16_t STR;             /*!< (@ 0x00000842) Status Register                                            */

        struct
        {
            __IM uint16_t NDST  : 1;   /*!< [0..0] NEWDATA Status Flag                                                */
            __IM uint16_t SDST  : 1;   /*!< [1..1] SENTDATA Status Flag                                               */
            __IM uint16_t RFST  : 1;   /*!< [2..2] Receive FIFO Status Flag                                           */
            __IM uint16_t TFST  : 1;   /*!< [3..3] Transmit FIFO Status Flag                                          */
            __IM uint16_t NMLST : 1;   /*!< [4..4] Normal Mailbox Message Lost Status Flag                            */
            __IM uint16_t FMLST : 1;   /*!< [5..5] FIFO Mailbox Message Lost Status Flag                              */
            __IM uint16_t TABST : 1;   /*!< [6..6] Transmission Abort Status Flag                                     */
            __IM uint16_t EST   : 1;   /*!< [7..7] Error Status Flag                                                  */
            __IM uint16_t RSTST : 1;   /*!< [8..8] CAN Reset Status Flag                                              */
            __IM uint16_t HLTST : 1;   /*!< [9..9] CAN Halt Status Flag                                               */
            __IM uint16_t SLPST : 1;   /*!< [10..10] CAN Sleep Status Flag                                            */
            __IM uint16_t EPST  : 1;   /*!< [11..11] Error-Passive Status Flag                                        */
            __IM uint16_t BOST  : 1;   /*!< [12..12] Bus-Off Status Flag                                              */
            __IM uint16_t TRMST : 1;   /*!< [13..13] Transmit Status Flag (transmitter)                               */
            __IM uint16_t RECST : 1;   /*!< [14..14] Receive Status Flag (receiver)                                   */
            uint16_t            : 1;
        } STR_b;
    };

    union
    {
        __IOM uint32_t BCR;            /*!< (@ 0x00000844) Bit Configuration Register                                 */

        struct
        {
            __IOM uint32_t CCLKS : 1;  /*!< [0..0] CAN Clock Source Selection                                         */
            uint32_t             : 7;
            __IOM uint32_t TSEG2 : 3;  /*!< [10..8] Time Segment 2 Control                                            */
            uint32_t             : 1;
            __IOM uint32_t SJW   : 2;  /*!< [13..12] Resynchronization Jump Width Control                             */
            uint32_t             : 2;
            __IOM uint32_t BRP   : 10; /*!< [25..16] Prescaler Division Ratio Select . These bits set the
                                        *   frequency of the CAN communication clock (fCANCLK).                       */
            uint32_t             : 2;
            __IOM uint32_t TSEG1 : 4;  /*!< [31..28] Time Segment 1 Control                                           */
        } BCR_b;
    };

    union
    {
        __IOM uint8_t RFCR;            /*!< (@ 0x00000848) Receive FIFO Control Register                              */

        struct
        {
            __IOM uint8_t RFE   : 1;   /*!< [0..0] Receive FIFO Enable                                                */
            __IM uint8_t  RFUST : 3;   /*!< [3..1] Receive FIFO Unread Message Number Status                          */
            __IOM uint8_t RFMLF : 1;   /*!< [4..4] Receive FIFO Message Lost Flag                                     */
            __IM uint8_t  RFFST : 1;   /*!< [5..5] Receive FIFO Full Status Flag                                      */
            __IM uint8_t  RFWST : 1;   /*!< [6..6] Receive FIFO Buffer Warning Status Flag                            */
            __IM uint8_t  RFEST : 1;   /*!< [7..7] Receive FIFO Empty Status Flag                                     */
        } RFCR_b;
    };

    union
    {
        __OM uint8_t RFPCR;            /*!< (@ 0x00000849) Receive FIFO Pointer Control Register                      */

        struct
        {
            __OM uint8_t RFPCR : 8;    /*!< [7..0] The CPU-side pointer for the receive FIFO is incremented
                                        *   by writing FFh to RFPCR.                                                  */
        } RFPCR_b;
    };

    union
    {
        __IOM uint8_t TFCR;            /*!< (@ 0x0000084A) Transmit FIFO Control Register                             */

        struct
        {
            __IOM uint8_t TFE   : 1;   /*!< [0..0] Transmit FIFO Enable                                               */
            __IM uint8_t  TFUST : 3;   /*!< [3..1] Transmit FIFO Unsent Message Number Status                         */
            uint8_t             : 2;
            __IM uint8_t TFFST  : 1;   /*!< [6..6] Transmit FIFO Full Status                                          */
            __IM uint8_t TFEST  : 1;   /*!< [7..7] Transmit FIFO Empty Status                                         */
        } TFCR_b;
    };

    union
    {
        __OM uint8_t TFPCR;            /*!< (@ 0x0000084B) Transmit FIFO Pointer Control Register                     */

        struct
        {
            __OM uint8_t TFPCR : 8;    /*!< [7..0] The CPU-side pointer for the transmit FIFO is incremented
                                        *   by writing FFh to TFPCR.                                                  */
        } TFPCR_b;
    };

    union
    {
        __IOM uint8_t EIER;            /*!< (@ 0x0000084C) Error Interrupt Enable Register                            */

        struct
        {
            __IOM uint8_t BEIE  : 1;   /*!< [0..0] Bus Error Interrupt Enable                                         */
            __IOM uint8_t EWIE  : 1;   /*!< [1..1] Error-Warning Interrupt Enable                                     */
            __IOM uint8_t EPIE  : 1;   /*!< [2..2] Error-Passive Interrupt Enable                                     */
            __IOM uint8_t BOEIE : 1;   /*!< [3..3] Bus-Off Entry Interrupt Enable                                     */
            __IOM uint8_t BORIE : 1;   /*!< [4..4] Bus-Off Recovery Interrupt Enable                                  */
            __IOM uint8_t ORIE  : 1;   /*!< [5..5] Overrun Interrupt Enable                                           */
            __IOM uint8_t OLIE  : 1;   /*!< [6..6] Overload Frame Transmit Interrupt Enable                           */
            __IOM uint8_t BLIE  : 1;   /*!< [7..7] Bus Lock Interrupt Enable                                          */
        } EIER_b;
    };

    union
    {
        __IOM uint8_t EIFR;            /*!< (@ 0x0000084D) Error Interrupt Factor Judge Register                      */

        struct
        {
            __IOM uint8_t BEIF  : 1;   /*!< [0..0] Bus Error Detect Flag                                              */
            __IOM uint8_t EWIF  : 1;   /*!< [1..1] Error-Warning Detect Flag                                          */
            __IOM uint8_t EPIF  : 1;   /*!< [2..2] Error-Passive Detect Flag                                          */
            __IOM uint8_t BOEIF : 1;   /*!< [3..3] Bus-Off Entry Detect Flag                                          */
            __IOM uint8_t BORIF : 1;   /*!< [4..4] Bus-Off Recovery Detect Flag                                       */
            __IOM uint8_t ORIF  : 1;   /*!< [5..5] Receive Overrun Detect Flag                                        */
            __IOM uint8_t OLIF  : 1;   /*!< [6..6] Overload Frame Transmission Detect Flag                            */
            __IOM uint8_t BLIF  : 1;   /*!< [7..7] Bus Lock Detect Flag                                               */
        } EIFR_b;
    };

    union
    {
        __IM uint8_t RECR;             /*!< (@ 0x0000084E) Receive Error Count Register                               */

        struct
        {
            __IM uint8_t RECR : 8;     /*!< [7..0] Receive error count functionRECR increments or decrements
                                        *   the counter value according to the error status of the
                                        *   CAN module during reception.                                              */
        } RECR_b;
    };

    union
    {
        __IM uint8_t TECR;             /*!< (@ 0x0000084F) Transmit Error Count Register                              */

        struct
        {
            __IM uint8_t TECR : 8;     /*!< [7..0] Transmit error count functionTECR increments or decrements
                                        *   the counter value according to the error status of the
                                        *   CAN module during transmission.                                           */
        } TECR_b;
    };

    union
    {
        __IOM uint8_t ECSR;            /*!< (@ 0x00000850) Error Code Store Register                                  */

        struct
        {
            __IOM uint8_t SEF  : 1;    /*!< [0..0] Stuff Error Flag                                                   */
            __IOM uint8_t FEF  : 1;    /*!< [1..1] Form Error Flag                                                    */
            __IOM uint8_t AEF  : 1;    /*!< [2..2] ACK Error Flag                                                     */
            __IOM uint8_t CEF  : 1;    /*!< [3..3] CRC Error Flag                                                     */
            __IOM uint8_t BE1F : 1;    /*!< [4..4] Bit Error (recessive) Flag                                         */
            __IOM uint8_t BE0F : 1;    /*!< [5..5] Bit Error (dominant) Flag                                          */
            __IOM uint8_t ADEF : 1;    /*!< [6..6] ACK Delimiter Error Flag                                           */
            __IOM uint8_t EDPM : 1;    /*!< [7..7] Error Display Mode Select                                          */
        } ECSR_b;
    };

    union
    {
        __IOM uint8_t CSSR;            /*!< (@ 0x00000851) Channel Search Support Register                            */

        struct
        {
            __IOM uint8_t CSSR : 8;    /*!< [7..0] When the value for the channel search is input, the channel
                                        *   number is output to MSSR.                                                 */
        } CSSR_b;
    };

    union
    {
        __IM uint8_t MSSR;             /*!< (@ 0x00000852) Mailbox Search Status Register                             */

        struct
        {
            __IM uint8_t MBNST : 5;    /*!< [4..0] Search Result Mailbox Number Status These bits output
                                        *   the smallest mailbox number that is searched in each mode
                                        *   of MSMR.                                                                  */
            uint8_t           : 2;
            __IM uint8_t SEST : 1;     /*!< [7..7] Search Result Status                                               */
        } MSSR_b;
    };

    union
    {
        __IOM uint8_t MSMR;            /*!< (@ 0x00000853) Mailbox Search Mode Register                               */

        struct
        {
            __IOM uint8_t MBSM : 2;    /*!< [1..0] Mailbox Search Mode Select                                         */
            uint8_t            : 6;
        } MSMR_b;
    };

    union
    {
        __IM uint16_t TSR;             /*!< (@ 0x00000854) Time Stamp Register                                        */

        struct
        {
            __IM uint16_t TSR : 16;    /*!< [15..0] Free-running counter value for the time stamp function            */
        } TSR_b;
    };

    union
    {
        __IOM uint16_t AFSR;           /*!< (@ 0x00000856) Acceptance Filter Support Register                         */

        struct
        {
            __IOM uint16_t AFSR : 16;  /*!< [15..0] After the standard ID of a received message is written,
                                        *   the value converted for data table search can be read.                    */
        } AFSR_b;
    };

    union
    {
        __IOM uint8_t TCR;             /*!< (@ 0x00000858) Test Control Register                                      */

        struct
        {
            __IOM uint8_t TSTE : 1;    /*!< [0..0] CAN Test Mode Enable                                               */
            __IOM uint8_t TSTM : 2;    /*!< [2..1] CAN Test Mode Select                                               */
            uint8_t            : 5;
        } TCR_b;
    };
    __IM uint8_t  RESERVED2;
    __IM uint16_t RESERVED3;
} R_CAN0_Type;                         /*!< Size = 2140 (0x85c)                                                       */

/* =========================================================================================================================== */
/* ================                                           R_CAC                                           ================ */
/* =========================================================================================================================== */

/* =========================================================  CACR0  ========================================================= */
 #define R_CAC_CACR0_CFME_Pos         (0UL)      /*!< CFME (Bit 0)                                          */
 #define R_CAC_CACR0_CFME_Msk         (0x1UL)    /*!< CFME (Bitfield-Mask: 0x01)                            */
/* =========================================================  CACR1  ========================================================= */
 #define R_CAC_CACR1_EDGES_Pos        (6UL)      /*!< EDGES (Bit 6)                                         */
 #define R_CAC_CACR1_EDGES_Msk        (0xc0UL)   /*!< EDGES (Bitfield-Mask: 0x03)                           */
 #define R_CAC_CACR1_TCSS_Pos         (4UL)      /*!< TCSS (Bit 4)                                          */
 #define R_CAC_CACR1_TCSS_Msk         (0x30UL)   /*!< TCSS (Bitfield-Mask: 0x03)                            */
 #define R_CAC_CACR1_FMCS_Pos         (1UL)      /*!< FMCS (Bit 1)                                          */
 #define R_CAC_CACR1_FMCS_Msk         (0xeUL)    /*!< FMCS (Bitfield-Mask: 0x07)                            */
 #define R_CAC_CACR1_CACREFE_Pos      (0UL)      /*!< CACREFE (Bit 0)                                       */
 #define R_CAC_CACR1_CACREFE_Msk      (0x1UL)    /*!< CACREFE (Bitfield-Mask: 0x01)                         */
/* =========================================================  CACR2  ========================================================= */
 #define R_CAC_CACR2_DFS_Pos          (6UL)      /*!< DFS (Bit 6)                                           */
 #define R_CAC_CACR2_DFS_Msk          (0xc0UL)   /*!< DFS (Bitfield-Mask: 0x03)                             */
 #define R_CAC_CACR2_RCDS_Pos         (4UL)      /*!< RCDS (Bit 4)                                          */
 #define R_CAC_CACR2_RCDS_Msk         (0x30UL)   /*!< RCDS (Bitfield-Mask: 0x03)                            */
 #define R_CAC_CACR2_RSCS_Pos         (1UL)      /*!< RSCS (Bit 1)                                          */
 #define R_CAC_CACR2_RSCS_Msk         (0xeUL)    /*!< RSCS (Bitfield-Mask: 0x07)                            */
 #define R_CAC_CACR2_RPS_Pos          (0UL)      /*!< RPS (Bit 0)                                           */
 #define R_CAC_CACR2_RPS_Msk          (0x1UL)    /*!< RPS (Bitfield-Mask: 0x01)                             */
/* =========================================================  CAICR  ========================================================= */
 #define R_CAC_CAICR_OVFFCL_Pos       (6UL)      /*!< OVFFCL (Bit 6)                                        */
 #define R_CAC_CAICR_OVFFCL_Msk       (0x40UL)   /*!< OVFFCL (Bitfield-Mask: 0x01)                          */
 #define R_CAC_CAICR_MENDFCL_Pos      (5UL)      /*!< MENDFCL (Bit 5)                                       */
 #define R_CAC_CAICR_MENDFCL_Msk      (0x20UL)   /*!< MENDFCL (Bitfield-Mask: 0x01)                         */
 #define R_CAC_CAICR_FERRFCL_Pos      (4UL)      /*!< FERRFCL (Bit 4)                                       */
 #define R_CAC_CAICR_FERRFCL_Msk      (0x10UL)   /*!< FERRFCL (Bitfield-Mask: 0x01)                         */
 #define R_CAC_CAICR_OVFIE_Pos        (2UL)      /*!< OVFIE (Bit 2)                                         */
 #define R_CAC_CAICR_OVFIE_Msk        (0x4UL)    /*!< OVFIE (Bitfield-Mask: 0x01)                           */
 #define R_CAC_CAICR_MENDIE_Pos       (1UL)      /*!< MENDIE (Bit 1)                                        */
 #define R_CAC_CAICR_MENDIE_Msk       (0x2UL)    /*!< MENDIE (Bitfield-Mask: 0x01)                          */
 #define R_CAC_CAICR_FERRIE_Pos       (0UL)      /*!< FERRIE (Bit 0)                                        */
 #define R_CAC_CAICR_FERRIE_Msk       (0x1UL)    /*!< FERRIE (Bitfield-Mask: 0x01)                          */
/* =========================================================  CASTR  ========================================================= */
 #define R_CAC_CASTR_OVFF_Pos         (2UL)      /*!< OVFF (Bit 2)                                          */
 #define R_CAC_CASTR_OVFF_Msk         (0x4UL)    /*!< OVFF (Bitfield-Mask: 0x01)                            */
 #define R_CAC_CASTR_MENDF_Pos        (1UL)      /*!< MENDF (Bit 1)                                         */
 #define R_CAC_CASTR_MENDF_Msk        (0x2UL)    /*!< MENDF (Bitfield-Mask: 0x01)                           */
 #define R_CAC_CASTR_FERRF_Pos        (0UL)      /*!< FERRF (Bit 0)                                         */
 #define R_CAC_CASTR_FERRF_Msk        (0x1UL)    /*!< FERRF (Bitfield-Mask: 0x01)                           */
/* ========================================================  CAULVR  ========================================================= */
 #define R_CAC_CAULVR_CAULVR_Pos      (0UL)      /*!< CAULVR (Bit 0)                                        */
 #define R_CAC_CAULVR_CAULVR_Msk      (0xffffUL) /*!< CAULVR (Bitfield-Mask: 0xffff)                        */
/* ========================================================  CALLVR  ========================================================= */
 #define R_CAC_CALLVR_CALLVR_Pos      (0UL)      /*!< CALLVR (Bit 0)                                        */
 #define R_CAC_CALLVR_CALLVR_Msk      (0xffffUL) /*!< CALLVR (Bitfield-Mask: 0xffff)                        */
/* ========================================================  CACNTBR  ======================================================== */
 #define R_CAC_CACNTBR_CACNTBR_Pos    (0UL)      /*!< CACNTBR (Bit 0)                                       */
 #define R_CAC_CACNTBR_CACNTBR_Msk    (0xffffUL) /*!< CACNTBR (Bitfield-Mask: 0xffff)                       */

/* =========================================================================================================================== */
/* ================                                          R_CAN0                                           ================ */
/* =========================================================================================================================== */

/* ==========================================================  MKR  ========================================================== */
 #define R_CAN0_MKR_SID_Pos              (18UL)         /*!< SID (Bit 18)                                          */
 #define R_CAN0_MKR_SID_Msk              (0x1ffc0000UL) /*!< SID (Bitfield-Mask: 0x7ff)                            */
 #define R_CAN0_MKR_EID_Pos              (0UL)          /*!< EID (Bit 0)                                           */
 #define R_CAN0_MKR_EID_Msk              (0x3ffffUL)    /*!< EID (Bitfield-Mask: 0x3ffff)                          */
/* =========================================================  FIDCR  ========================================================= */
 #define R_CAN0_FIDCR_IDE_Pos            (31UL)         /*!< IDE (Bit 31)                                          */
 #define R_CAN0_FIDCR_IDE_Msk            (0x80000000UL) /*!< IDE (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_FIDCR_RTR_Pos            (30UL)         /*!< RTR (Bit 30)                                          */
 #define R_CAN0_FIDCR_RTR_Msk            (0x40000000UL) /*!< RTR (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_FIDCR_SID_Pos            (18UL)         /*!< SID (Bit 18)                                          */
 #define R_CAN0_FIDCR_SID_Msk            (0x1ffc0000UL) /*!< SID (Bitfield-Mask: 0x7ff)                            */
 #define R_CAN0_FIDCR_EID_Pos            (0UL)          /*!< EID (Bit 0)                                           */
 #define R_CAN0_FIDCR_EID_Msk            (0x3ffffUL)    /*!< EID (Bitfield-Mask: 0x3ffff)                          */
/* ========================================================  MKIVLR  ========================================================= */
 #define R_CAN0_MKIVLR_MB31_Pos          (31UL)         /*!< MB31 (Bit 31)                                         */
 #define R_CAN0_MKIVLR_MB31_Msk          (0x80000000UL) /*!< MB31 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB30_Pos          (30UL)         /*!< MB30 (Bit 30)                                         */
 #define R_CAN0_MKIVLR_MB30_Msk          (0x40000000UL) /*!< MB30 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB29_Pos          (29UL)         /*!< MB29 (Bit 29)                                         */
 #define R_CAN0_MKIVLR_MB29_Msk          (0x20000000UL) /*!< MB29 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB28_Pos          (28UL)         /*!< MB28 (Bit 28)                                         */
 #define R_CAN0_MKIVLR_MB28_Msk          (0x10000000UL) /*!< MB28 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB27_Pos          (27UL)         /*!< MB27 (Bit 27)                                         */
 #define R_CAN0_MKIVLR_MB27_Msk          (0x8000000UL)  /*!< MB27 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB26_Pos          (26UL)         /*!< MB26 (Bit 26)                                         */
 #define R_CAN0_MKIVLR_MB26_Msk          (0x4000000UL)  /*!< MB26 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB25_Pos          (25UL)         /*!< MB25 (Bit 25)                                         */
 #define R_CAN0_MKIVLR_MB25_Msk          (0x2000000UL)  /*!< MB25 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB24_Pos          (24UL)         /*!< MB24 (Bit 24)                                         */
 #define R_CAN0_MKIVLR_MB24_Msk          (0x1000000UL)  /*!< MB24 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB23_Pos          (23UL)         /*!< MB23 (Bit 23)                                         */
 #define R_CAN0_MKIVLR_MB23_Msk          (0x800000UL)   /*!< MB23 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB22_Pos          (22UL)         /*!< MB22 (Bit 22)                                         */
 #define R_CAN0_MKIVLR_MB22_Msk          (0x400000UL)   /*!< MB22 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB21_Pos          (21UL)         /*!< MB21 (Bit 21)                                         */
 #define R_CAN0_MKIVLR_MB21_Msk          (0x200000UL)   /*!< MB21 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB20_Pos          (20UL)         /*!< MB20 (Bit 20)                                         */
 #define R_CAN0_MKIVLR_MB20_Msk          (0x100000UL)   /*!< MB20 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB19_Pos          (19UL)         /*!< MB19 (Bit 19)                                         */
 #define R_CAN0_MKIVLR_MB19_Msk          (0x80000UL)    /*!< MB19 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB18_Pos          (18UL)         /*!< MB18 (Bit 18)                                         */
 #define R_CAN0_MKIVLR_MB18_Msk          (0x40000UL)    /*!< MB18 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB17_Pos          (17UL)         /*!< MB17 (Bit 17)                                         */
 #define R_CAN0_MKIVLR_MB17_Msk          (0x20000UL)    /*!< MB17 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB16_Pos          (16UL)         /*!< MB16 (Bit 16)                                         */
 #define R_CAN0_MKIVLR_MB16_Msk          (0x10000UL)    /*!< MB16 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB15_Pos          (15UL)         /*!< MB15 (Bit 15)                                         */
 #define R_CAN0_MKIVLR_MB15_Msk          (0x8000UL)     /*!< MB15 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB14_Pos          (14UL)         /*!< MB14 (Bit 14)                                         */
 #define R_CAN0_MKIVLR_MB14_Msk          (0x4000UL)     /*!< MB14 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB13_Pos          (13UL)         /*!< MB13 (Bit 13)                                         */
 #define R_CAN0_MKIVLR_MB13_Msk          (0x2000UL)     /*!< MB13 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB12_Pos          (12UL)         /*!< MB12 (Bit 12)                                         */
 #define R_CAN0_MKIVLR_MB12_Msk          (0x1000UL)     /*!< MB12 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB11_Pos          (11UL)         /*!< MB11 (Bit 11)                                         */
 #define R_CAN0_MKIVLR_MB11_Msk          (0x800UL)      /*!< MB11 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB10_Pos          (10UL)         /*!< MB10 (Bit 10)                                         */
 #define R_CAN0_MKIVLR_MB10_Msk          (0x400UL)      /*!< MB10 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MKIVLR_MB9_Pos           (9UL)          /*!< MB9 (Bit 9)                                           */
 #define R_CAN0_MKIVLR_MB9_Msk           (0x200UL)      /*!< MB9 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB8_Pos           (8UL)          /*!< MB8 (Bit 8)                                           */
 #define R_CAN0_MKIVLR_MB8_Msk           (0x100UL)      /*!< MB8 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB7_Pos           (7UL)          /*!< MB7 (Bit 7)                                           */
 #define R_CAN0_MKIVLR_MB7_Msk           (0x80UL)       /*!< MB7 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB6_Pos           (6UL)          /*!< MB6 (Bit 6)                                           */
 #define R_CAN0_MKIVLR_MB6_Msk           (0x40UL)       /*!< MB6 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB5_Pos           (5UL)          /*!< MB5 (Bit 5)                                           */
 #define R_CAN0_MKIVLR_MB5_Msk           (0x20UL)       /*!< MB5 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB4_Pos           (4UL)          /*!< MB4 (Bit 4)                                           */
 #define R_CAN0_MKIVLR_MB4_Msk           (0x10UL)       /*!< MB4 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB3_Pos           (3UL)          /*!< MB3 (Bit 3)                                           */
 #define R_CAN0_MKIVLR_MB3_Msk           (0x8UL)        /*!< MB3 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB2_Pos           (2UL)          /*!< MB2 (Bit 2)                                           */
 #define R_CAN0_MKIVLR_MB2_Msk           (0x4UL)        /*!< MB2 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB1_Pos           (1UL)          /*!< MB1 (Bit 1)                                           */
 #define R_CAN0_MKIVLR_MB1_Msk           (0x2UL)        /*!< MB1 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MKIVLR_MB0_Pos           (0UL)          /*!< MB0 (Bit 0)                                           */
 #define R_CAN0_MKIVLR_MB0_Msk           (0x1UL)        /*!< MB0 (Bitfield-Mask: 0x01)                             */
/* =========================================================  MIER  ========================================================== */
 #define R_CAN0_MIER_MB31_Pos            (31UL)         /*!< MB31 (Bit 31)                                         */
 #define R_CAN0_MIER_MB31_Msk            (0x80000000UL) /*!< MB31 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB30_Pos            (30UL)         /*!< MB30 (Bit 30)                                         */
 #define R_CAN0_MIER_MB30_Msk            (0x40000000UL) /*!< MB30 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB29_Pos            (29UL)         /*!< MB29 (Bit 29)                                         */
 #define R_CAN0_MIER_MB29_Msk            (0x20000000UL) /*!< MB29 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB28_Pos            (28UL)         /*!< MB28 (Bit 28)                                         */
 #define R_CAN0_MIER_MB28_Msk            (0x10000000UL) /*!< MB28 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB27_Pos            (27UL)         /*!< MB27 (Bit 27)                                         */
 #define R_CAN0_MIER_MB27_Msk            (0x8000000UL)  /*!< MB27 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB26_Pos            (26UL)         /*!< MB26 (Bit 26)                                         */
 #define R_CAN0_MIER_MB26_Msk            (0x4000000UL)  /*!< MB26 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB25_Pos            (25UL)         /*!< MB25 (Bit 25)                                         */
 #define R_CAN0_MIER_MB25_Msk            (0x2000000UL)  /*!< MB25 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB24_Pos            (24UL)         /*!< MB24 (Bit 24)                                         */
 #define R_CAN0_MIER_MB24_Msk            (0x1000000UL)  /*!< MB24 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB23_Pos            (23UL)         /*!< MB23 (Bit 23)                                         */
 #define R_CAN0_MIER_MB23_Msk            (0x800000UL)   /*!< MB23 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB22_Pos            (22UL)         /*!< MB22 (Bit 22)                                         */
 #define R_CAN0_MIER_MB22_Msk            (0x400000UL)   /*!< MB22 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB21_Pos            (21UL)         /*!< MB21 (Bit 21)                                         */
 #define R_CAN0_MIER_MB21_Msk            (0x200000UL)   /*!< MB21 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB20_Pos            (20UL)         /*!< MB20 (Bit 20)                                         */
 #define R_CAN0_MIER_MB20_Msk            (0x100000UL)   /*!< MB20 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB19_Pos            (19UL)         /*!< MB19 (Bit 19)                                         */
 #define R_CAN0_MIER_MB19_Msk            (0x80000UL)    /*!< MB19 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB18_Pos            (18UL)         /*!< MB18 (Bit 18)                                         */
 #define R_CAN0_MIER_MB18_Msk            (0x40000UL)    /*!< MB18 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB17_Pos            (17UL)         /*!< MB17 (Bit 17)                                         */
 #define R_CAN0_MIER_MB17_Msk            (0x20000UL)    /*!< MB17 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB16_Pos            (16UL)         /*!< MB16 (Bit 16)                                         */
 #define R_CAN0_MIER_MB16_Msk            (0x10000UL)    /*!< MB16 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB15_Pos            (15UL)         /*!< MB15 (Bit 15)                                         */
 #define R_CAN0_MIER_MB15_Msk            (0x8000UL)     /*!< MB15 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB14_Pos            (14UL)         /*!< MB14 (Bit 14)                                         */
 #define R_CAN0_MIER_MB14_Msk            (0x4000UL)     /*!< MB14 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB13_Pos            (13UL)         /*!< MB13 (Bit 13)                                         */
 #define R_CAN0_MIER_MB13_Msk            (0x2000UL)     /*!< MB13 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB12_Pos            (12UL)         /*!< MB12 (Bit 12)                                         */
 #define R_CAN0_MIER_MB12_Msk            (0x1000UL)     /*!< MB12 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB11_Pos            (11UL)         /*!< MB11 (Bit 11)                                         */
 #define R_CAN0_MIER_MB11_Msk            (0x800UL)      /*!< MB11 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB10_Pos            (10UL)         /*!< MB10 (Bit 10)                                         */
 #define R_CAN0_MIER_MB10_Msk            (0x400UL)      /*!< MB10 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_MB9_Pos             (9UL)          /*!< MB9 (Bit 9)                                           */
 #define R_CAN0_MIER_MB9_Msk             (0x200UL)      /*!< MB9 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB8_Pos             (8UL)          /*!< MB8 (Bit 8)                                           */
 #define R_CAN0_MIER_MB8_Msk             (0x100UL)      /*!< MB8 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB7_Pos             (7UL)          /*!< MB7 (Bit 7)                                           */
 #define R_CAN0_MIER_MB7_Msk             (0x80UL)       /*!< MB7 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB6_Pos             (6UL)          /*!< MB6 (Bit 6)                                           */
 #define R_CAN0_MIER_MB6_Msk             (0x40UL)       /*!< MB6 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB5_Pos             (5UL)          /*!< MB5 (Bit 5)                                           */
 #define R_CAN0_MIER_MB5_Msk             (0x20UL)       /*!< MB5 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB4_Pos             (4UL)          /*!< MB4 (Bit 4)                                           */
 #define R_CAN0_MIER_MB4_Msk             (0x10UL)       /*!< MB4 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB3_Pos             (3UL)          /*!< MB3 (Bit 3)                                           */
 #define R_CAN0_MIER_MB3_Msk             (0x8UL)        /*!< MB3 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB2_Pos             (2UL)          /*!< MB2 (Bit 2)                                           */
 #define R_CAN0_MIER_MB2_Msk             (0x4UL)        /*!< MB2 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB1_Pos             (1UL)          /*!< MB1 (Bit 1)                                           */
 #define R_CAN0_MIER_MB1_Msk             (0x2UL)        /*!< MB1 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_MB0_Pos             (0UL)          /*!< MB0 (Bit 0)                                           */
 #define R_CAN0_MIER_MB0_Msk             (0x1UL)        /*!< MB0 (Bitfield-Mask: 0x01)                             */
/* =======================================================  MIER_FIFO  ======================================================= */
 #define R_CAN0_MIER_FIFO_MB29_Pos       (29UL)         /*!< MB29 (Bit 29)                                         */
 #define R_CAN0_MIER_FIFO_MB29_Msk       (0x20000000UL) /*!< MB29 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB28_Pos       (28UL)         /*!< MB28 (Bit 28)                                         */
 #define R_CAN0_MIER_FIFO_MB28_Msk       (0x10000000UL) /*!< MB28 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB25_Pos       (25UL)         /*!< MB25 (Bit 25)                                         */
 #define R_CAN0_MIER_FIFO_MB25_Msk       (0x2000000UL)  /*!< MB25 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB24_Pos       (24UL)         /*!< MB24 (Bit 24)                                         */
 #define R_CAN0_MIER_FIFO_MB24_Msk       (0x1000000UL)  /*!< MB24 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB23_Pos       (23UL)         /*!< MB23 (Bit 23)                                         */
 #define R_CAN0_MIER_FIFO_MB23_Msk       (0x800000UL)   /*!< MB23 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB22_Pos       (22UL)         /*!< MB22 (Bit 22)                                         */
 #define R_CAN0_MIER_FIFO_MB22_Msk       (0x400000UL)   /*!< MB22 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB21_Pos       (21UL)         /*!< MB21 (Bit 21)                                         */
 #define R_CAN0_MIER_FIFO_MB21_Msk       (0x200000UL)   /*!< MB21 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB20_Pos       (20UL)         /*!< MB20 (Bit 20)                                         */
 #define R_CAN0_MIER_FIFO_MB20_Msk       (0x100000UL)   /*!< MB20 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB19_Pos       (19UL)         /*!< MB19 (Bit 19)                                         */
 #define R_CAN0_MIER_FIFO_MB19_Msk       (0x80000UL)    /*!< MB19 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB18_Pos       (18UL)         /*!< MB18 (Bit 18)                                         */
 #define R_CAN0_MIER_FIFO_MB18_Msk       (0x40000UL)    /*!< MB18 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB17_Pos       (17UL)         /*!< MB17 (Bit 17)                                         */
 #define R_CAN0_MIER_FIFO_MB17_Msk       (0x20000UL)    /*!< MB17 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB16_Pos       (16UL)         /*!< MB16 (Bit 16)                                         */
 #define R_CAN0_MIER_FIFO_MB16_Msk       (0x10000UL)    /*!< MB16 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB15_Pos       (15UL)         /*!< MB15 (Bit 15)                                         */
 #define R_CAN0_MIER_FIFO_MB15_Msk       (0x8000UL)     /*!< MB15 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB14_Pos       (14UL)         /*!< MB14 (Bit 14)                                         */
 #define R_CAN0_MIER_FIFO_MB14_Msk       (0x4000UL)     /*!< MB14 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB13_Pos       (13UL)         /*!< MB13 (Bit 13)                                         */
 #define R_CAN0_MIER_FIFO_MB13_Msk       (0x2000UL)     /*!< MB13 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB12_Pos       (12UL)         /*!< MB12 (Bit 12)                                         */
 #define R_CAN0_MIER_FIFO_MB12_Msk       (0x1000UL)     /*!< MB12 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB11_Pos       (11UL)         /*!< MB11 (Bit 11)                                         */
 #define R_CAN0_MIER_FIFO_MB11_Msk       (0x800UL)      /*!< MB11 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB10_Pos       (10UL)         /*!< MB10 (Bit 10)                                         */
 #define R_CAN0_MIER_FIFO_MB10_Msk       (0x400UL)      /*!< MB10 (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MIER_FIFO_MB9_Pos        (9UL)          /*!< MB9 (Bit 9)                                           */
 #define R_CAN0_MIER_FIFO_MB9_Msk        (0x200UL)      /*!< MB9 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB8_Pos        (8UL)          /*!< MB8 (Bit 8)                                           */
 #define R_CAN0_MIER_FIFO_MB8_Msk        (0x100UL)      /*!< MB8 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB7_Pos        (7UL)          /*!< MB7 (Bit 7)                                           */
 #define R_CAN0_MIER_FIFO_MB7_Msk        (0x80UL)       /*!< MB7 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB6_Pos        (6UL)          /*!< MB6 (Bit 6)                                           */
 #define R_CAN0_MIER_FIFO_MB6_Msk        (0x40UL)       /*!< MB6 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB5_Pos        (5UL)          /*!< MB5 (Bit 5)                                           */
 #define R_CAN0_MIER_FIFO_MB5_Msk        (0x20UL)       /*!< MB5 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB4_Pos        (4UL)          /*!< MB4 (Bit 4)                                           */
 #define R_CAN0_MIER_FIFO_MB4_Msk        (0x10UL)       /*!< MB4 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB3_Pos        (3UL)          /*!< MB3 (Bit 3)                                           */
 #define R_CAN0_MIER_FIFO_MB3_Msk        (0x8UL)        /*!< MB3 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB2_Pos        (2UL)          /*!< MB2 (Bit 2)                                           */
 #define R_CAN0_MIER_FIFO_MB2_Msk        (0x4UL)        /*!< MB2 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB1_Pos        (1UL)          /*!< MB1 (Bit 1)                                           */
 #define R_CAN0_MIER_FIFO_MB1_Msk        (0x2UL)        /*!< MB1 (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MIER_FIFO_MB0_Pos        (0UL)          /*!< MB0 (Bit 0)                                           */
 #define R_CAN0_MIER_FIFO_MB0_Msk        (0x1UL)        /*!< MB0 (Bitfield-Mask: 0x01)                             */
/* ========================================================  MCTL_TX  ======================================================== */
 #define R_CAN0_MCTL_TX_TRMREQ_Pos       (7UL)          /*!< TRMREQ (Bit 7)                                        */
 #define R_CAN0_MCTL_TX_TRMREQ_Msk       (0x80UL)       /*!< TRMREQ (Bitfield-Mask: 0x01)                          */
 #define R_CAN0_MCTL_TX_RECREQ_Pos       (6UL)          /*!< RECREQ (Bit 6)                                        */
 #define R_CAN0_MCTL_TX_RECREQ_Msk       (0x40UL)       /*!< RECREQ (Bitfield-Mask: 0x01)                          */
 #define R_CAN0_MCTL_TX_ONESHOT_Pos      (4UL)          /*!< ONESHOT (Bit 4)                                       */
 #define R_CAN0_MCTL_TX_ONESHOT_Msk      (0x10UL)       /*!< ONESHOT (Bitfield-Mask: 0x01)                         */
 #define R_CAN0_MCTL_TX_TRMABT_Pos       (2UL)          /*!< TRMABT (Bit 2)                                        */
 #define R_CAN0_MCTL_TX_TRMABT_Msk       (0x4UL)        /*!< TRMABT (Bitfield-Mask: 0x01)                          */
 #define R_CAN0_MCTL_TX_TRMACTIVE_Pos    (1UL)          /*!< TRMACTIVE (Bit 1)                                     */
 #define R_CAN0_MCTL_TX_TRMACTIVE_Msk    (0x2UL)        /*!< TRMACTIVE (Bitfield-Mask: 0x01)                       */
 #define R_CAN0_MCTL_TX_SENTDATA_Pos     (0UL)          /*!< SENTDATA (Bit 0)                                      */
 #define R_CAN0_MCTL_TX_SENTDATA_Msk     (0x1UL)        /*!< SENTDATA (Bitfield-Mask: 0x01)                        */
/* ========================================================  MCTL_RX  ======================================================== */
 #define R_CAN0_MCTL_RX_TRMREQ_Pos       (7UL)          /*!< TRMREQ (Bit 7)                                        */
 #define R_CAN0_MCTL_RX_TRMREQ_Msk       (0x80UL)       /*!< TRMREQ (Bitfield-Mask: 0x01)                          */
 #define R_CAN0_MCTL_RX_RECREQ_Pos       (6UL)          /*!< RECREQ (Bit 6)                                        */
 #define R_CAN0_MCTL_RX_RECREQ_Msk       (0x40UL)       /*!< RECREQ (Bitfield-Mask: 0x01)                          */
 #define R_CAN0_MCTL_RX_ONESHOT_Pos      (4UL)          /*!< ONESHOT (Bit 4)                                       */
 #define R_CAN0_MCTL_RX_ONESHOT_Msk      (0x10UL)       /*!< ONESHOT (Bitfield-Mask: 0x01)                         */
 #define R_CAN0_MCTL_RX_MSGLOST_Pos      (2UL)          /*!< MSGLOST (Bit 2)                                       */
 #define R_CAN0_MCTL_RX_MSGLOST_Msk      (0x4UL)        /*!< MSGLOST (Bitfield-Mask: 0x01)                         */
 #define R_CAN0_MCTL_RX_INVALDATA_Pos    (1UL)          /*!< INVALDATA (Bit 1)                                     */
 #define R_CAN0_MCTL_RX_INVALDATA_Msk    (0x2UL)        /*!< INVALDATA (Bitfield-Mask: 0x01)                       */
 #define R_CAN0_MCTL_RX_NEWDATA_Pos      (0UL)          /*!< NEWDATA (Bit 0)                                       */
 #define R_CAN0_MCTL_RX_NEWDATA_Msk      (0x1UL)        /*!< NEWDATA (Bitfield-Mask: 0x01)                         */
/* =========================================================  CTLR  ========================================================== */
 #define R_CAN0_CTLR_RBOC_Pos            (13UL)         /*!< RBOC (Bit 13)                                         */
 #define R_CAN0_CTLR_RBOC_Msk            (0x2000UL)     /*!< RBOC (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_CTLR_BOM_Pos             (11UL)         /*!< BOM (Bit 11)                                          */
 #define R_CAN0_CTLR_BOM_Msk             (0x1800UL)     /*!< BOM (Bitfield-Mask: 0x03)                             */
 #define R_CAN0_CTLR_SLPM_Pos            (10UL)         /*!< SLPM (Bit 10)                                         */
 #define R_CAN0_CTLR_SLPM_Msk            (0x400UL)      /*!< SLPM (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_CTLR_CANM_Pos            (8UL)          /*!< CANM (Bit 8)                                          */
 #define R_CAN0_CTLR_CANM_Msk            (0x300UL)      /*!< CANM (Bitfield-Mask: 0x03)                            */
 #define R_CAN0_CTLR_TSPS_Pos            (6UL)          /*!< TSPS (Bit 6)                                          */
 #define R_CAN0_CTLR_TSPS_Msk            (0xc0UL)       /*!< TSPS (Bitfield-Mask: 0x03)                            */
 #define R_CAN0_CTLR_TSRC_Pos            (5UL)          /*!< TSRC (Bit 5)                                          */
 #define R_CAN0_CTLR_TSRC_Msk            (0x20UL)       /*!< TSRC (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_CTLR_TPM_Pos             (4UL)          /*!< TPM (Bit 4)                                           */
 #define R_CAN0_CTLR_TPM_Msk             (0x10UL)       /*!< TPM (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_CTLR_MLM_Pos             (3UL)          /*!< MLM (Bit 3)                                           */
 #define R_CAN0_CTLR_MLM_Msk             (0x8UL)        /*!< MLM (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_CTLR_IDFM_Pos            (1UL)          /*!< IDFM (Bit 1)                                          */
 #define R_CAN0_CTLR_IDFM_Msk            (0x6UL)        /*!< IDFM (Bitfield-Mask: 0x03)                            */
 #define R_CAN0_CTLR_MBM_Pos             (0UL)          /*!< MBM (Bit 0)                                           */
 #define R_CAN0_CTLR_MBM_Msk             (0x1UL)        /*!< MBM (Bitfield-Mask: 0x01)                             */
/* ==========================================================  STR  ========================================================== */
 #define R_CAN0_STR_RECST_Pos            (14UL)         /*!< RECST (Bit 14)                                        */
 #define R_CAN0_STR_RECST_Msk            (0x4000UL)     /*!< RECST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_TRMST_Pos            (13UL)         /*!< TRMST (Bit 13)                                        */
 #define R_CAN0_STR_TRMST_Msk            (0x2000UL)     /*!< TRMST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_BOST_Pos             (12UL)         /*!< BOST (Bit 12)                                         */
 #define R_CAN0_STR_BOST_Msk             (0x1000UL)     /*!< BOST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_STR_EPST_Pos             (11UL)         /*!< EPST (Bit 11)                                         */
 #define R_CAN0_STR_EPST_Msk             (0x800UL)      /*!< EPST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_STR_SLPST_Pos            (10UL)         /*!< SLPST (Bit 10)                                        */
 #define R_CAN0_STR_SLPST_Msk            (0x400UL)      /*!< SLPST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_HLTST_Pos            (9UL)          /*!< HLTST (Bit 9)                                         */
 #define R_CAN0_STR_HLTST_Msk            (0x200UL)      /*!< HLTST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_RSTST_Pos            (8UL)          /*!< RSTST (Bit 8)                                         */
 #define R_CAN0_STR_RSTST_Msk            (0x100UL)      /*!< RSTST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_EST_Pos              (7UL)          /*!< EST (Bit 7)                                           */
 #define R_CAN0_STR_EST_Msk              (0x80UL)       /*!< EST (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_STR_TABST_Pos            (6UL)          /*!< TABST (Bit 6)                                         */
 #define R_CAN0_STR_TABST_Msk            (0x40UL)       /*!< TABST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_FMLST_Pos            (5UL)          /*!< FMLST (Bit 5)                                         */
 #define R_CAN0_STR_FMLST_Msk            (0x20UL)       /*!< FMLST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_NMLST_Pos            (4UL)          /*!< NMLST (Bit 4)                                         */
 #define R_CAN0_STR_NMLST_Msk            (0x10UL)       /*!< NMLST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_STR_TFST_Pos             (3UL)          /*!< TFST (Bit 3)                                          */
 #define R_CAN0_STR_TFST_Msk             (0x8UL)        /*!< TFST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_STR_RFST_Pos             (2UL)          /*!< RFST (Bit 2)                                          */
 #define R_CAN0_STR_RFST_Msk             (0x4UL)        /*!< RFST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_STR_SDST_Pos             (1UL)          /*!< SDST (Bit 1)                                          */
 #define R_CAN0_STR_SDST_Msk             (0x2UL)        /*!< SDST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_STR_NDST_Pos             (0UL)          /*!< NDST (Bit 0)                                          */
 #define R_CAN0_STR_NDST_Msk             (0x1UL)        /*!< NDST (Bitfield-Mask: 0x01)                            */
/* ==========================================================  BCR  ========================================================== */
 #define R_CAN0_BCR_TSEG1_Pos            (28UL)         /*!< TSEG1 (Bit 28)                                        */
 #define R_CAN0_BCR_TSEG1_Msk            (0xf0000000UL) /*!< TSEG1 (Bitfield-Mask: 0x0f)                           */
 #define R_CAN0_BCR_BRP_Pos              (16UL)         /*!< BRP (Bit 16)                                          */
 #define R_CAN0_BCR_BRP_Msk              (0x3ff0000UL)  /*!< BRP (Bitfield-Mask: 0x3ff)                            */
 #define R_CAN0_BCR_SJW_Pos              (12UL)         /*!< SJW (Bit 12)                                          */
 #define R_CAN0_BCR_SJW_Msk              (0x3000UL)     /*!< SJW (Bitfield-Mask: 0x03)                             */
 #define R_CAN0_BCR_TSEG2_Pos            (8UL)          /*!< TSEG2 (Bit 8)                                         */
 #define R_CAN0_BCR_TSEG2_Msk            (0x700UL)      /*!< TSEG2 (Bitfield-Mask: 0x07)                           */
 #define R_CAN0_BCR_CCLKS_Pos            (0UL)          /*!< CCLKS (Bit 0)                                         */
 #define R_CAN0_BCR_CCLKS_Msk            (0x1UL)        /*!< CCLKS (Bitfield-Mask: 0x01)                           */
/* =========================================================  RFCR  ========================================================== */
 #define R_CAN0_RFCR_RFEST_Pos           (7UL)          /*!< RFEST (Bit 7)                                         */
 #define R_CAN0_RFCR_RFEST_Msk           (0x80UL)       /*!< RFEST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_RFCR_RFWST_Pos           (6UL)          /*!< RFWST (Bit 6)                                         */
 #define R_CAN0_RFCR_RFWST_Msk           (0x40UL)       /*!< RFWST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_RFCR_RFFST_Pos           (5UL)          /*!< RFFST (Bit 5)                                         */
 #define R_CAN0_RFCR_RFFST_Msk           (0x20UL)       /*!< RFFST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_RFCR_RFMLF_Pos           (4UL)          /*!< RFMLF (Bit 4)                                         */
 #define R_CAN0_RFCR_RFMLF_Msk           (0x10UL)       /*!< RFMLF (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_RFCR_RFUST_Pos           (1UL)          /*!< RFUST (Bit 1)                                         */
 #define R_CAN0_RFCR_RFUST_Msk           (0xeUL)        /*!< RFUST (Bitfield-Mask: 0x07)                           */
 #define R_CAN0_RFCR_RFE_Pos             (0UL)          /*!< RFE (Bit 0)                                           */
 #define R_CAN0_RFCR_RFE_Msk             (0x1UL)        /*!< RFE (Bitfield-Mask: 0x01)                             */
/* =========================================================  RFPCR  ========================================================= */
 #define R_CAN0_RFPCR_RFPCR_Pos          (0UL)          /*!< RFPCR (Bit 0)                                         */
 #define R_CAN0_RFPCR_RFPCR_Msk          (0xffUL)       /*!< RFPCR (Bitfield-Mask: 0xff)                           */
/* =========================================================  TFCR  ========================================================== */
 #define R_CAN0_TFCR_TFEST_Pos           (7UL)          /*!< TFEST (Bit 7)                                         */
 #define R_CAN0_TFCR_TFEST_Msk           (0x80UL)       /*!< TFEST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_TFCR_TFFST_Pos           (6UL)          /*!< TFFST (Bit 6)                                         */
 #define R_CAN0_TFCR_TFFST_Msk           (0x40UL)       /*!< TFFST (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_TFCR_TFUST_Pos           (1UL)          /*!< TFUST (Bit 1)                                         */
 #define R_CAN0_TFCR_TFUST_Msk           (0xeUL)        /*!< TFUST (Bitfield-Mask: 0x07)                           */
 #define R_CAN0_TFCR_TFE_Pos             (0UL)          /*!< TFE (Bit 0)                                           */
 #define R_CAN0_TFCR_TFE_Msk             (0x1UL)        /*!< TFE (Bitfield-Mask: 0x01)                             */
/* =========================================================  TFPCR  ========================================================= */
 #define R_CAN0_TFPCR_TFPCR_Pos          (0UL)          /*!< TFPCR (Bit 0)                                         */
 #define R_CAN0_TFPCR_TFPCR_Msk          (0xffUL)       /*!< TFPCR (Bitfield-Mask: 0xff)                           */
/* =========================================================  EIER  ========================================================== */
 #define R_CAN0_EIER_BLIE_Pos            (7UL)          /*!< BLIE (Bit 7)                                          */
 #define R_CAN0_EIER_BLIE_Msk            (0x80UL)       /*!< BLIE (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIER_OLIE_Pos            (6UL)          /*!< OLIE (Bit 6)                                          */
 #define R_CAN0_EIER_OLIE_Msk            (0x40UL)       /*!< OLIE (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIER_ORIE_Pos            (5UL)          /*!< ORIE (Bit 5)                                          */
 #define R_CAN0_EIER_ORIE_Msk            (0x20UL)       /*!< ORIE (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIER_BORIE_Pos           (4UL)          /*!< BORIE (Bit 4)                                         */
 #define R_CAN0_EIER_BORIE_Msk           (0x10UL)       /*!< BORIE (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_EIER_BOEIE_Pos           (3UL)          /*!< BOEIE (Bit 3)                                         */
 #define R_CAN0_EIER_BOEIE_Msk           (0x8UL)        /*!< BOEIE (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_EIER_EPIE_Pos            (2UL)          /*!< EPIE (Bit 2)                                          */
 #define R_CAN0_EIER_EPIE_Msk            (0x4UL)        /*!< EPIE (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIER_EWIE_Pos            (1UL)          /*!< EWIE (Bit 1)                                          */
 #define R_CAN0_EIER_EWIE_Msk            (0x2UL)        /*!< EWIE (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIER_BEIE_Pos            (0UL)          /*!< BEIE (Bit 0)                                          */
 #define R_CAN0_EIER_BEIE_Msk            (0x1UL)        /*!< BEIE (Bitfield-Mask: 0x01)                            */
/* =========================================================  EIFR  ========================================================== */
 #define R_CAN0_EIFR_BLIF_Pos            (7UL)          /*!< BLIF (Bit 7)                                          */
 #define R_CAN0_EIFR_BLIF_Msk            (0x80UL)       /*!< BLIF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIFR_OLIF_Pos            (6UL)          /*!< OLIF (Bit 6)                                          */
 #define R_CAN0_EIFR_OLIF_Msk            (0x40UL)       /*!< OLIF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIFR_ORIF_Pos            (5UL)          /*!< ORIF (Bit 5)                                          */
 #define R_CAN0_EIFR_ORIF_Msk            (0x20UL)       /*!< ORIF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIFR_BORIF_Pos           (4UL)          /*!< BORIF (Bit 4)                                         */
 #define R_CAN0_EIFR_BORIF_Msk           (0x10UL)       /*!< BORIF (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_EIFR_BOEIF_Pos           (3UL)          /*!< BOEIF (Bit 3)                                         */
 #define R_CAN0_EIFR_BOEIF_Msk           (0x8UL)        /*!< BOEIF (Bitfield-Mask: 0x01)                           */
 #define R_CAN0_EIFR_EPIF_Pos            (2UL)          /*!< EPIF (Bit 2)                                          */
 #define R_CAN0_EIFR_EPIF_Msk            (0x4UL)        /*!< EPIF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIFR_EWIF_Pos            (1UL)          /*!< EWIF (Bit 1)                                          */
 #define R_CAN0_EIFR_EWIF_Msk            (0x2UL)        /*!< EWIF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_EIFR_BEIF_Pos            (0UL)          /*!< BEIF (Bit 0)                                          */
 #define R_CAN0_EIFR_BEIF_Msk            (0x1UL)        /*!< BEIF (Bitfield-Mask: 0x01)                            */
/* =========================================================  RECR  ========================================================== */
 #define R_CAN0_RECR_RECR_Pos            (0UL)          /*!< RECR (Bit 0)                                          */
 #define R_CAN0_RECR_RECR_Msk            (0xffUL)       /*!< RECR (Bitfield-Mask: 0xff)                            */
/* =========================================================  TECR  ========================================================== */
 #define R_CAN0_TECR_TECR_Pos            (0UL)          /*!< TECR (Bit 0)                                          */
 #define R_CAN0_TECR_TECR_Msk            (0xffUL)       /*!< TECR (Bitfield-Mask: 0xff)                            */
/* =========================================================  ECSR  ========================================================== */
 #define R_CAN0_ECSR_EDPM_Pos            (7UL)          /*!< EDPM (Bit 7)                                          */
 #define R_CAN0_ECSR_EDPM_Msk            (0x80UL)       /*!< EDPM (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_ECSR_ADEF_Pos            (6UL)          /*!< ADEF (Bit 6)                                          */
 #define R_CAN0_ECSR_ADEF_Msk            (0x40UL)       /*!< ADEF (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_ECSR_BE0F_Pos            (5UL)          /*!< BE0F (Bit 5)                                          */
 #define R_CAN0_ECSR_BE0F_Msk            (0x20UL)       /*!< BE0F (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_ECSR_BE1F_Pos            (4UL)          /*!< BE1F (Bit 4)                                          */
 #define R_CAN0_ECSR_BE1F_Msk            (0x10UL)       /*!< BE1F (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_ECSR_CEF_Pos             (3UL)          /*!< CEF (Bit 3)                                           */
 #define R_CAN0_ECSR_CEF_Msk             (0x8UL)        /*!< CEF (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_ECSR_AEF_Pos             (2UL)          /*!< AEF (Bit 2)                                           */
 #define R_CAN0_ECSR_AEF_Msk             (0x4UL)        /*!< AEF (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_ECSR_FEF_Pos             (1UL)          /*!< FEF (Bit 1)                                           */
 #define R_CAN0_ECSR_FEF_Msk             (0x2UL)        /*!< FEF (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_ECSR_SEF_Pos             (0UL)          /*!< SEF (Bit 0)                                           */
 #define R_CAN0_ECSR_SEF_Msk             (0x1UL)        /*!< SEF (Bitfield-Mask: 0x01)                             */
/* =========================================================  CSSR  ========================================================== */
 #define R_CAN0_CSSR_CSSR_Pos            (0UL)          /*!< CSSR (Bit 0)                                          */
 #define R_CAN0_CSSR_CSSR_Msk            (0xffUL)       /*!< CSSR (Bitfield-Mask: 0xff)                            */
/* =========================================================  MSSR  ========================================================== */
 #define R_CAN0_MSSR_SEST_Pos            (7UL)          /*!< SEST (Bit 7)                                          */
 #define R_CAN0_MSSR_SEST_Msk            (0x80UL)       /*!< SEST (Bitfield-Mask: 0x01)                            */
 #define R_CAN0_MSSR_MBNST_Pos           (0UL)          /*!< MBNST (Bit 0)                                         */
 #define R_CAN0_MSSR_MBNST_Msk           (0x1fUL)       /*!< MBNST (Bitfield-Mask: 0x1f)                           */
/* =========================================================  MSMR  ========================================================== */
 #define R_CAN0_MSMR_MBSM_Pos            (0UL)          /*!< MBSM (Bit 0)                                          */
 #define R_CAN0_MSMR_MBSM_Msk            (0x3UL)        /*!< MBSM (Bitfield-Mask: 0x03)                            */
/* ==========================================================  TSR  ========================================================== */
 #define R_CAN0_TSR_TSR_Pos              (0UL)          /*!< TSR (Bit 0)                                           */
 #define R_CAN0_TSR_TSR_Msk              (0xffffUL)     /*!< TSR (Bitfield-Mask: 0xffff)                           */
/* =========================================================  AFSR  ========================================================== */
 #define R_CAN0_AFSR_AFSR_Pos            (0UL)          /*!< AFSR (Bit 0)                                          */
 #define R_CAN0_AFSR_AFSR_Msk            (0xffffUL)     /*!< AFSR (Bitfield-Mask: 0xffff)                          */
/* ==========================================================  TCR  ========================================================== */
 #define R_CAN0_TCR_TSTM_Pos             (1UL)          /*!< TSTM (Bit 1)                                          */
 #define R_CAN0_TCR_TSTM_Msk             (0x6UL)        /*!< TSTM (Bitfield-Mask: 0x03)                            */
 #define R_CAN0_TCR_TSTE_Pos             (0UL)          /*!< TSTE (Bit 0)                                          */
 #define R_CAN0_TCR_TSTE_Msk             (0x1UL)        /*!< TSTE (Bitfield-Mask: 0x01)                            */

/* =========================================================================================================================== */
/* ================                                           R_CRC                                           ================ */
/* =========================================================================================================================== */

/* ========================================================  CRCCR0  ========================================================= */
 #define R_CRC_CRCCR0_DORCLR_Pos          (7UL)          /*!< DORCLR (Bit 7)                                        */
 #define R_CRC_CRCCR0_DORCLR_Msk          (0x80UL)       /*!< DORCLR (Bitfield-Mask: 0x01)                          */
 #define R_CRC_CRCCR0_LMS_Pos             (6UL)          /*!< LMS (Bit 6)                                           */
 #define R_CRC_CRCCR0_LMS_Msk             (0x40UL)       /*!< LMS (Bitfield-Mask: 0x01)                             */
 #define R_CRC_CRCCR0_GPS_Pos             (0UL)          /*!< GPS (Bit 0)                                           */
 #define R_CRC_CRCCR0_GPS_Msk             (0x7UL)        /*!< GPS (Bitfield-Mask: 0x07)                             */
/* ========================================================  CRCCR1  ========================================================= */
 #define R_CRC_CRCCR1_CRCSEN_Pos          (7UL)          /*!< CRCSEN (Bit 7)                                        */
 #define R_CRC_CRCCR1_CRCSEN_Msk          (0x80UL)       /*!< CRCSEN (Bitfield-Mask: 0x01)                          */
 #define R_CRC_CRCCR1_CRCSWR_Pos          (6UL)          /*!< CRCSWR (Bit 6)                                        */
 #define R_CRC_CRCCR1_CRCSWR_Msk          (0x40UL)       /*!< CRCSWR (Bitfield-Mask: 0x01)                          */
/* ========================================================  CRCDIR  ========================================================= */
 #define R_CRC_CRCDIR_CRCDIR_Pos          (0UL)          /*!< CRCDIR (Bit 0)                                        */
 #define R_CRC_CRCDIR_CRCDIR_Msk          (0xffffffffUL) /*!< CRCDIR (Bitfield-Mask: 0xffffffff)                    */
/* =======================================================  CRCDIR_BY  ======================================================= */
 #define R_CRC_CRCDIR_BY_CRCDIR_BY_Pos    (0UL)          /*!< CRCDIR_BY (Bit 0)                                     */
 #define R_CRC_CRCDIR_BY_CRCDIR_BY_Msk    (0xffUL)       /*!< CRCDIR_BY (Bitfield-Mask: 0xff)                       */
/* ========================================================  CRCDOR  ========================================================= */
 #define R_CRC_CRCDOR_CRCDOR_Pos          (0UL)          /*!< CRCDOR (Bit 0)                                        */
 #define R_CRC_CRCDOR_CRCDOR_Msk          (0xffffffffUL) /*!< CRCDOR (Bitfield-Mask: 0xffffffff)                    */
/* =======================================================  CRCDOR_HA  ======================================================= */
 #define R_CRC_CRCDOR_HA_CRCDOR_HA_Pos    (0UL)          /*!< CRCDOR_HA (Bit 0)                                     */
 #define R_CRC_CRCDOR_HA_CRCDOR_HA_Msk    (0xffffUL)     /*!< CRCDOR_HA (Bitfield-Mask: 0xffff)                     */
/* =======================================================  CRCDOR_BY  ======================================================= */
 #define R_CRC_CRCDOR_BY_CRCDOR_BY_Pos    (0UL)          /*!< CRCDOR_BY (Bit 0)                                     */
 #define R_CRC_CRCDOR_BY_CRCDOR_BY_Msk    (0xffUL)       /*!< CRCDOR_BY (Bitfield-Mask: 0xff)                       */
/* ========================================================  CRCSAR  ========================================================= */
 #define R_CRC_CRCSAR_CRCSA_Pos           (0UL)          /*!< CRCSA (Bit 0)                                         */
 #define R_CRC_CRCSAR_CRCSA_Msk           (0x3fffUL)     /*!< CRCSA (Bitfield-Mask: 0x3fff)                         */

/* =========================================================================================================================== */
/* ================                                            MB                                             ================ */
/* =========================================================================================================================== */

/* ==========================================================  ID  =========================================================== */
 #define R_CAN0_MB_ID_IDE_Pos    (31UL)         /*!< IDE (Bit 31)                                          */
 #define R_CAN0_MB_ID_IDE_Msk    (0x80000000UL) /*!< IDE (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MB_ID_RTR_Pos    (30UL)         /*!< RTR (Bit 30)                                          */
 #define R_CAN0_MB_ID_RTR_Msk    (0x40000000UL) /*!< RTR (Bitfield-Mask: 0x01)                             */
 #define R_CAN0_MB_ID_SID_Pos    (18UL)         /*!< SID (Bit 18)                                          */
 #define R_CAN0_MB_ID_SID_Msk    (0x1ffc0000UL) /*!< SID (Bitfield-Mask: 0x7ff)                            */
 #define R_CAN0_MB_ID_EID_Pos    (0UL)          /*!< EID (Bit 0)                                           */
 #define R_CAN0_MB_ID_EID_Msk    (0x3ffffUL)    /*!< EID (Bitfield-Mask: 0x3ffff)                          */
/* ==========================================================  DL  =========================================================== */
 #define R_CAN0_MB_DL_DLC_Pos    (0UL)          /*!< DLC (Bit 0)                                           */
 #define R_CAN0_MB_DL_DLC_Msk    (0xfUL)        /*!< DLC (Bitfield-Mask: 0x0f)                             */
/* ===========================================================  D  =========================================================== */
 #define R_CAN0_MB_D_DATA_Pos    (0UL)          /*!< DATA (Bit 0)                                          */
 #define R_CAN0_MB_D_DATA_Msk    (0xffUL)       /*!< DATA (Bitfield-Mask: 0xff)                            */
/* ==========================================================  TS  =========================================================== */
 #define R_CAN0_MB_TS_TSH_Pos    (8UL)          /*!< TSH (Bit 8)                                           */
 #define R_CAN0_MB_TS_TSH_Msk    (0xff00UL)     /*!< TSH (Bitfield-Mask: 0xff)                             */
 #define R_CAN0_MB_TS_TSL_Pos    (0UL)          /*!< TSL (Bit 0)                                           */
 #define R_CAN0_MB_TS_TSL_Msk    (0xffUL)       /*!< TSL (Bitfield-Mask: 0xff)                             */

 /** Common error codes */
typedef enum e_fsp_err
{
    FSP_SUCCESS = 0,

    FSP_ERR_ASSERTION             = 1,                      ///< A critical assertion has failed
    FSP_ERR_INVALID_POINTER       = 2,                      ///< Pointer points to invalid memory location
    FSP_ERR_INVALID_ARGUMENT      = 3,                      ///< Invalid input parameter
    FSP_ERR_INVALID_CHANNEL       = 4,                      ///< Selected channel does not exist
    FSP_ERR_INVALID_MODE          = 5,                      ///< Unsupported or incorrect mode
    FSP_ERR_UNSUPPORTED           = 6,                      ///< Selected mode not supported by this API
    FSP_ERR_NOT_OPEN              = 7,                      ///< Requested channel is not configured or API not open
    FSP_ERR_IN_USE                = 8,                      ///< Channel/peripheral is running/busy
    FSP_ERR_OUT_OF_MEMORY         = 9,                      ///< Allocate more memory in the driver's cfg.h
    FSP_ERR_HW_LOCKED             = 10,                     ///< Hardware is locked
    FSP_ERR_IRQ_BSP_DISABLED      = 11,                     ///< IRQ not enabled in BSP
    FSP_ERR_OVERFLOW              = 12,                     ///< Hardware overflow
    FSP_ERR_UNDERFLOW             = 13,                     ///< Hardware underflow
    FSP_ERR_ALREADY_OPEN          = 14,                     ///< Requested channel is already open in a different configuration
    FSP_ERR_APPROXIMATION         = 15,                     ///< Could not set value to exact result
    FSP_ERR_CLAMPED               = 16,                     ///< Value had to be limited for some reason
    FSP_ERR_INVALID_RATE          = 17,                     ///< Selected rate could not be met
    FSP_ERR_ABORTED               = 18,                     ///< An operation was aborted
    FSP_ERR_NOT_ENABLED           = 19,                     ///< Requested operation is not enabled
    FSP_ERR_TIMEOUT               = 20,                     ///< Timeout error
    FSP_ERR_INVALID_BLOCKS        = 21,                     ///< Invalid number of blocks supplied
    FSP_ERR_INVALID_ADDRESS       = 22,                     ///< Invalid address supplied
    FSP_ERR_INVALID_SIZE          = 23,                     ///< Invalid size/length supplied for operation
    FSP_ERR_WRITE_FAILED          = 24,                     ///< Write operation failed
    FSP_ERR_ERASE_FAILED          = 25,                     ///< Erase operation failed
    FSP_ERR_INVALID_CALL          = 26,                     ///< Invalid function call is made
    FSP_ERR_INVALID_HW_CONDITION  = 27,                     ///< Detected hardware is in invalid condition
    FSP_ERR_INVALID_FACTORY_FLASH = 28,                     ///< Factory flash is not available on this MCU
    FSP_ERR_INVALID_STATE         = 30,                     ///< API or command not valid in the current state
    FSP_ERR_NOT_ERASED            = 31,                     ///< Erase verification failed
    FSP_ERR_SECTOR_RELEASE_FAILED = 32,                     ///< Sector release failed
    FSP_ERR_NOT_INITIALIZED       = 33,                     ///< Required initialization not complete
    FSP_ERR_NOT_FOUND             = 34,                     ///< The requested item could not be found
    FSP_ERR_NO_CALLBACK_MEMORY    = 35,                     ///< Non-secure callback memory not provided for non-secure callback
    FSP_ERR_BUFFER_EMPTY          = 36,                     ///< No data available in buffer
    FSP_ERR_INVALID_DATA          = 37,                     ///< Accuracy of data is not guaranteed

    /* Start of RTOS only error codes */
    FSP_ERR_INTERNAL     = 100,                             ///< Internal error
    FSP_ERR_WAIT_ABORTED = 101,                             ///< Wait aborted

    /* Start of UART specific */
    FSP_ERR_FRAMING            = 200,                       ///< Framing error occurs
    FSP_ERR_BREAK_DETECT       = 201,                       ///< Break signal detects
    FSP_ERR_PARITY             = 202,                       ///< Parity error occurs
    FSP_ERR_RXBUF_OVERFLOW     = 203,                       ///< Receive queue overflow
    FSP_ERR_QUEUE_UNAVAILABLE  = 204,                       ///< Can't open s/w queue
    FSP_ERR_INSUFFICIENT_SPACE = 205,                       ///< Not enough space in transmission circular buffer
    FSP_ERR_INSUFFICIENT_DATA  = 206,                       ///< Not enough data in receive circular buffer

    /* Start of SPI specific */
    FSP_ERR_TRANSFER_ABORTED = 300,                         ///< The data transfer was aborted.
    FSP_ERR_MODE_FAULT       = 301,                         ///< Mode fault error.
    FSP_ERR_READ_OVERFLOW    = 302,                         ///< Read overflow.
    FSP_ERR_SPI_PARITY       = 303,                         ///< Parity error.
    FSP_ERR_OVERRUN          = 304,                         ///< Overrun error.

    /* Start of CGC Specific */
    FSP_ERR_CLOCK_INACTIVE        = 400,                    ///< Inactive clock specified as system clock.
    FSP_ERR_CLOCK_ACTIVE          = 401,                    ///< Active clock source cannot be modified without stopping first.
    FSP_ERR_NOT_STABILIZED        = 403,                    ///< Clock has not stabilized after its been turned on/off
    FSP_ERR_PLL_SRC_INACTIVE      = 404,                    ///< PLL initialization attempted when PLL source is turned off
    FSP_ERR_OSC_STOP_DET_ENABLED  = 405,                    ///< Illegal attempt to stop LOCO when Oscillation stop is enabled
    FSP_ERR_OSC_STOP_DETECTED     = 406,                    ///< The Oscillation stop detection status flag is set
    FSP_ERR_OSC_STOP_CLOCK_ACTIVE = 407,                    ///< Attempt to clear Oscillation Stop Detect Status with PLL/MAIN_OSC active
    FSP_ERR_CLKOUT_EXCEEDED       = 408,                    ///< Output on target output clock pin exceeds maximum supported limit
    FSP_ERR_USB_MODULE_ENABLED    = 409,                    ///< USB clock configure request with USB Module enabled
    FSP_ERR_HARDWARE_TIMEOUT      = 410,                    ///< A register read or write timed out
    FSP_ERR_LOW_VOLTAGE_MODE      = 411,                    ///< Invalid clock setting attempted in low voltage mode

    /* Start of FLASH Specific */
    FSP_ERR_PE_FAILURE             = 500,                   ///< Unable to enter Programming mode.
    FSP_ERR_CMD_LOCKED             = 501,                   ///< Peripheral in command locked state
    FSP_ERR_FCLK                   = 502,                   ///< FCLK must be >= 4 MHz
    FSP_ERR_INVALID_LINKED_ADDRESS = 503,                   ///< Function or data are linked at an invalid region of memory
    FSP_ERR_BLANK_CHECK_FAILED     = 504,                   ///< Blank check operation failed

    /* Start of CAC Specific */
    FSP_ERR_INVALID_CAC_REF_CLOCK = 600,                    ///< Measured clock rate < reference clock rate

    /* Start of IIRFA Specific */
    FSP_ERR_INVALID_RESULT = 700,                           ///< The result of one or more calculations was +/- infinity.

    /* Start of GLCD Specific */
    FSP_ERR_CLOCK_GENERATION           = 1000,              ///< Clock cannot be specified as system clock
    FSP_ERR_INVALID_TIMING_SETTING     = 1001,              ///< Invalid timing parameter
    FSP_ERR_INVALID_LAYER_SETTING      = 1002,              ///< Invalid layer parameter
    FSP_ERR_INVALID_ALIGNMENT          = 1003,              ///< Invalid memory alignment found
    FSP_ERR_INVALID_GAMMA_SETTING      = 1004,              ///< Invalid gamma correction parameter
    FSP_ERR_INVALID_LAYER_FORMAT       = 1005,              ///< Invalid color format in layer
    FSP_ERR_INVALID_UPDATE_TIMING      = 1006,              ///< Invalid timing for register update
    FSP_ERR_INVALID_CLUT_ACCESS        = 1007,              ///< Invalid access to CLUT entry
    FSP_ERR_INVALID_FADE_SETTING       = 1008,              ///< Invalid fade-in/fade-out setting
    FSP_ERR_INVALID_BRIGHTNESS_SETTING = 1009,              ///< Invalid gamma correction parameter

    /* Start of JPEG Specific */
    FSP_ERR_JPEG_ERR                                = 1100, ///< JPEG error
    FSP_ERR_JPEG_SOI_NOT_DETECTED                   = 1101, ///< SOI not detected until EOI detected.
    FSP_ERR_JPEG_SOF1_TO_SOFF_DETECTED              = 1102, ///< SOF1 to SOFF detected.
    FSP_ERR_JPEG_UNSUPPORTED_PIXEL_FORMAT           = 1103, ///< Unprovided pixel format detected.
    FSP_ERR_JPEG_SOF_ACCURACY_ERROR                 = 1104, ///< SOF accuracy error: other than 8 detected.
    FSP_ERR_JPEG_DQT_ACCURACY_ERROR                 = 1105, ///< DQT accuracy error: other than 0 detected.
    FSP_ERR_JPEG_COMPONENT_ERROR1                   = 1106, ///< Component error 1: the number of SOF0 header components detected is other than 1, 3, or 4.
    FSP_ERR_JPEG_COMPONENT_ERROR2                   = 1107, ///< Component error 2: the number of components differs between SOF0 header and SOS.
    FSP_ERR_JPEG_SOF0_DQT_DHT_NOT_DETECTED          = 1108, ///< SOF0, DQT, and DHT not detected when SOS detected.
    FSP_ERR_JPEG_SOS_NOT_DETECTED                   = 1109, ///< SOS not detected: SOS not detected until EOI detected.
    FSP_ERR_JPEG_EOI_NOT_DETECTED                   = 1110, ///< EOI not detected (default)
    FSP_ERR_JPEG_RESTART_INTERVAL_DATA_NUMBER_ERROR = 1111, ///< Restart interval data number error detected.
    FSP_ERR_JPEG_IMAGE_SIZE_ERROR                   = 1112, ///< Image size error detected.
    FSP_ERR_JPEG_LAST_MCU_DATA_NUMBER_ERROR         = 1113, ///< Last MCU data number error detected.
    FSP_ERR_JPEG_BLOCK_DATA_NUMBER_ERROR            = 1114, ///< Block data number error detected.
    FSP_ERR_JPEG_BUFFERSIZE_NOT_ENOUGH              = 1115, ///< User provided buffer size not enough
    FSP_ERR_JPEG_UNSUPPORTED_IMAGE_SIZE             = 1116, ///< JPEG Image size is not aligned with MCU

    /* Start of touch panel framework specific */
    FSP_ERR_CALIBRATE_FAILED = 1200,                        ///< Calibration failed

    /* Start of IIRFA specific */
    FSP_ERR_IIRFA_ECC_1BIT = 1300,                          ///< 1-bit ECC error detected
    FSP_ERR_IIRFA_ECC_2BIT = 1301,                          ///< 2-bit ECC error detected

    /* Start of IP specific */
    FSP_ERR_IP_HARDWARE_NOT_PRESENT = 1400,                 ///< Requested IP does not exist on this device
    FSP_ERR_IP_UNIT_NOT_PRESENT     = 1401,                 ///< Requested unit does not exist on this device
    FSP_ERR_IP_CHANNEL_NOT_PRESENT  = 1402,                 ///< Requested channel does not exist on this device

    /* Start of USB specific */
    FSP_ERR_USB_FAILED      = 1500,
    FSP_ERR_USB_BUSY        = 1501,
    FSP_ERR_USB_SIZE_SHORT  = 1502,
    FSP_ERR_USB_SIZE_OVER   = 1503,
    FSP_ERR_USB_NOT_OPEN    = 1504,
    FSP_ERR_USB_NOT_SUSPEND = 1505,
    FSP_ERR_USB_PARAMETER   = 1506,

    /* Start of Message framework specific */
    FSP_ERR_NO_MORE_BUFFER           = 2000,         ///< No more buffer found in the memory block pool
    FSP_ERR_ILLEGAL_BUFFER_ADDRESS   = 2001,         ///< Buffer address is out of block memory pool
    FSP_ERR_INVALID_WORKBUFFER_SIZE  = 2002,         ///< Work buffer size is invalid
    FSP_ERR_INVALID_MSG_BUFFER_SIZE  = 2003,         ///< Message buffer size is invalid
    FSP_ERR_TOO_MANY_BUFFERS         = 2004,         ///< Number of buffer is too many
    FSP_ERR_NO_SUBSCRIBER_FOUND      = 2005,         ///< No message subscriber found
    FSP_ERR_MESSAGE_QUEUE_EMPTY      = 2006,         ///< No message found in the message queue
    FSP_ERR_MESSAGE_QUEUE_FULL       = 2007,         ///< No room for new message in the message queue
    FSP_ERR_ILLEGAL_SUBSCRIBER_LISTS = 2008,         ///< Message subscriber lists is illegal
    FSP_ERR_BUFFER_RELEASED          = 2009,         ///< Buffer has been released

    /* Start of 2DG Driver specific */
    FSP_ERR_D2D_ERROR_INIT      = 3000,              ///< D/AVE 2D has an error in the initialization
    FSP_ERR_D2D_ERROR_DEINIT    = 3001,              ///< D/AVE 2D has an error in the initialization
    FSP_ERR_D2D_ERROR_RENDERING = 3002,              ///< D/AVE 2D has an error in the rendering
    FSP_ERR_D2D_ERROR_SIZE      = 3003,              ///< D/AVE 2D has an error in the rendering

    /* Start of ETHER Driver specific */
    FSP_ERR_ETHER_ERROR_NO_DATA              = 4000, ///< No Data in Receive buffer.
    FSP_ERR_ETHER_ERROR_LINK                 = 4001, ///< ETHERC/EDMAC has an error in the Auto-negotiation
    FSP_ERR_ETHER_ERROR_MAGIC_PACKET_MODE    = 4002, ///< As a Magic Packet is being detected, and transmission/reception is not enabled
    FSP_ERR_ETHER_ERROR_TRANSMIT_BUFFER_FULL = 4003, ///< Transmit buffer is not empty
    FSP_ERR_ETHER_ERROR_FILTERING            = 4004, ///< Detect multicast frame when multicast frame filtering enable
    FSP_ERR_ETHER_ERROR_PHY_COMMUNICATION    = 4005, ///< ETHERC/EDMAC has an error in the phy communication
    FSP_ERR_ETHER_RECEIVE_BUFFER_ACTIVE      = 4006, ///< Receive buffer is active.

    /* Start of ETHER_PHY Driver specific */
    FSP_ERR_ETHER_PHY_ERROR_LINK = 5000,             ///< PHY is not link up.
    FSP_ERR_ETHER_PHY_NOT_READY  = 5001,             ///< PHY has an error in the Auto-negotiation

    /* Start of BYTEQ library specific */
    FSP_ERR_QUEUE_FULL  = 10000,                     ///< Queue is full, cannot queue another data
    FSP_ERR_QUEUE_EMPTY = 10001,                     ///< Queue is empty, no data to dequeue

    /* Start of CTSU Driver specific */
    FSP_ERR_CTSU_SCANNING              = 6000,       ///< Scanning.
    FSP_ERR_CTSU_NOT_GET_DATA          = 6001,       ///< Not processed previous scan data.
    FSP_ERR_CTSU_INCOMPLETE_TUNING     = 6002,       ///< Incomplete initial offset tuning.
    FSP_ERR_CTSU_DIAG_NOT_YET          = 6003,       ///< Diagnosis of data collected no yet.
    FSP_ERR_CTSU_DIAG_LDO_OVER_VOLTAGE = 6004,       ///< Diagnosis of LDO over voltage failed.
    FSP_ERR_CTSU_DIAG_CCO_HIGH         = 6005,       ///< Diagnosis of CCO into 19.2uA failed.
    FSP_ERR_CTSU_DIAG_CCO_LOW          = 6006,       ///< Diagnosis of CCO into 2.4uA failed.
    FSP_ERR_CTSU_DIAG_SSCG             = 6007,       ///< Diagnosis of SSCG frequency failed.
    FSP_ERR_CTSU_DIAG_DAC              = 6008,       ///< Diagnosis of non-touch count value failed.
    FSP_ERR_CTSU_DIAG_OUTPUT_VOLTAGE   = 6009,       ///< Diagnosis of LDO output voltage failed.
    FSP_ERR_CTSU_DIAG_OVER_VOLTAGE     = 6010,       ///< Diagnosis of over voltage detection circuit failed.
    FSP_ERR_CTSU_DIAG_OVER_CURRENT     = 6011,       ///< Diagnosis of over current detection circuit failed.
    FSP_ERR_CTSU_DIAG_LOAD_RESISTANCE  = 6012,       ///< Diagnosis of LDO internal resistance value failed.
    FSP_ERR_CTSU_DIAG_CURRENT_SOURCE   = 6013,       ///< Diagnosis of Current source value failed.
    FSP_ERR_CTSU_DIAG_SENSCLK_GAIN     = 6014,       ///< Diagnosis of SENSCLK frequency gain failed.
    FSP_ERR_CTSU_DIAG_SUCLK_GAIN       = 6015,       ///< Diagnosis of SUCLK frequency gain failed.
    FSP_ERR_CTSU_DIAG_CLOCK_RECOVERY   = 6016,       ///< Diagnosis of SUCLK clock recovery function failed.
    FSP_ERR_CTSU_DIAG_CFC_GAIN         = 6017,       ///< Diagnosis of CFC oscillator gain failed.

    /* Start of SDMMC specific */
    FSP_ERR_CARD_INIT_FAILED     = 40000,            ///< SD card or eMMC device failed to initialize.
    FSP_ERR_CARD_NOT_INSERTED    = 40001,            ///< SD card not installed.
    FSP_ERR_DEVICE_BUSY          = 40002,            ///< Device is holding DAT0 low or another operation is ongoing.
    FSP_ERR_CARD_NOT_INITIALIZED = 40004,            ///< SD card was removed.
    FSP_ERR_CARD_WRITE_PROTECTED = 40005,            ///< Media is write protected.
    FSP_ERR_TRANSFER_BUSY        = 40006,            ///< Transfer in progress.
    FSP_ERR_RESPONSE             = 40007,            ///< Card did not respond or responded with an error.

    /* Start of FX_IO specific */
    FSP_ERR_MEDIA_FORMAT_FAILED = 50000,             ///< Media format failed.
    FSP_ERR_MEDIA_OPEN_FAILED   = 50001,             ///< Media open failed.

    /* Start of CAN specific */
    FSP_ERR_CAN_DATA_UNAVAILABLE   = 60000,          ///< No data available.
    FSP_ERR_CAN_MODE_SWITCH_FAILED = 60001,          ///< Switching operation modes failed.
    FSP_ERR_CAN_INIT_FAILED        = 60002,          ///< Hardware initialization failed.
    FSP_ERR_CAN_TRANSMIT_NOT_READY = 60003,          ///< Transmit in progress.
    FSP_ERR_CAN_RECEIVE_MAILBOX    = 60004,          ///< Mailbox is setup as a receive mailbox.
    FSP_ERR_CAN_TRANSMIT_MAILBOX   = 60005,          ///< Mailbox is setup as a transmit mailbox.
    FSP_ERR_CAN_MESSAGE_LOST       = 60006,          ///< Receive message has been overwritten or overrun.
    FSP_ERR_CAN_TRANSMIT_FIFO_FULL = 60007,          ///< Transmit FIFO is full.

    /* Start of SF_WIFI Specific */
    FSP_ERR_WIFI_CONFIG_FAILED    = 70000,           ///< WiFi module Configuration failed.
    FSP_ERR_WIFI_INIT_FAILED      = 70001,           ///< WiFi module initialization failed.
    FSP_ERR_WIFI_TRANSMIT_FAILED  = 70002,           ///< Transmission failed
    FSP_ERR_WIFI_INVALID_MODE     = 70003,           ///< API called when provisioned in client mode
    FSP_ERR_WIFI_FAILED           = 70004,           ///< WiFi Failed.
    FSP_ERR_WIFI_SCAN_COMPLETE    = 70005,           ///< Wifi scan has completed.
    FSP_ERR_WIFI_AP_NOT_CONNECTED = 70006,           ///< WiFi module is not connected to access point
    FSP_ERR_WIFI_UNKNOWN_AT_CMD   = 70007,           ///< DA16XXX Unknown AT command Error
    FSP_ERR_WIFI_INSUF_PARAM      = 70008,           ///< DA16XXX Insufficient parameter
    FSP_ERR_WIFI_TOO_MANY_PARAMS  = 70009,           ///< DA16XXX Too many parameters
    FSP_ERR_WIFI_INV_PARAM_VAL    = 70010,           ///< DA16XXX Wrong parameter value
    FSP_ERR_WIFI_NO_RESULT        = 70011,           ///< DA16XXX No result
    FSP_ERR_WIFI_RSP_BUF_OVFLW    = 70012,           ///< DA16XXX Response buffer overflow
    FSP_ERR_WIFI_FUNC_NOT_CONFIG  = 70013,           ///< DA16XXX Function is not configured
    FSP_ERR_WIFI_NVRAM_WR_FAIL    = 70014,           ///< DA16XXX NVRAM write failure
    FSP_ERR_WIFI_RET_MEM_WR_FAIL  = 70015,           ///< DA16XXX Retention memory write failure
    FSP_ERR_WIFI_UNKNOWN_ERR      = 70016,           ///< DA16XXX unknown error

    /* Start of SF_CELLULAR Specific */
    FSP_ERR_CELLULAR_CONFIG_FAILED       = 80000,    ///< Cellular module Configuration failed.
    FSP_ERR_CELLULAR_INIT_FAILED         = 80001,    ///< Cellular module initialization failed.
    FSP_ERR_CELLULAR_TRANSMIT_FAILED     = 80002,    ///< Transmission failed
    FSP_ERR_CELLULAR_FW_UPTODATE         = 80003,    ///< Firmware is uptodate
    FSP_ERR_CELLULAR_FW_UPGRADE_FAILED   = 80004,    ///< Firmware upgrade failed
    FSP_ERR_CELLULAR_FAILED              = 80005,    ///< Cellular Failed.
    FSP_ERR_CELLULAR_INVALID_STATE       = 80006,    ///< API Called in invalid state.
    FSP_ERR_CELLULAR_REGISTRATION_FAILED = 80007,    ///< Cellular Network registration failed

    /* Start of SF_BLE specific */
    FSP_ERR_BLE_FAILED              = 90001,         ///< BLE operation failed
    FSP_ERR_BLE_INIT_FAILED         = 90002,         ///< BLE device initialization failed
    FSP_ERR_BLE_CONFIG_FAILED       = 90003,         ///< BLE device configuration failed
    FSP_ERR_BLE_PRF_ALREADY_ENABLED = 90004,         ///< BLE device Profile already enabled
    FSP_ERR_BLE_PRF_NOT_ENABLED     = 90005,         ///< BLE device not enabled

    /* Start of SF_BLE_ABS specific */
    FSP_ERR_BLE_ABS_INVALID_OPERATION = 91001,       ///< Invalid operation is executed.
    FSP_ERR_BLE_ABS_NOT_FOUND         = 91002,       ///< Valid data or free space is not found.

    /* Start of Crypto specific (0x10000) @note Refer to sf_cryoto_err.h for Crypto error code. */
    FSP_ERR_CRYPTO_CONTINUE              = 0x10000,  ///< Continue executing function
    FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT = 0x10001,  ///< Hardware resource busy
    FSP_ERR_CRYPTO_SCE_FAIL              = 0x10002,  ///< Internal I/O buffer is not empty
    FSP_ERR_CRYPTO_SCE_HRK_INVALID_INDEX = 0x10003,  ///< Invalid index
    FSP_ERR_CRYPTO_SCE_RETRY             = 0x10004,  ///< Retry
    FSP_ERR_CRYPTO_SCE_VERIFY_FAIL       = 0x10005,  ///< Verify is failed
    FSP_ERR_CRYPTO_SCE_ALREADY_OPEN      = 0x10006,  ///< HW SCE module is already opened
    FSP_ERR_CRYPTO_NOT_OPEN              = 0x10007,  ///< Hardware module is not initialized
    FSP_ERR_CRYPTO_UNKNOWN               = 0x10008,  ///< Some unknown error occurred
    FSP_ERR_CRYPTO_NULL_POINTER          = 0x10009,  ///< Null pointer input as a parameter
    FSP_ERR_CRYPTO_NOT_IMPLEMENTED       = 0x1000a,  ///< Algorithm/size not implemented
    FSP_ERR_CRYPTO_RNG_INVALID_PARAM     = 0x1000b,  ///< An invalid parameter is specified
    FSP_ERR_CRYPTO_RNG_FATAL_ERROR       = 0x1000c,  ///< A fatal error occurred
    FSP_ERR_CRYPTO_INVALID_SIZE          = 0x1000d,  ///< Size specified is invalid
    FSP_ERR_CRYPTO_INVALID_STATE         = 0x1000e,  ///< Function used in an valid state
    FSP_ERR_CRYPTO_ALREADY_OPEN          = 0x1000f,  ///< control block is already opened
    FSP_ERR_CRYPTO_INSTALL_KEY_FAILED    = 0x10010,  ///< Specified input key is invalid.
    FSP_ERR_CRYPTO_AUTHENTICATION_FAILED = 0x10011,  ///< Authentication failed
    FSP_ERR_CRYPTO_SCE_KEY_SET_FAIL      = 0x10012,  ///< Failure to Init Cipher
    FSP_ERR_CRYPTO_SCE_AUTHENTICATION    = 0x10013,  ///< Authentication failed
    FSP_ERR_CRYPTO_SCE_PARAMETER         = 0x10014,  ///< Input date is illegal.
    FSP_ERR_CRYPTO_SCE_PROHIBIT_FUNCTION = 0x10015,  ///< An invalid function call occurred.

    /* Start of Crypto RSIP specific (0x10100) */
    FSP_ERR_CRYPTO_RSIP_RESOURCE_CONFLICT = 0x10100, ///< Hardware resource is busy
    FSP_ERR_CRYPTO_RSIP_FATAL             = 0x10101, ///< Hardware fatal error or unexpected return
    FSP_ERR_CRYPTO_RSIP_FAIL              = 0x10102, ///< Internal error
    FSP_ERR_CRYPTO_RSIP_KEY_SET_FAIL      = 0x10103, ///< Input key type is illegal
    FSP_ERR_CRYPTO_RSIP_AUTHENTICATION    = 0x10104, ///< Authentication failed

    /* Start of SF_CRYPTO specific */
    FSP_ERR_CRYPTO_COMMON_NOT_OPENED      = 0x20000, ///< Crypto Framework Common is not opened
    FSP_ERR_CRYPTO_HAL_ERROR              = 0x20001, ///< Cryoto HAL module returned an error
    FSP_ERR_CRYPTO_KEY_BUF_NOT_ENOUGH     = 0x20002, ///< Key buffer size is not enough to generate a key
    FSP_ERR_CRYPTO_BUF_OVERFLOW           = 0x20003, ///< Attempt to write data larger than what the buffer can hold
    FSP_ERR_CRYPTO_INVALID_OPERATION_MODE = 0x20004, ///< Invalid operation mode.
    FSP_ERR_MESSAGE_TOO_LONG              = 0x20005, ///< Message for RSA encryption is too long.
    FSP_ERR_RSA_DECRYPTION_ERROR          = 0x20006, ///< RSA Decryption error.

    /** @note SF_CRYPTO APIs may return an error code starting from 0x10000 which is of Crypto module.
     *        Refer to sf_cryoto_err.h for Crypto error codes.
     */

    /* Start of Sensor specific */
    FSP_ERR_SENSOR_INVALID_DATA             = 0x30000, ///< Data is invalid.
    FSP_ERR_SENSOR_IN_STABILIZATION         = 0x30001, ///< Sensor is stabilizing.
    FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED = 0x30002, ///< Measurement is not finished.

    /* Start of COMMS specific */
    FSP_ERR_COMMS_BUS_NOT_OPEN = 0x40000,              ///< Bus is not open.
} fsp_err_t;

#endif /* COMMON_H_ */
