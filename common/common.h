/***********************************************************************************************************************
 * File Name    : common.h
 * Description  : Contains bit math macro and reg access.
 **********************************************************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_

/*=====Bit Math Macro=====*/
#define SET_BIT(REG, Pos)     ((REG) |= (1U << Pos))

#define CLEAR_BIT(REG, Pos)   ((REG) &= ~(1U << Pos))

#define READ_BIT(REG, Pos)    ((REG) & (1 << Pos))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))



#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
/* =========================================================================================================================== */
/* ================                                          R_CAN0                                           ================ */
/* =========================================================================================================================== */

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

#endif /* COMMON_H_ */
