/*
 * CAN_Driver.h
 *
 *  Created on: 26 thg 7, 2024
 *      Author: dthanhchau
 */
#ifndef R_CAN_H
#define R_CAN_H
/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
//#include "bsp_api.h"
#include "common.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define CAN_OPEN                (0x5243414EU)
#define CAN_CLOSE               (0x0U)
#define CAN_FIFO_CPU_POINTER    (0xFF)
#define CAN_DATA_PAYLOAD        (0x08)
/**********************************************************************************************************************
 * Typedef definitions enum
 **********************************************************************************************************************/
/** CAN channel select */
typedef enum e_can_channel
{
    CAN_CHANNEL_0 = 0,           ///< CAN Channel 0 .
    CAN_CHANNEL_1,               ///< CAN Channel 1 .
} can_channel_t;

/** CAN frame type */
typedef enum e_can_frame_type
{
    CAN_FRAME_TYPE_DATA = 0,           ///< Data frame.
    CAN_FRAME_TYPE_REMOTE,             ///< Remote frame.
} can_frame_type_t;

/** CAN event codes */
typedef enum e_can_event
{
    CAN_EVENT_ERR_WARNING          = 0x0002, ///< Error Warning event.
    CAN_EVENT_ERR_PASSIVE          = 0x0004, ///< Error Passive event.
    CAN_EVENT_ERR_BUS_OFF          = 0x0008, ///< Bus Off event.
    CAN_EVENT_BUS_RECOVERY         = 0x0010, ///< Bus Off Recovery event.
    CAN_EVENT_MAILBOX_MESSAGE_LOST = 0x0020, ///< Mailbox has been overrun.
    CAN_EVENT_ERR_BUS_LOCK         = 0x0080, ///< Bus lock detected (32 consecutive dominant bits).
    CAN_EVENT_ERR_CHANNEL          = 0x0100, ///< Channel error has occurred.
    CAN_EVENT_TX_ABORTED           = 0x0200, ///< Transmit abort event.
    CAN_EVENT_RX_COMPLETE          = 0x0400, ///< Receive complete event.
    CAN_EVENT_TX_COMPLETE          = 0x0800, ///< Transmit complete event.
    CAN_EVENT_ERR_GLOBAL           = 0x1000, ///< Global error has occurred.
    CAN_EVENT_TX_FIFO_EMPTY        = 0x2000, ///< Transmit FIFO is empty.
    CAN_EVENT_FIFO_MESSAGE_LOST    = 0x4000, ///< Receive FIFO overrun.
} can_event_t;

///<============Control Register (CTLR)============>///

/* CAN mailbox mode */
typedef enum e_can_mailbox_mode
{
    CAN_MAILBOX_NONMAL_MODE = 0,    //< Mailbox Nomal Mode
    CAN_MAILBOX_FIFO_MODE           //< Mailbox FIFO Mode
} can_mailbox_mode_t;

/* CAN ID mode */
typedef enum e_can_id_mode
{   
    CAN_STANDARD_ID_MODE = 0,       //< Standard ID Mode
    CAN_EXTENDED_ID_MODE,           //< Extended ID Mode
    CAN_MIXED_ID_MODE,              //< Mixed ID Mode
} can_id_mode_t;

/* CAN Message Lost Mode Select */
typedef enum e_can_message_lost_mode
{
    CAN_OVERWRITE_MODE = 0,         //< Overwrite mode
    CAN_OVERRUN_MODE,               //< Overrun mode
} can_message_lost_mode_t;

/* CAN Transmission Priority Mode */
typedef enum e_can_transmit_priority_mode
{
    CAN_TRANSMIT_ID_PRIORITY = 0,         //< ID priority
    CAN_TRANSMIT_MAILBOX_PRIORITY,        //< Mailbox priority
} can_transmit_priority_mode_t;

/* CAN timestamp Pre-scaler */
typedef enum e_can_timestamp_prescaler{
    CAN_TIMESTAMP_PRESCALER_1BITTIME,    //<Every bit time
    CAN_TIMESTAMP_PRESCALER_2BITTIME,    //<Every 2 bit time
    CAN_TIMESTAMP_PRESCALER_4BITTIME,    //<Every 4 bit time
    CAN_TIMESTAMP_PRESCALER_8BITTIME     //<Every 8 bit time
} can_timestamp_prescaler_t;

/* CAN Operation modes */
typedef enum e_can_operation_mode
{
    CAN_OPERATION_MODE_NORMAL = 0,       ///< CAN Normal Operation Mode
    CAN_OPERATION_MODE_RESET,            ///< CAN Reset Operation Mode
    CAN_OPERATION_MODE_HALT,             ///< CAN Halt Operation Mode
    CAN_OPERATION_MODE_RESET_FORCED,     ///< CAN Reset Operation Mode
    CAN_OPERATION_MODE_SLEEP  = 5,       ///< CAN Sleep Operation Mode
} can_operation_mode_t;

/* CAN Bus-Off Recovery Mode */
typedef enum e_can_busoff_recovery_mode
{
    CAN_BUSOFF_RECOVERY_NORMAL = 0,          ///< Normal mode
    CAN_BUSOFF_RECOVERY_ENTERING_BUSOFF, ///< Enter CAN halt mode automatically on entering bus-off state
    CAN_BUSOFF_RECOVERY_END_BUSOFF,      ///< Enter CAN halt mode automatically on end of bus-off state
    CAN_BUSOFF_RECOVERY_BY_SOFTWARE      ///< Enter CAN halt mode during bus-off recovery period through a software request
}can_busoff_recovery_mode_t;

///<============Bit Configuration Register (BCR)============>///
/*CAN Clock Source Selection */
typedef enum e_can_clock_source
{
    CAN_CLKSCR_PCLKB = 0,           ///< PCLKB is the source of the CAN Clock
    CAN_CLKSCR_CANMCLK,             ///< CANMCLK is the source of the CAN Clock
}can_clock_source_t;

/** CAN Mailbox IDs (MB + FIFO) */
typedef enum e_can_mailbox_id
{
    CAN_MAILBOX_ID_0  = 0,
    CAN_MAILBOX_ID_1  = 1,
    CAN_MAILBOX_ID_2  = 2,
    CAN_MAILBOX_ID_3  = 3,
    CAN_MAILBOX_ID_4  = 4,
    CAN_MAILBOX_ID_5  = 5,
    CAN_MAILBOX_ID_6  = 6,
    CAN_MAILBOX_ID_7  = 7,
    CAN_MAILBOX_ID_8  = 8,
    CAN_MAILBOX_ID_9  = 9,
    CAN_MAILBOX_ID_10 = 10,
    CAN_MAILBOX_ID_11 = 11,
    CAN_MAILBOX_ID_12 = 12,
    CAN_MAILBOX_ID_13 = 13,
    CAN_MAILBOX_ID_14 = 14,
    CAN_MAILBOX_ID_15 = 15,
    CAN_MAILBOX_ID_16 = 16,
    CAN_MAILBOX_ID_17 = 17,
    CAN_MAILBOX_ID_18 = 18,
    CAN_MAILBOX_ID_19 = 19,
    CAN_MAILBOX_ID_20 = 20,
    CAN_MAILBOX_ID_21 = 21,
    CAN_MAILBOX_ID_22 = 22,
    CAN_MAILBOX_ID_23 = 23,
    CAN_MAILBOX_ID_24 = 24,
    CAN_MAILBOX_ID_25 = 25,
    CAN_MAILBOX_ID_26 = 26,
    CAN_MAILBOX_ID_27 = 27,
    CAN_MAILBOX_ID_28 = 28,
    CAN_MAILBOX_ID_29 = 29,
    CAN_MAILBOX_ID_30 = 30,
    CAN_MAILBOX_ID_31 = 31,

    CAN_MAILBOX_ID_TX_FIFO = 24,
    CAN_MAILBOX_ID_RX_FIFO = 28,
} can_mailbox_id_t;

/* CAN mailbox type */
typedef enum e_can_mailbox_type
{
    CAN_MAILBOX_RECEIVE = 0,    ///< CAN Receive mailbox
    CAN_MAILBOX_TRANSMIT        ///< CAN Transmit mailbox
} can_mailbox_type_t;

///<============Status Register (STR)============>///
/** CAN Status */
typedef enum e_can_status
{
    CAN_STATUS_NEW_DATA                 = 0x01,     ///< New Data status flag
    CAN_STATUS_SENT_DATA                = 0x02,     ///< Sent Data status flag
    CAN_STATUS_RECEIVE_FIFO             = 0x04,     ///< Receive FIFO status flag
    CAN_STATUS_TRANSMIT_FIFO            = 0x08,     ///< Transmit FIFO status flag
    CAN_STATUS_NORMAL_MBOX_MESSAGE_LOST = 0x10,     ///< Normal mailbox message lost status flag
    CAN_STATUS_FIFO_MBOX_MESSAGE_LOST   = 0x20,     ///< FIFO mailbox message lost status flag
    CAN_STATUS_TRANSMISSION_ABORT       = 0x40,     ///< Transmission abort status flag
    CAN_STATUS_ERROR                    = 0x80,     ///< Error status flag
    CAN_STATUS_RESET_MODE               = 0x100,    ///< Reset mode status flag
    CAN_STATUS_HALT_MODE                = 0x200,    ///< Halt mode status flag
    CAN_STATUS_SLEEP_MODE               = 0x400,    ///< Sleep mode status flag
    CAN_STATUS_ERROR_PASSIVE            = 0x800,    ///< Error-passive status flag
    CAN_STATUS_BUS_OFF                  = 0x1000,   ///< Bus-off status flag
    CAN_STATUS_TRANSMIT                 = 0x2000,   ///< Transmit status flag
    CAN_STATUS_RECEIVE                  = 0x4000,   ///< Receive status flag
} can_status_t;

///<============Mailbox Search Mode Register (MSMR)============>///
/** CAN Mailbox Search Options*/
typedef enum e_can_mailbox_search_mode
{
    CAN_SEARCHMODE_RECEIVE_MAILBOX = 0,     ///< Receive mailbox search mode.
    CAN_SEARCHMODE_TRANSMIT_MAILBOX,        ///< Transmit mailbox search mode.
    CAN_SEARCHMODE_MESSAGE_LOST,            ///< Message lost search mode.
    CAN_SEARCHMODE_CHANNEL                  ///< Channel search mode.
} can_mailbox_search_mode_t;

///<============Error Interrupt Enable Register (EIER)============>///
/** CAN Error IRQ Enable Options*/
typedef enum e_can_error_irq
{
    CAN_IRQ_ERROR_BUS_ERROR          = 0x01,      ///<Bus Error Interrupt Enable
    CAN_IRQ_ERROR_ERROR_WARNING      = 0x02,      ///<Error-Warning Interrupt Enable
    CAN_IRQ_ERROR_ERROR_PASSIVE      = 0x04,      ///<Error-Passive Interrupt Enable
    CAN_IRQ_ERROR_BUS_OFF_ENTRY      = 0x08,      ///<Bus-Off Entry Interrupt Enable
    CAN_IRQ_ERROR_BUS_OFF_RECOVERY   = 0x10,      ///<Bus-Off Recovery Interrupt Enable
    CAN_IRQ_ERROR_OVERRUN            = 0x20,      ///<Overrun Interrupt Enable
    CAN_IRQ_ERROR_OVERLOAD           = 0x40,      ///<Overload Frame Transmit Interrupt Enable
    CAN_IRQ_ERROR_BUS_LOCK           = 0x80       ///<Bus Lock Interrupt Enable
} can_irq_error_t;

/** CAN FIFO Interrupt Modes */
typedef enum e_can_fifo_interrupt_mode
{
    CAN_FIFO_INTERRUPT_MODE_TX_EVERY_FRAME      = R_CAN0_MIER_FIFO_MB24_Msk,
    CAN_FIFO_INTERRUPT_MODE_RX_EVERY_FRAME      = R_CAN0_MIER_FIFO_MB28_Msk,
    CAN_FIFO_INTERRUPT_MODE_TX_EMPTY            = R_CAN0_MIER_FIFO_MB25_Msk | R_CAN0_MIER_FIFO_MB24_Msk,
    CAN_FIFO_INTERRUPT_MODE_RX_BUFFER_WARNING   = R_CAN0_MIER_FIFO_MB29_Msk | R_CAN0_MIER_FIFO_MB28_Msk,
} can_fifo_interrupt_mode_t;

///<============Error Interrupt Factor Judge Register (EIFR)============>///
/** CAN Error Detect Flags*/
typedef enum e_can_error_detect
{
    CAN_DETECT_BUS_ERROR         = 0x01,      ///<Bus Error Detect Flag
    CAN_DETECT_ERROR_WARNING     = 0x02,      ///<Error-Warning Detect Flag
    CAN_DETECT_ERROR_PASSIVE     = 0x04,      ///<Error-Passive Detect Flag
    CAN_DETECT_BUS_OFF_ENTRY     = 0x08,      ///<Bus-Off Entry Detect FlaT
    CAN_DETECT_BUS_OFF_RECOVERY  = 0x10,      ///<Bus-Off Recovery Detect Flag
    CAN_DETECT_OVERRUN           = 0x20,      ///<Receive Overrun Detect Flag
    CAN_DETECT_OVERLOAD          = 0x40,      ///<Overload Frame Transmission Detect Flag
    CAN_DETECT_BUS_LOCK          = 0x80       ///<Bus Lock Detect Flag
} can_error_detect_t;

///<============Error Code Store Register (ECSR)============>///
/** CAN Error Codes*/
typedef enum e_can_error_code
{
    CAN_ERROR_STUFF         = 0x01,       ///< Stuff Error
    CAN_ERROR_FORM          = 0x02,       ///< Form Error
    CAN_ERROR_ACK           = 0x04,       ///< ACK Error
    CAN_ERROR_CRC           = 0x08,       ///< CRC Error
    CAN_ERROR_BIT_RECESSIVE = 0x10,       ///< Bit Error (recessive) Error
    CAN_ERROR_BIT_DOMINANT  = 0x20,       ///< Bit Error (dominant) Error
    CAN_ERROR_ACK_DELIMITER = 0x40,       ///< ACK Delimiter Error
} can_error_t;

///<============Error Code Store Register (ECSR)============>///
typedef enum e_can_test_mode
{
    CAN_TEST_MODE_DISABLE            = 0x00,     ///< Disable Test Mode
    CAN_TEST_MODE_LISTEN_ONLY        = 0x03,     ///< Listen-only mode
    CAN_TEST_MODE_EXTERNAL_LOOPBACK  = 0x04,     ///< Self-test mode 0 (external loopback)
    CAN_TEST_MODE_INTERNAL_LOOPBACK  = 0x05      ///< Self-test mode 1 (internal loopback)
} can_test_mode_t;


/**********************************************************************************************************************
 * Typedef definitions struct
 **********************************************************************************************************************/
/** CAN bit rate configuration. */
typedef struct st_can_bit_timing_cfg
{
    uint32_t Baud_rate_prescaler        :10;    ///< Baud rate prescaler. Valid values: 1 - 1024.
    uint32_t Time_segment_1             :4;     ///< Time segment 1 control.
    uint32_t Time_segment_2             :3;     ///< Time segment 2 control.
    uint32_t Synchronization_jump_width :2;     ///< Synchronization jump width.
} can_bit_timing_cfg_t;

/** CAN control bits configuration */
typedef struct st_can_cfg_ctrl
{
    can_id_mode_t                   can_g_id_mode;              ///< Global ID mode
    can_message_lost_mode_t         can_msg_lost_mode;          ///< CAN MSG lost mode.
    can_transmit_priority_mode_t    can_tx_priority;            ///< CAN tx priority (ID or Mailbox priority).
    can_timestamp_prescaler_t       can_timestamp_prescaler;    ///< CAN time stamp pre-scaler.
    can_busoff_recovery_mode_t      can_busoff_recovery_mode;   ///< CAN bus-off recovery mode.
} can_cfg_ctrl_t;

/** CAN Mailbox configuration */
typedef struct st_can_mailbox_cfg
{
    uint32_t                can_mailbox_id;          ///< CAN mailbox acceptance ID
    can_id_mode_t           can_id_mode;             ///< CAN ID mode
    can_frame_type_t        can_frame_type;          ///< CAN frame type
    can_mailbox_type_t      can_mailbox_type;        ///< CAN mailbox receive or transmit
} can_mailbox_t;

/** CAN Mailbox configuration */
typedef struct st_can_mailbox_option
{
    uint8_t                     can_mailboxs_count;     ///< Number of mailbox avaiable
    uint32_t * const            p_can_mailboxs_mask;    ///< CAN Normal Mailbox Mask (max : 8)
    can_mailbox_t * const       p_can_mailboxs_cfg;     ///< CAN Normal Mailbox Config (max : 32)
} can_mailbox_options_t;

/** CAN FIFO options */
typedef struct st_can_fifo_option
{
    can_fifo_interrupt_mode_t   can_fifo_irq_mode;      ///< Can_IRQ_fifo_mode
    uint32_t                    can_rx_fifo_mask1;      ///< Can_mailbox_mask1
    uint32_t                    can_rx_fifo_mask2;      ///< Can_mailbox_mask2
    can_mailbox_t               can_rx_fifo_id1;        ///< Can_mailbox_cfg_fifo1
    can_mailbox_t               can_rx_fifo_id2;        ///< Can_mailbox_cfg_fifo2
    IRQn_Type                   can_tx_fifo_irq;        ///< IRQ number tx fifo
    IRQn_Type                   can_rx_fifo_irq;        ///< IRQ number rx fifo
} can_fifo_options_t;

/** CAN IRQ */
typedef struct st_can_irq
{
    IRQn_Type   can_error_irq;   ///< IRQ number error
    IRQn_Type   can_tx_irq;      ///< IRQ number tx
    IRQn_Type   can_rx_irq;      ///< IRQ number rx
} can_irq_t;

/** CAN frame */
typedef struct st_can_frame
{
    uint32_t           can_frame_id;                        ///< Frame ID
    can_id_mode_t      can_frame_id_mode;                   ///< Frame Mode (Standard or Extend)
    can_frame_type_t   can_frame_type;                      ///< Frame type (Data frame or Remote frame)
    uint8_t            can_frame_dlc;                       ///< Frame data lenght code
    uint8_t            can_frame_data[CAN_DATA_PAYLOAD];    ///< Frame data
} can_frame_t;

/** CAN Callback Arguments */
typedef struct st_can_callback_arg
{
    can_channel_t       can_channel;    ///< CAN Channel (0 or 1)
    can_event_t         can_event;      ///< CAN Event Occur
    uint32_t            can_mailbox;    ///< CAN Mailbox Event Orrur (Except ERROR)
    can_frame_t         can_frame;      ///< CAN Frame Receive or Transmit
} can_callback_arg_t;

/** CAN Configuration Struct */
typedef struct st_can_cfg
{
    can_clock_source_t              can_clock_src;              ///< Source of the CAN clock.
    can_mailbox_mode_t              can_mailbox_mode;
    can_channel_t                   can_channel;                ///< CAN Channel (0 or 1)
    can_cfg_ctrl_t * const          p_can_cfg_ctrl;             ///< Pointer to CAN Control Config Struct (Bits in CLTR reg)
    can_bit_timing_cfg_t * const    p_can_cfg_bit_timing;       ///< Pointer to CAN Bit timing Struct
    can_mailbox_options_t  *const   p_can_mailbox_options;      ///< Pointer to CAN Mailboxs options (IDE ,RTR ,Mask ,Avaiable MailBox )
    can_fifo_options_t *const       p_can_fifo_options;         ///< Pointer to CAN FIFO options
    can_irq_t                       can_irq_option;             ///< CAN IRQ options for normal mailbox
    uint8_t                         can_irq_priority;           ///< CAN IRQ Priority (priority >= 0)
    void const*                     p_can_callback_context;     ///< CAN Callback conext (can be NULL)
    void (* p_callback)(can_callback_arg_t * p_args);           ///< Pointer to Callback function
} can_cfg_t;

/** CAN Control Struct */
typedef struct st_can_ctrl
{
    can_cfg_t const        *p_cfg;
    R_CAN0_Type            *p_reg;
    uint32_t                can_open;                   ///< CAN Open (CAN Open: 0x5243414E )
    can_operation_mode_t    can_operation_mode;         ///< CAN Operation Mode
    can_test_mode_t         can_test_mode;              ///< CAN Test Mode
    can_clock_source_t      can_clock_src;              ///< CAN Clock Src
    uint8_t                 can_mailbox_count;          ///< CAN mailbox avaiable

    can_callback_arg_t     *p_callback_memory;          ///< Pointer to CAN Callback memory
    void const             *p_callback_context;         ///< Pointer to CAN Callback conext (can be NULL)
    void (* p_callback)(can_callback_arg_t * p_args);
} can_ctrl_ins_t;

/* CAN control block.  Allocate an instance specific control block to pass into the CAN API calls. */
typedef void can_ctrl_t;

/** Shared Interface definition for CAN */
typedef struct st_can_api
{
    /** Open function for CAN device
     *
     * @param[in,out]  p_ctrl     Pointer to the CAN control block. Must be declared by user. Value set here.
     * @param[in]      p_cfg      Pointer to CAN configuration structure. All elements of this structure must be set by
     *                            user.
     */
    fsp_err_t (* open)(can_ctrl_t * const p_ctrl, can_cfg_t const * const p_cfg);

    /** Write function for CAN device
     * @param[in]   p_ctrl          Pointer to the CAN control block.
     * @param[in]   buffer          Buffer number (mailbox or message buffer) to write to.
     * @param[in]   p_frame         Pointer for frame of CAN ID, DLC, data and frame type to write.
     */
    fsp_err_t (* write)(can_ctrl_t * const p_ctrl, uint32_t mailbox, can_frame_t * const p_frame);

    /** Read function for CAN device
     * @param[in]   p_ctrl          Pointer to the CAN control block.
     * @param[in]   p_frame         Pointer to store the CAN ID, DLC, data and frame type.
     */
    fsp_err_t (* read)(can_ctrl_t * const p_ctrl, can_frame_t * const p_frame);

    /** Close function for CAN device
     * @param[in]   p_ctrl     Pointer to the CAN control block.
     */
    fsp_err_t (* close)(can_ctrl_t * const p_ctrl);

    /** Mode Transition function for CAN device
     * @param[in]   p_ctrl               Pointer to the CAN control block.
     * @param[in]   operation_mode       Destination CAN operation state.
     * @param[in]   test_mode            Destination CAN test state.
     */
    fsp_err_t (* modeTransition)(can_ctrl_t * const p_ctrl, can_operation_mode_t operation_mode, can_test_mode_t test_mode);

    /** Specify callback function and optional context pointer and working memory pointer.
     *
     * @param[in]   p_ctrl                   Control block set in @ref can_api_t::open call.
     * @param[in]   p_callback               Callback function to register
     * @param[in]   p_context                Pointer to send to callback function (if)
     * @param[in]   p_working_memory         Pointer to volatile memory where callback structure can be allocated.
     *                                       Callback arguments allocated here are only valid during the callback.
     */
    fsp_err_t (* callbackSet)(can_ctrl_t * const p_ctrl, void (* p_callback)(can_callback_arg_t *),
                              void const * const p_context, can_callback_arg_t * const p_callback_memory);
} can_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_can_instance
{
    can_ctrl_t      * p_ctrl;          ///< Pointer to the control structure for this instance
    can_cfg_t const * p_cfg;           ///< Pointer to the configuration structure for this instance
    can_api_t const * p_api;           ///< Pointer to the API structure for this instance
} can_instance_t;

/**********************************************************************************************************************
 * Extend global variables
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t HAL_CAN_Open(can_ctrl_t * const p_ctrl, can_cfg_t const * const p_cfg);
fsp_err_t HAL_CAN_Close(can_ctrl_t * const p_api_ctrl);
fsp_err_t HAL_CAN_Write(can_ctrl_t * const p_api_ctrl, uint32_t const mailbox, can_frame_t * const p_frame);
fsp_err_t HAL_CAN_Read(can_ctrl_t * const p_api_ctrl, can_frame_t * const p_frame);
fsp_err_t HAL_CAN_ModeTransition(can_ctrl_t * const   p_api_ctrl,
                               can_operation_mode_t operation_mode,
                               can_test_mode_t      test_mode);
fsp_err_t HAL_CAN_CallbackSet(can_ctrl_t * const          p_api_ctrl,
                            void (                    * p_callback)(can_callback_arg_t *),
                            void const * const          p_context,
                            can_callback_arg_t * const p_callback_memory);
#endif
