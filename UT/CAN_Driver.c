/*
 * CAN_Driver.c
 *
 *  Created on: 30 thg 7, 2024
 *      Author: dthanhchau
 */
/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "CAN_Driver.h"
/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define ENABLE                  (0)
#define CHECK_CAN_PARRAM        ENABLE
#define CHECK_NULL_POINTER(a)   (a != NULL)

#define CAN_SLEEP_AWAKE 0
#define CAN_SLEEP_SLEEP 1
#define CAN_TIMESTAMP_RESET  (0U)

#define CAN_BAUD_RATE_PRESCALER_MAX (0x3FF)
#define CAN_BAUD_RATE_PRESCALER_MIN (0x0)

#define CAN_CANM_SETTING_MSK (0x03)
#define CAN_STR_MODE_MSK     (0x700)

#define CAN_TCR_MODE_MSK (0x07)
#define CAN_CTLR_MODE_MSK (0x07)

#define CAN_MAX_NO_MAILBOXES (0x20)
#define CAN_MAX_NO_MAILBOXES_FIFO (0x18)
#define CAN_MAILBOX_RX    (0x40)
#define CAN_GROUP_MASK    (0xF)
#define CAN_GROUP_MAILBOX (0x04)        
#define CAN_SID_MASK      (0x000007FF) // Maximum Standard ID
#define CAN_XID_MASK      (0x1FFFFFFF) // Maximum Extend ID
#define CAN_MAILBOX_RX_COMPLETE (0x45)


#define CAN_FIFO_RX_MASK1  (0x06)
#define CAN_FIFO_RX_MASK2  (0x07)

#define CAN_TX_MAILBOX_READY                             (0x0U)
#define CAN_TX_FIFO_MAILBOX_FULL                         (0x1U)
#define CAN_TX_RX_INTERRUPTS_ENABLE                 (0xFFFFFFFFU)
#define CAN_TX_RX_ANOTHER_INTERRUPTS_ENABLE_FIFO    (0x00FFFFFFU)
/* Error-Warning | Error-Passive | Bus-Off Entry | Bus-Off Recovery | Overrun */
#define CAN_ERROR_INTERRUPTS_ENABLE         (0x3EU)
/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
void can_internal_transmition_mode(  can_ctrl_ins_t* p_ctrl,                       \
                                            const can_operation_mode_t can_operation_mode,\
                                            const can_test_mode_t can_test_mode);

void can_internal_switch_mode(can_ctrl_ins_t* p_ctrl,const can_operation_mode_t operation_mode);
 
void can_internal_mailboxs_config(can_ctrl_ins_t* p_ctrl, can_cfg_t const* const p_cfg);

void can_internal_fifo_support(can_ctrl_ins_t* p_ctrl, can_cfg_t const* const p_cfg);

void can_internal_single_mailbox_ID_set(can_cfg_t const* const p_cfg,
                                               volatile uint32_t * p_MB_id,
                                               can_mailbox_t * p_MB_data);

void can_internal_mailbox_read (can_ctrl_ins_t * p_ctrl, uint32_t mailbox, can_frame_t * p_frame);

void can_internal_call_callback (can_ctrl_ins_t * p_ctrl, can_callback_arg_t * p_args);

/***********************************************************************************************************************
 * ISR prototypes
 **********************************************************************************************************************/
void can_error_isr(void);

void can_rx_isr(void);

void can_tx_isr(void);

/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/
/* CAN function pointers   */
const can_api_t g_can_on_can =
{
    .open           = HAL_CAN_Open,
    .write          = HAL_CAN_Write,
    .read           = HAL_CAN_Read,
    .close          = HAL_CAN_Close,
    .modeTransition = HAL_CAN_ModeTransition,
    .callbackSet    = HAL_CAN_CallbackSet
};

/***********************************************************************************************************************
 * API Functions
 **********************************************************************************************************************/
fsp_err_t HAL_CAN_Open(can_ctrl_t * const p_ctrl_arg, can_cfg_t const * const p_cfg)
{
    can_ctrl_ins_t *const   p_ctrl = (can_ctrl_ins_t*) p_ctrl_arg;
#if CHECK_CAN_PARRAM
    /* Check pointer valid ? */
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg->p_can_cfg_bit_timing));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg->p_can_cfg_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg->p_can_fifo_options));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg->p_can_mailbox_options));
    
    /* Check re-init CAN ? */
    FSP_ERROR_RETURN(p_ctrl->can_open != CAN_OPEN, FSP_ERR_ALREADY_OPEN);

    /* Check baud Rate Pre-scaler */
    FSP_ERROR_RETURN(p_cfg->p_can_cfg_bit_timing->Baud_rate_prescaler <= CAN_BAUD_RATE_PRESCALER_MAX, FSP_ERR_CAN_INIT_FAILED);
    FSP_ERROR_RETURN(p_cfg->p_can_cfg_bit_timing->Baud_rate_prescaler >= CAN_BAUD_RATE_PRESCALER_MIN, FSP_ERR_CAN_INIT_FAILED);

    /* Check Bit timing configuration: TSEG1 > TSEG2 >= SJW
       More detail 37.4.2 Page 1389 
       Renesas RA6M3 Group User’s Manual: Hardware  (Rev.1.20 Feb 2023) */
    FSP_ERROR_RETURN(p_cfg->p_can_cfg_bit_timing->Time_segment_1 > p_cfg->p_can_cfg_bit_timing->Time_segment_2
                    ,FSP_ERR_CAN_INIT_FAILED);
    FSP_ERROR_RETURN(p_cfg->p_can_cfg_bit_timing->Time_segment_2 >= p_cfg->p_can_cfg_bit_timing->Synchronization_jump_width
                    ,FSP_ERR_CAN_INIT_FAILED);
    
    /* Check nums of mailbox are avaiable Max: 32 Mailbox*/ 
    FSP_ERROR_RETURN((p_cfg->p_can_mailbox_options->can_mailboxs_count % CAN_GROUP_MAILBOX) == 0
                    ,FSP_ERR_CAN_INIT_FAILED);

    if (p_cfg->can_mailbox_mode == CAN_MAILBOX_FIFO_MODE){
        FSP_ERROR_RETURN(p_cfg->p_can_mailbox_options->can_mailboxs_count <= CAN_MAX_NO_MAILBOXES_FIFO, FSP_ERR_CAN_INIT_FAILED);
    }
    else {
        FSP_ERROR_RETURN(p_cfg->p_can_mailbox_options->can_mailboxs_count <= CAN_MAX_NO_MAILBOXES, FSP_ERR_CAN_INIT_FAILED);       
    }
    
     /* Get the frequency of pclkb for later validation */
    uint32_t pclkb_frequency = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB);
    if(p_cfg->can_clock_src == CAN_CLKSCR_PCLKB){
        /*The source of the peripheral module clock must be PLL
          More detail 37.9.2 Page 1397 
          Renesas RA6M3 Group User’s Manual: Hardware  (Rev.1.20 Feb 2023)*/
        FSP_ERROR_RETURN(R_SYSTEM->SCKSCR == BSP_CLOCKS_SOURCE_CLOCK_PLL, FSP_ERR_CAN_INIT_FAILED);
    }
    else{
         /*fPCLKB >= fCANMCLK (clock constraint must be satisfied for the CAN module)s
          More detail 37.9.2 Page 1397 
          Renesas RA6M3 Group User’s Manual: Hardware  (Rev.1.20 Feb 2023)*/
        FSP_ERROR_RETURN(pclkb_frequency >= (BSP_CFG_XTAL_HZ), FSP_ERR_CAN_INIT_FAILED);
    }

#endif
    /*Assign CAN reg*/
#if CHECK_CAN_PARRAM
    R_CAN0_Type *p_reg  = (R_CAN0_Type *) ((uint32_t) R_CAN0 + \
                        (p_cfg->can_channel * ((uint32_t)R_CAN1 - (uint32_t)R_CAN0 )));
    p_ctrl->p_reg = p_reg;
   
#endif

    R_CAN0_Type *p_reg =p_ctrl->p_reg;
    /*Enable CAN module in Module Stop Control Register B (MSTPCRB)
      More Detail in 11.2.3 page 214 
      Renesas RA6M3 Group User’s Manual: Hardware  (Rev.1.20 Feb 2023)*/
    //R_BSP_MODULE_START(FSP_IP_CAN, p_cfg->can_channel);

    /*Setup control pointer with */
    p_ctrl->p_cfg               = p_cfg;
    p_ctrl->p_callback          = p_cfg->p_callback;
    p_ctrl->p_callback_context  = p_cfg->p_can_callback_context;
    p_ctrl->p_callback_memory   = nullptr;

    /*Set clock source*/
    p_ctrl->can_clock_src = p_cfg->can_clock_src;
    
    /*Switch reset mode to config CLTR and BTR*/
    can_internal_transmition_mode(p_ctrl,CAN_OPERATION_MODE_RESET, CAN_TEST_MODE_DISABLE);
    
    /*Set CTLR: MBM, IDFM, MLM, */
    WRITE_REG(p_reg->CTLR,(uint16_t)((p_cfg->can_mailbox_mode << R_CAN0_CTLR_MBM_Pos) |
                                     (p_cfg->p_can_cfg_ctrl->can_g_id_mode << R_CAN0_CTLR_IDFM_Pos) |
                                     (p_cfg->p_can_cfg_ctrl->can_msg_lost_mode << R_CAN0_CTLR_MLM_Pos) |
                                     (p_cfg->p_can_cfg_ctrl->can_tx_priority << R_CAN0_CTLR_TPM_Pos) |
                                     (p_cfg->p_can_cfg_ctrl->can_timestamp_prescaler << R_CAN0_CTLR_TSPS_Pos) |
                                     (p_ctrl->can_operation_mode << R_CAN0_CTLR_CANM_Pos)|
                                     (p_cfg->p_can_cfg_ctrl->can_busoff_recovery_mode << R_CAN0_CTLR_BOM_Pos))
            );

    /*Set BTR*/
    WRITE_REG(p_reg->BCR, (uint32_t)((p_cfg->p_can_cfg_bit_timing->Baud_rate_prescaler << R_CAN0_BCR_BRP_Pos) | 
                                     (p_cfg->p_can_cfg_bit_timing->Time_segment_1 << R_CAN0_BCR_TSEG1_Pos ) |
                                     (p_cfg->p_can_cfg_bit_timing->Time_segment_2 << R_CAN0_BCR_TSEG2_Pos ) |
                                     (p_cfg->p_can_cfg_bit_timing->Synchronization_jump_width << R_CAN0_BCR_SJW_Pos) |
                                     (p_cfg->can_clock_src << R_CAN0_BCR_CCLKS_Pos ))                                  
             );

    /*Set halt mode to config mailbox*/
    can_internal_transmition_mode(p_ctrl, CAN_OPERATION_MODE_HALT, CAN_TEST_MODE_DISABLE);

    /*Config Avaiable Mailboxs*/
    can_internal_mailboxs_config(p_ctrl, p_cfg);

    /*If FIFO supported => config fifo mailbox*/
    if (p_cfg->can_mailbox_mode == CAN_MAILBOX_FIFO_MODE)
    {
        can_internal_fifo_support(p_ctrl, p_cfg);
    }
    
    /* Set Error Display mode in ECSR in Halt mode. */
    SET_BIT(p_reg->ECSR,R_CAN0_ECSR_EDPM_Pos);

    /*Switch normal mode to config IRQ*/
    can_internal_transmition_mode(p_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLE);

    /* Time Stamp Counter reset in CAN Operation mode. */
    SET_BIT(p_reg->CTLR, R_CAN0_CTLR_TSRC_Pos);
    // FSP_HARDWARE_REGISTER_WAIT(p_reg->CTLR_b.TSRC, CAN_TIMESTAMP_RESET);

    // /*Enable IRQ*/
    // R_BSP_IrqCfgEnable(p_cfg->can_irq_option.can_error_irq, (uint32_t)(p_cfg->can_irq_priority), p_ctrl);
    // R_BSP_IrqCfgEnable(p_cfg->can_irq_option.can_tx_irq, (uint32_t)(p_cfg->can_irq_priority), p_ctrl);
    // R_BSP_IrqCfgEnable(p_cfg->can_irq_option.can_rx_irq, (uint32_t)(p_cfg->can_irq_priority), p_ctrl);

    /*If FIFO supported => config fifo mailbox*/
    if (p_cfg->can_mailbox_mode == CAN_MAILBOX_FIFO_MODE)
    {
        // R_BSP_IrqCfgEnable(p_cfg->p_can_fifo_options->can_tx_fifo_irq, (uint32_t)(p_cfg->can_irq_priority), p_ctrl);
        // R_BSP_IrqCfgEnable(p_cfg->p_can_fifo_options->can_rx_fifo_irq, (uint32_t)(p_cfg->can_irq_priority), p_ctrl);

        uint32_t rx_enable_mailboxs = (CAN_TX_RX_ANOTHER_INTERRUPTS_ENABLE_FIFO);
        WRITE_REG(p_reg->MIER_FIFO, (uint32_t)(p_cfg->p_can_fifo_options->can_fifo_irq_mode) | (rx_enable_mailboxs));
        
        /* Enable FIFO Transmit and Receive*/
        SET_BIT(p_reg->RFCR, R_CAN0_RFCR_RFE_Pos);
        SET_BIT(p_reg->TFCR, R_CAN0_TFCR_TFE_Pos);     
    }
    else {
        /* Set interrupt enable bits */
        WRITE_REG(p_reg->MIER, (uint32_t)CAN_TX_RX_INTERRUPTS_ENABLE);
    }

    /* Enable IRQ Error */
    WRITE_REG(p_reg->EIER, (uint8_t)CAN_ERROR_INTERRUPTS_ENABLE);

    /* If successful, Mark the control block as open */
    p_ctrl->can_open = CAN_OPEN;

    return  FSP_SUCCESS;
}

fsp_err_t HAL_CAN_Close(can_ctrl_t * const p_ctrl_arg)
{
    can_ctrl_ins_t*     p_ctrl = (can_ctrl_ins_t*) p_ctrl_arg;
    can_cfg_t const*    p_cfg  = p_ctrl->p_cfg;
    R_CAN0_Type*        p_reg  = p_ctrl->p_reg;

#if CHECK_CAN_PARRAM
    //Check null pointer and check is CAN module opened ?
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_reg));
    FSP_ERROR_RETURN(p_ctrl->can_open == CAN_OPEN, FSP_ERR_NOT_OPEN);
#endif
    // Close CAN
    p_ctrl->can_open = CAN_CLOSE;

    // //Disable IRQ error , transmit , receive
    // R_BSP_IrqDisable(p_cfg->can_irq_option.can_error_irq);
    // R_BSP_IrqDisable(p_cfg->can_irq_option.can_tx_irq);
    // R_BSP_IrqDisable(p_cfg->can_irq_option.can_rx_irq);

    //Disable IRQ FIFO transmit , receive
    if (p_cfg->can_mailbox_mode == CAN_MAILBOX_FIFO_MODE)
    {
        // R_BSP_IrqDisable(p_cfg->p_can_fifo_options->can_tx_fifo_irq);
        // R_BSP_IrqDisable(p_cfg->p_can_fifo_options->can_rx_fifo_irq);
        CLEAR_BIT(p_reg->RFCR, R_CAN0_RFCR_RFE_Pos);
        CLEAR_BIT(p_reg->TFCR, R_CAN0_TFCR_TFE_Pos);   
    }

    /*Disable tx, rx IRQ Trigger*/
    CLEAR_REG(p_reg->MIER);
    /*Disable ERROR IRQ Trigger*/
    CLEAR_REG(p_reg->EIER);

    /*Disable CAN Module via BSP MODULE STOP */
    //R_BSP_MODULE_STOP(FSP_IP_CAN, p_ctrl->p_cfg->can_channel);

    return FSP_SUCCESS;
}

fsp_err_t HAL_CAN_Write(can_ctrl_t * const p_ctrl_arg, uint32_t const mailbox, can_frame_t * const p_frame)
{
    can_ctrl_ins_t *    p_ctrl = (can_ctrl_ins_t *) p_ctrl_arg;
    can_cfg_t const*    p_cfg  = p_ctrl->p_cfg;
    R_CAN0_Type*        p_reg  = p_ctrl->p_reg;

#if CHECK_CAN_PARRAM
    //Check null pointer and check is CAN module opened ?
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_reg));
    FSP_ERROR_RETURN(p_ctrl->can_open == CAN_OPEN, FSP_ERR_NOT_OPEN);

    //Check Mailbox valid ?
    if (p_cfg->can_mailbox_mode == CAN_MAILBOX_FIFO_MODE){
        /* Check FIFO TX mailbox is selected if transmit in FIFO mode?*/
        FSP_ERROR_RETURN(mailbox == CAN_MAILBOX_ID_TX_FIFO, FSP_ERR_INVALID_ARGUMENT);
        /* Check FIFO mailbox is full ? */  
        FSP_ERROR_RETURN(p_reg->TFCR_b.TFFST == CAN_TX_FIFO_MAILBOX_FULL , FSP_ERR_CAN_TRANSMIT_FIFO_FULL);
    }
    else{
        /* Check normal MB is ready */
        FSP_ERROR_RETURN(mailbox < p_cfg->p_can_mailbox_options->can_mailboxs_count, FSP_ERR_INVALID_ARGUMENT);
        FSP_ERROR_RETURN(p_reg->MCTL_TX_b->TRMREQ == CAN_TX_MAILBOX_READY , FSP_ERR_CAN_TRANSMIT_FIFO_FULL);
    }
    
    //Check data length p_frame
    FSP_ERROR_RETURN(p_frame->can_frame_dlc <= CAN_DATA_PAYLOAD, FSP_ERR_INVALID_ARGUMENT);
    // Check CAN Operation mode
    FSP_ERROR_RETURN(p_ctrl->can_operation_mode == CAN_OPERATION_MODE_NORMAL, FSP_ERR_INVALID_MODE);
#endif
    
    /* Set ID Mailbox */
    can_internal_single_mailbox_ID_set(p_cfg, &p_reg->MB[mailbox].ID , (can_mailbox_t *)p_frame );
   
    /* Set Data Length */
    WRITE_REG(p_reg->MB[mailbox].DL, (uint16_t)p_frame->can_frame_dlc);

    /* Set Data to Mailbox */
    for (uint8_t i = 0; i < p_frame->can_frame_dlc; i++)
    {
        WRITE_REG(p_reg->MB[mailbox].D[i], (uint8_t)(p_frame->can_frame_data[i]));
    }

    /* Trigger Mailbox Transmt Request */
    if (mailbox == CAN_MAILBOX_ID_TX_FIFO){
        WRITE_REG(p_reg->TFPCR, (uint8_t)CAN_FIFO_CPU_POINTER);
    }
    else{
        SET_BIT(p_reg->MCTL_TX[mailbox],R_CAN0_MCTL_TX_TRMREQ_Pos);
    }

    return FSP_SUCCESS;
}

fsp_err_t HAL_CAN_Read(can_ctrl_t * const p_ctrl_arg, can_frame_t * const p_frame)
{
    can_ctrl_ins_t*     p_ctrl = (can_ctrl_ins_t*) p_ctrl_arg;
    can_cfg_t const*    p_cfg  = p_ctrl->p_cfg;
    R_CAN0_Type*        p_reg  = p_ctrl->p_reg;
    uint32_t            mailbox;

#if CHECK_CAN_PARRAM
    //Check null pointer and check is CAN module opened ?
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_reg));
    FSP_ERROR_RETURN(p_ctrl->can_open == CAN_OPEN, FSP_ERR_NOT_OPEN);
    // Check CAN Operation mode
    FSP_ERROR_RETURN(p_ctrl->can_operation_mode == CAN_OPERATION_MODE_NORMAL, FSP_ERR_INVALID_MODE);
#endif

    /* Search Which Mailbox was received msg*/
    uint8_t Saved_msmr = p_reg->MSMR;               // Save the current MSMR value
    WRITE_REG(p_reg->MSMR , (uint8_t)CAN_SEARCHMODE_RECEIVE_MAILBOX);   // Search for lowest numbered &mailbox with message received
    if (p_reg->MSSR_b.SEST == 1)
    {
        return FSP_ERR_CAN_DATA_UNAVAILABLE;
    }
    mailbox = p_reg->MSSR_b.MBNST;                  // get mailbox number
    WRITE_REG(p_reg->MSMR , (uint8_t)Saved_msmr);   // Restore the previous MSMR value
    can_internal_mailbox_read(p_ctrl, mailbox, p_frame);

    return FSP_SUCCESS;
}

fsp_err_t HAL_CAN_ModeTransition (  can_ctrl_t * const   p_ctrl_arg,
                                    can_operation_mode_t operation_mode,
                                    can_test_mode_t      test_mode      )
{
    can_ctrl_ins_t * p_ctrl = (can_ctrl_ins_t *) p_ctrl_arg;
    fsp_err_t        err    = FSP_SUCCESS;

#if CHECK_CAN_PARRAM
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl->p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl->p_reg));
    FSP_ERROR_RETURN(p_ctrl->can_open == CAN_OPEN, FSP_ERR_NOT_OPEN);
#endif

    can_internal_transmition_mode(p_ctrl, operation_mode, test_mode);

    return err;
}

fsp_err_t HAL_CAN_CallbackSet(  can_ctrl_t * const          p_ctrl_arg,
                                void (                    * p_callback)(can_callback_arg_t *),
                                void const * const          p_context,
                                can_callback_arg_t * const p_callback_memory)
{
    can_ctrl_ins_t * p_ctrl = (can_ctrl_ins_t *) p_ctrl_arg;
    fsp_err_t        err    = FSP_SUCCESS;

#if CHECK_CAN_PARRAM
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl));
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl->p_cfg));
    FSP_ASSERT(CHECK_NULL_POINTER(p_ctrl->p_reg));
    FSP_ERROR_RETURN(p_ctrl->can_open == CAN_OPEN, FSP_ERR_NOT_OPEN);
#endif
    /* Set Callback */
    p_ctrl->p_callback          = p_callback;
    p_ctrl->p_callback_context  = p_context;
    p_ctrl->p_callback_memory   = p_callback_memory;
    return err;
}
#if CHECK_CAN_PARRAM
/***********************************************************************************************************************
 * IRQ Functions
 **********************************************************************************************************************/
void can_error_isr (void)
{
    /* Get IRQ Local Variable */
    R_BSP_IrqStatusClear(R_FSP_CurrentIrqGet());
    IRQn_Type           irq     = R_FSP_CurrentIrqGet();
    can_ctrl_ins_t *    p_ctrl  = (can_ctrl_ins_t *) R_FSP_IsrContextGet(irq);
    R_CAN0_Type *       p_reg   = p_ctrl->p_reg;
    can_callback_arg_t  args    = {0U};
    uint32_t            mailbox = 0U;
 

    /* Get source of error interrupt */
    args.can_event = (can_event_t) p_reg->EIFR; // Read Error Interrupt Factor Judge register
    CLEAR_REG(p_reg->EIFR);
    
    /* If message lost Normal Mailbox*/
    if (p_reg->STR_b.NMLST)
    {
        args.can_event |= CAN_EVENT_MAILBOX_MESSAGE_LOST;     /// Set Event
        uint8_t saved_msmr = p_reg->MSMR;                     /// Save the current MSMR value
        WRITE_REG(p_reg->MSMR, CAN_SEARCHMODE_MESSAGE_LOST);  /// search for lowest numbered mailbox with message lost
        mailbox     = p_reg->MSSR_b.MBNST;                    /// get mailbox number
        WRITE_REG(p_reg->MSMR, saved_msmr);                   /// Restore the previous MSMR value
        p_reg->MCTL_RX_b[mailbox].MSGLOST = 0U;               /// Clear the error so that NMLST is not set again for 
                                                              // an already handled error.
    }
    
    /* Set arg callback*/
    args.can_channel = p_ctrl->p_cfg->can_channel;
    args.can_mailbox = mailbox;
    
    can_internal_call_callback(p_ctrl, &args);

    /* Check for mailboxes with data loss due to overrun,
       if true, fire this interrupt again.
       Do not re-trigger this interrupt due to message loss in overwrite mode */
    if (p_reg->STR_b.NMLST && (CAN_OVERWRITE_MODE != p_ctrl->p_cfg->p_can_cfg_ctrl->can_msg_lost_mode))
    {
        NVIC_SetPendingIRQ(p_ctrl->p_cfg->can_irq_option.can_error_irq);
    }
}

void can_tx_isr(void)
{
    /* Get IRQ Local Variable */
    R_BSP_IrqStatusClear(R_FSP_CurrentIrqGet());
    IRQn_Type           irq      = R_FSP_CurrentIrqGet();
    can_ctrl_ins_t *    p_ctrl   = (can_ctrl_ins_t *) R_FSP_IsrContextGet(irq);
    R_CAN0_Type *       p_reg    = p_ctrl->p_reg;
    can_callback_arg_t  arg      = {0U};
    uint32_t            mailbox  = 0U;
    
    /* Set CAN channel */
    arg.can_channel = p_ctrl->p_cfg->can_channel;

    /* Find which mailbox occur IRQ */
    if (irq == p_ctrl->p_cfg->p_can_fifo_options->can_tx_fifo_irq)
    {
        arg.can_mailbox = CAN_MAILBOX_ID_TX_FIFO;
        if(p_ctrl->p_cfg->p_can_fifo_options->can_fifo_irq_mode & R_CAN0_MIER_FIFO_MB25_Msk)
        {
            /* When MB25 is set the TX FIFO ISR only trigger when the last message has been transmitted. */
            arg.can_event = CAN_EVENT_TX_FIFO_EMPTY;
        }
    }
    else{
        uint8_t saved_msmr = p_reg->MSMR;                                    // Save the current MSMR value
        WRITE_REG(p_reg->MSMR, (uint8_t)CAN_SEARCHMODE_TRANSMIT_MAILBOX);    // search for lowest numbered mailbox with message lost
        mailbox     = p_reg->MSSR_b.MBNST;                                   // get mailbox number
        WRITE_REG(p_reg->MSMR, saved_msmr);                                  // Restore the previous MSMR value

        WRITE_REG(p_reg->MCTL_TX[mailbox],(uint8_t)CAN_TX_MAILBOX_READY);    // Clear SENTDATA and TRMREQ.
        arg.can_mailbox = mailbox;
    }
    /*  Set event argument to transmit complete. */
    arg.can_event |= CAN_EVENT_TX_COMPLETE;
    can_internal_call_callback(p_ctrl, &arg);

    /* Check for other mailboxes with pending transmit complete flags. */
    if ((irq != p_ctrl->p_cfg->p_can_fifo_options->can_tx_fifo_irq) && p_reg->STR_b.SDST)
    {
        NVIC_SetPendingIRQ(p_ctrl->p_cfg->can_irq_option.can_tx_irq);
    }
}

void can_rx_isr(void)
{
    R_BSP_IrqStatusClear(R_FSP_CurrentIrqGet());
    IRQn_Type             irq    = R_FSP_CurrentIrqGet();
    can_ctrl_ins_t * p_ctrl      = (can_ctrl_ins_t *) R_FSP_IsrContextGet(irq);
    R_CAN0_Type * p_reg          = p_ctrl->p_reg;
    can_callback_arg_t  arg      = {0U};
    uint32_t mailbox             = 0U;
     
    /* Set CAN channel */
    arg.can_channel = p_ctrl->p_cfg->can_channel;
    /* Find which mailbox occur IRQ */
    if (irq == p_ctrl->p_cfg->p_can_fifo_options->can_rx_fifo_irq){
        mailbox = CAN_MAILBOX_ID_RX_FIFO;
        if(p_reg->RFCR & R_CAN0_RFCR_RFMLF_Msk)
        {
           /* Clear the Receive FIFO Message Lost Flag if set   */
            CLEAR_BIT(p_reg->RFCR, R_CAN0_RFCR_RFMLF_Pos);

            /* Add a FIFO message lost flag to the event */
            arg.can_event |= CAN_EVENT_FIFO_MESSAGE_LOST;
        }
    }
    else{
        uint8_t saved_msmr = p_reg->MSMR;                       // Save the current MSMR value
        WRITE_REG(p_reg->MSMR, CAN_SEARCHMODE_RECEIVE_MAILBOX); // search for lowest numbered &mailbox with message received
        mailbox     = p_reg->MSSR_b.MBNST;                      // get mailbox number
        WRITE_REG(p_reg->MSMR, saved_msmr);                     // Restore the previous MSMR value
    }

    can_internal_mailbox_read(p_ctrl, mailbox, &arg.can_frame);

    arg.can_mailbox = mailbox;
    arg.can_channel = p_ctrl->p_cfg->can_channel;

    can_internal_call_callback(p_ctrl,&arg);

    if (irq == p_ctrl->p_cfg->p_can_fifo_options->can_rx_fifo_irq && !p_reg->RFCR_b.RFEST)
    {
        NVIC_SetPendingIRQ(irq);
    }
    else{
        if (p_reg->STR_b.NDST)
        {
            /* Reset MB IRQ if another MB has requested it. */
            NVIC_SetPendingIRQ(irq);
        }
    }
    
}
#endif
/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
 void can_internal_transmition_mode(    can_ctrl_ins_t* p_ctrl,
                                        const can_operation_mode_t operation_mode,
                                        const can_test_mode_t test_mode)
{
    /*Is test mode enable*/
    if (test_mode != CAN_TEST_MODE_DISABLE)
    {
        can_internal_switch_mode(p_ctrl, CAN_OPERATION_MODE_HALT);
        /* Write to TCR reg in halt mode only */
        WRITE_REG(p_ctrl->p_reg->TCR ,(uint8_t)(test_mode & CAN_TCR_MODE_MSK));
        
        p_ctrl->can_test_mode      = test_mode;
        p_ctrl->can_operation_mode = CAN_OPERATION_MODE_HALT;
    }
    
    if (p_ctrl->can_operation_mode != operation_mode)
    {
        if(operation_mode == CAN_OPERATION_MODE_SLEEP){
            can_internal_switch_mode(p_ctrl, CAN_OPERATION_MODE_HALT);
            /* Set SLPM bit in halt or reset mode */
            SET_BIT(p_ctrl->p_reg->CTLR, R_CAN0_CTLR_SLPM_Pos);
            //FSP_HARDWARE_REGISTER_WAIT(p_ctrl->p_reg->STR_b.SLPST, CAN_SLEEP_SLEEP);
        }
        else{
            can_internal_switch_mode(p_ctrl, operation_mode);
        }
        p_ctrl->can_operation_mode = operation_mode;
    }
}

void can_internal_switch_mode(can_ctrl_ins_t* p_ctrl,const can_operation_mode_t operation_mode)
{
    /* Clear bit SLPM*/
    CLEAR_BIT(p_ctrl->p_reg->CTLR, R_CAN0_CTLR_SLPM_Pos);
    /* Set CANM*/
    p_ctrl->p_reg->CTLR_b.CANM = (uint16_t) (operation_mode & CAN_CANM_SETTING_MSK);

    // FSP_HARDWARE_REGISTER_WAIT((p_ctrl->p_reg->STR & CAN_STR_MODE_MSK),
    //                            (uint16_t) (operation_mode << R_CAN0_STR_RSTST_Pos)); 
}

 void can_internal_fifo_support(can_ctrl_ins_t* p_ctrl, can_cfg_t const* const p_cfg)
{
    R_CAN0_Type* p_reg = p_ctrl->p_reg;
    /* Get pointer to FIFO configuration */
    can_fifo_options_t* p_fifo_options = p_cfg->p_can_fifo_options;

    /*Set mask compare*/
    uint32_t mask1 = p_fifo_options->can_rx_fifo_mask1;
    uint32_t mask2 = p_fifo_options->can_rx_fifo_mask2;
    if(p_cfg->p_can_cfg_ctrl->can_g_id_mode == CAN_STANDARD_ID_MODE)
    {
       WRITE_REG(p_reg->MKR[CAN_FIFO_RX_MASK1], (mask1 << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
       WRITE_REG(p_reg->MKR[CAN_FIFO_RX_MASK2], (mask2 << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    }
    else
    {
       WRITE_REG(p_reg->MKR[CAN_FIFO_RX_MASK1], (mask1));
       WRITE_REG(p_reg->MKR[CAN_FIFO_RX_MASK2], (mask2));
    }

    /*Set ID compare*/
    can_internal_single_mailbox_ID_set(p_cfg, &p_reg->FIDCR[0], &p_fifo_options->can_rx_fifo_id1);
    can_internal_single_mailbox_ID_set(p_cfg, &p_reg->FIDCR[1], &p_fifo_options->can_rx_fifo_id2);

}

 void can_internal_mailboxs_config(can_ctrl_ins_t* p_ctrl, can_cfg_t const* const p_cfg)
{
    R_CAN0_Type* p_reg = p_ctrl->p_reg;
    uint8_t num_of_mailboxs = p_cfg->p_can_mailbox_options->can_mailboxs_count;
    uint32_t mask_enabled = 0;
    
     /* Clear all mailboxes in Halt mode (FIFO mailboxes are cleared automatically). */
    memset((void*) p_reg->MB, 0 ,sizeof(R_CAN0_MB_Type)*num_of_mailboxs);

    /* Set MKR*/
    for (uint8_t i = 0; i < (num_of_mailboxs / CAN_GROUP_MAILBOX); i++)
    {
        uint32_t local_mask =  (uint32_t)(p_cfg->p_can_mailbox_options->p_can_mailboxs_mask[i]);
        if(p_cfg->p_can_cfg_ctrl->can_g_id_mode == CAN_STANDARD_ID_MODE)
        {
            local_mask &= CAN_SID_MASK;
            local_mask =  local_mask << R_CAN0_MB_ID_SID_Pos;
        }
        else
        {
            local_mask &= CAN_XID_MASK;
        }
        WRITE_REG(p_reg->MKR[i], local_mask);

        /* Enable the mask for this group */
        mask_enabled |= CAN_GROUP_MASK << (i << 2);
    }
    /* Set mask valid in MKIVLR */
    WRITE_REG(p_reg->MKIVLR, ~mask_enabled);

    /* Set Mailboxs ID and type of Mailboxs*/
    for (uint8_t i = 0; i < (num_of_mailboxs); i++)
    {
        can_internal_single_mailbox_ID_set(p_cfg ,&p_reg->MB[i].ID, &p_cfg->p_can_mailbox_options->p_can_mailboxs_cfg[i]);
        if (p_cfg->p_can_mailbox_options->p_can_mailboxs_cfg[i].can_mailbox_type == CAN_MAILBOX_RECEIVE ){
            WRITE_REG(p_reg->MCTL_RX[i], (uint8_t) CAN_MAILBOX_RX);
        }
        else{
            CLEAR_REG(p_reg->MCTL_RX[i]);
        }
    }
}

 void can_internal_single_mailbox_ID_set(can_cfg_t const* const p_cfg, volatile uint32_t * p_MB_id ,can_mailbox_t * p_MB_data)
{ 
    can_id_mode_t global_id_mode = p_cfg->p_can_cfg_ctrl->can_g_id_mode;
    uint32_t local_id = p_MB_data->can_mailbox_id;
    if(global_id_mode == CAN_STANDARD_ID_MODE || ((global_id_mode == CAN_MIXED_ID_MODE) && (p_MB_data->can_id_mode == CAN_STANDARD_ID_MODE)) )
    {
        local_id = (local_id << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk;
    }
    if(global_id_mode == CAN_EXTENDED_ID_MODE || ((global_id_mode == CAN_MIXED_ID_MODE) && (p_MB_data->can_id_mode == CAN_EXTENDED_ID_MODE)) )
    {
        local_id = (local_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk;
    }
     
    WRITE_REG(*p_MB_id,(uint32_t)((p_MB_data->can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk) |
                                 ((p_MB_data->can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk) |
                                 (local_id)
            );
}

 void can_internal_mailbox_read (can_ctrl_ins_t * p_ctrl, uint32_t mailbox, can_frame_t * p_frame)
{
    R_CAN0_Type* p_reg = p_ctrl->p_reg;
    uint32_t mailbox_id = p_reg->MB[mailbox].ID;

    if(p_reg->MB[mailbox].DL > CAN_DATA_PAYLOAD){
        p_frame->can_frame_dlc = (uint8_t) CAN_DATA_PAYLOAD;
    }
    else{
        p_frame->can_frame_dlc = (uint8_t) (p_reg->MB[mailbox].DL);
    }
    /* Get the frame type */
    p_frame->can_frame_type = (can_frame_type_t) ((mailbox_id & R_CAN0_MB_ID_RTR_Msk) >> R_CAN0_MB_ID_RTR_Pos);
    /* Get the ID mode */
    p_frame->can_frame_id_mode = (can_id_mode_t) ((mailbox_id & R_CAN0_MB_ID_IDE_Msk) >> R_CAN0_MB_ID_IDE_Pos);

    if (p_frame->can_frame_id_mode == CAN_STANDARD_ID_MODE){
        p_frame->can_frame_id = (mailbox_id & R_CAN0_MB_ID_SID_Msk) >> R_CAN0_MB_ID_SID_Pos;
    }
    else{
        p_frame->can_frame_id = (mailbox_id & (R_CAN0_MB_ID_EID_Msk | R_CAN0_MB_ID_SID_Msk)) >> R_CAN0_MB_ID_EID_Pos;
    }
    /*Get Data Received*/
    for(uint8_t i = 0; i <  p_frame->can_frame_dlc ;i++)
    {
        (p_frame->can_frame_data[i]) = (uint8_t)(p_reg->MB[mailbox].D[i]); 
    }
    
    if(mailbox == CAN_MAILBOX_ID_RX_FIFO){
        WRITE_REG(p_reg->RFPCR, (uint8_t)CAN_FIFO_CPU_POINTER);
    }
    else{
        /*Clear MCTL_RX Reg
          Write 1 to the NEWDATA and MSGLOST bits if they are not the write target
          More Detail in 37.2.10 page 1367 
          Renesas RA6M3 Group User’s Manual: Hardware  (Rev.1.20 Feb 2023) */
        WRITE_REG(p_reg->MCTL_RX[mailbox], (uint8_t)CAN_MAILBOX_RX_COMPLETE);
    }

}

 void can_internal_call_callback (can_ctrl_ins_t * p_ctrl, can_callback_arg_t * p_args)
{
    can_callback_arg_t Local_arg;
    
    /*Check callback memnory in p_ctrl*/
    if(p_ctrl->p_callback_memory == NULL)
    {   
        p_ctrl->p_callback_memory = p_args;
    }
    else{
        /*Save previous callback memory*/
        Local_arg = *(p_ctrl->p_callback_memory);
        /*Assign current callback arg*/
        p_ctrl->p_callback_memory = p_args;
    }

    p_ctrl->p_callback(p_ctrl->p_callback_memory);
     
    if (CHECK_NULL_POINTER(p_ctrl->p_callback_memory))
    {    /* Restore callback memory */
        std::cout << "Restore callback memory"<< p_args->can_channel << std::endl;
        *(p_ctrl->p_callback_memory) = Local_arg;
    }
}