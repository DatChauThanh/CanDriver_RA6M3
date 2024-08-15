#include "CAN_Driver.h"
uint32_t can_mailbox_masks[2]{
    0x1FFFFFFF,
    0x1FFFFFFF
};

can_mailbox_t can_mailbox[32] =
{
  { .can_mailbox_id = 0, .can_id_mode = CAN_STANDARD_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_DATA, .can_mailbox_type =
        CAN_MAILBOX_TRANSMIT },
  { .can_mailbox_id = 1, .can_id_mode = CAN_STANDARD_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_REMOTE, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE },
  { .can_mailbox_id = 2, .can_id_mode = CAN_EXTENDED_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_DATA, .can_mailbox_type =
        CAN_MAILBOX_TRANSMIT, },
  { .can_mailbox_id = 3, .can_id_mode = CAN_EXTENDED_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_REMOTE, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE },
  { .can_mailbox_id = 4, .can_id_mode = CAN_STANDARD_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_DATA, .can_mailbox_type =
        CAN_MAILBOX_TRANSMIT },
  { .can_mailbox_id = 5, .can_id_mode = CAN_STANDARD_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_REMOTE, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE },
  { .can_mailbox_id = 6, .can_id_mode = CAN_EXTENDED_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_DATA, .can_mailbox_type =
        CAN_MAILBOX_TRANSMIT, },
  { .can_mailbox_id = 7, .can_id_mode = CAN_EXTENDED_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_REMOTE, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE }
};

can_fifo_options_t can_fifo_options{
    .can_fifo_irq_mode = (can_fifo_interrupt_mode_t)(CAN_FIFO_INTERRUPT_MODE_RX_EVERY_FRAME | CAN_FIFO_INTERRUPT_MODE_TX_EVERY_FRAME),
    .can_rx_fifo_mask1 = 0x1FFFFFFF,
    .can_rx_fifo_mask2 = 0x1FFFFFFF,
    .can_rx_fifo_id1 = { .can_mailbox_id = 20, .can_id_mode = CAN_STANDARD_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_DATA, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE },
    .can_rx_fifo_id2 = { .can_mailbox_id = 10, .can_id_mode = CAN_EXTENDED_ID_MODE, .can_frame_type = CAN_FRAME_TYPE_REMOTE, .can_mailbox_type =
        CAN_MAILBOX_RECEIVE },
    .can_tx_fifo_irq = ((IRQn_Type) 3),
    .can_rx_fifo_irq = ((IRQn_Type) 4),
};
can_irq_t can_irq{
    .can_error_irq = ((IRQn_Type) 0),
    .can_tx_irq = ((IRQn_Type) 1),
    .can_rx_irq = ((IRQn_Type) 2),
};

can_bit_timing_cfg_t can_bit_timing{
    .Baud_rate_prescaler = 5,
    .Time_segment_1 = 13,
    .Time_segment_2 = 4,
    .Synchronization_jump_width = 3
};
can_cfg_ctrl_t can_cfg_ctrl{
    .can_g_id_mode = CAN_MIXED_ID_MODE,
    .can_msg_lost_mode = CAN_OVERWRITE_MODE,
    .can_tx_priority = CAN_TRANSMIT_ID_PRIORITY,
    .can_timestamp_prescaler =CAN_TIMESTAMP_PRESCALER_8BITTIME,
    .can_busoff_recovery_mode = CAN_BUSOFF_RECOVERY_NORMAL
};

can_mailbox_options_t can_mailbox_options{
    .can_mailboxs_count = 8,
    .p_can_mailboxs_mask = &can_mailbox_masks[0],
    .p_can_mailboxs_cfg = &can_mailbox[0]
};
can_ctrl_ins_t can_ctrl_ins;
can_cfg_t g_can_cfg = {
    .can_clock_src = CAN_CLKSCR_PCLKB,
    .can_mailbox_mode = CAN_MAILBOX_FIFO_MODE,
    .can_channel = CAN_CHANNEL_0,
    .p_can_cfg_ctrl = &can_cfg_ctrl,
    .p_can_cfg_bit_timing = &can_bit_timing,
    .p_can_mailbox_options = &can_mailbox_options,
    .p_can_fifo_options =  &can_fifo_options,
    .can_irq_option = can_irq,
    .can_irq_priority = 1,
    .p_can_callback_context = NULL,
    .p_callback =NULL,
};
void can_callback(can_callback_arg_t *p_args){
    std::cout << "Call back func: Do something" << std::endl;
    std::cout << "Mailbox"<< p_args->can_mailbox << std::endl;
    std::cout << "Channel"<< p_args->can_channel << std::endl;
    std::cout << "CAN id"<< p_args->can_frame.can_frame_id << std::endl;
}