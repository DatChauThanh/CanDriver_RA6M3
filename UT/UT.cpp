#include <gtest/gtest.h>
#include <gmock/gmock.h>
extern "C"{
    #include "CAN_Driver.c"
    #include "Driver_StubData.h"
}

TEST(CAN_Driver_Func_can_internal_switch_mode, case1){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_HALT;
    //Act
    can_internal_switch_mode(&p_ctrl,operation_mode);
    //Assert
    ASSERT_EQ(p_ctrl.p_reg->CTLR & (CAN_CANM_SETTING_MSK << 8), (uint16_t)(operation_mode << 8));
}
TEST(CAN_Driver_Func_can_internal_switch_mode, case2){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_NORMAL;
    //Act
    can_internal_switch_mode(&p_ctrl,operation_mode);
    //Assert
    ASSERT_EQ(p_ctrl.p_reg->CTLR  & (CAN_CANM_SETTING_MSK << 8), (uint16_t)(operation_mode << 8));
}
TEST(CAN_Driver_Func_can_internal_switch_mode, case3){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_RESET;
    //Act
    can_internal_switch_mode(&p_ctrl,operation_mode);
    //Assert
    ASSERT_EQ(p_ctrl.p_reg->CTLR & (CAN_CANM_SETTING_MSK << 8) , (uint16_t)(operation_mode << 8));
}

TEST(CAN_Driver_Func_can_internal_transmition_mode,case1){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR = 0x0600;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_SLEEP;
    can_test_mode_t test_mode = CAN_TEST_MODE_DISABLE;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8) , (uint16_t)((operation_mode + 1) << 8));
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}
TEST(CAN_Driver_Func_can_internal_transmition_mode,case2){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR = 0x0600;
    p_ctrl.can_operation_mode = CAN_OPERATION_MODE_SLEEP;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_NORMAL;
    can_test_mode_t test_mode = CAN_TEST_MODE_DISABLE;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8) , (uint16_t)((operation_mode) << 8));
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}
TEST(CAN_Driver_Func_can_internal_transmition_mode,case3){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR_b.CANM = CAN_OPERATION_MODE_HALT;
    p_ctrl.can_operation_mode = CAN_OPERATION_MODE_HALT;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_SLEEP;
    can_test_mode_t test_mode = CAN_TEST_MODE_DISABLE;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8), (uint16_t)((operation_mode + 1) << 8));
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}
TEST(CAN_Driver_Func_can_internal_transmition_mode,case4){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR_b.CANM = CAN_OPERATION_MODE_NORMAL;
    p_ctrl.can_operation_mode = CAN_OPERATION_MODE_NORMAL;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_HALT;
    can_test_mode_t test_mode = CAN_TEST_MODE_LISTEN_ONLY;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8), (uint16_t)((operation_mode) << 8));
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}
TEST(CAN_Driver_Func_can_internal_transmition_mode,case5){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR_b.CANM = CAN_OPERATION_MODE_NORMAL;
    p_ctrl.can_operation_mode = CAN_OPERATION_MODE_NORMAL;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_SLEEP;
    can_test_mode_t test_mode = CAN_TEST_MODE_EXTERNAL_LOOPBACK;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8), (uint16_t)((operation_mode+1) << 8));//operation_mode+1 = Sleep_mode
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}
TEST(CAN_Driver_Func_can_internal_transmition_mode,case6){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    p_ctrl.p_reg->CTLR = 0x0600;
    p_ctrl.can_operation_mode = CAN_OPERATION_MODE_SLEEP;
    can_operation_mode_t operation_mode = CAN_OPERATION_MODE_NORMAL ;
    can_test_mode_t test_mode = CAN_TEST_MODE_INTERNAL_LOOPBACK;
    //Act
    can_internal_transmition_mode(&p_ctrl,operation_mode,test_mode);
    //Assert
    EXPECT_EQ(p_ctrl.p_reg->CTLR & (CAN_CTLR_MODE_MSK << 8), (uint16_t)((operation_mode) << 8));
    EXPECT_EQ(p_ctrl.p_reg->TCR & (CAN_TCR_MODE_MSK) , (uint8_t)(test_mode));
    EXPECT_EQ(p_ctrl.can_operation_mode,operation_mode);
    EXPECT_EQ(p_ctrl.can_test_mode,test_mode);
}

TEST(CAN_Driver_Func_can_internal_single_mailbox_ID_set,case1){
    //Arrange
    volatile uint32_t reg =0x00000000;
    can_mailbox_t can_mailbox=
    { 
        .can_mailbox_id = 0x500, 
        .can_id_mode = CAN_STANDARD_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_REMOTE, 
        .can_mailbox_type = CAN_MAILBOX_RECEIVE
    };
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_STANDARD_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl};
    //Act
    can_internal_single_mailbox_ID_set(&can_cfg, &reg , &can_mailbox);
    //Assert
    EXPECT_EQ(reg & R_CAN0_MB_ID_SID_Msk,(can_mailbox.can_mailbox_id << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_RTR_Msk,(can_mailbox.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_IDE_Msk,(can_mailbox.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}
TEST(CAN_Driver_Func_can_internal_single_mailbox_ID_set,case2){
    //Arrange
    volatile uint32_t reg =0x00000000;
    can_mailbox_t can_mailbox=
    { 
        .can_mailbox_id = 0x3839, 
        .can_id_mode = CAN_EXTENDED_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_DATA, 
        .can_mailbox_type = CAN_MAILBOX_RECEIVE
    };
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_EXTENDED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl};
    //Act
    can_internal_single_mailbox_ID_set(&can_cfg, &reg , &can_mailbox);
    //Assert
    EXPECT_EQ(reg & R_CAN0_MB_ID_EID_Msk,(can_mailbox.can_mailbox_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_RTR_Msk,(can_mailbox.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_IDE_Msk,(can_mailbox.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}
TEST(CAN_Driver_Func_can_internal_single_mailbox_ID_set,case3){
    //Arrange
    volatile uint32_t reg =0x00000000;
    can_mailbox_t can_mailbox=
    { 
        .can_mailbox_id = 10, 
        .can_id_mode = CAN_STANDARD_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_DATA, 
        .can_mailbox_type = CAN_MAILBOX_RECEIVE
    };
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_MIXED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl};
    //Act
    can_internal_single_mailbox_ID_set(&can_cfg, &reg , &can_mailbox);
    //Assert
    EXPECT_EQ(reg & R_CAN0_MB_ID_SID_Msk,(can_mailbox.can_mailbox_id << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_RTR_Msk,(can_mailbox.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_IDE_Msk,(can_mailbox.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}
TEST(CAN_Driver_Func_can_internal_single_mailbox_ID_set,case4){
    //Arrange
    volatile uint32_t reg =0x00000000;
    can_mailbox_t can_mailbox=
    { 
        .can_mailbox_id = 0x2829, 
        .can_id_mode = CAN_EXTENDED_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_REMOTE, 
        .can_mailbox_type = CAN_MAILBOX_RECEIVE
    };
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_MIXED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl};
    //Act
    can_internal_single_mailbox_ID_set(&can_cfg, &reg , &can_mailbox);
    //Assert
    EXPECT_EQ(reg & R_CAN0_MB_ID_EID_Msk,(can_mailbox.can_mailbox_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_RTR_Msk,(can_mailbox.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(reg & R_CAN0_MB_ID_IDE_Msk,(can_mailbox.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}

TEST(CAN_Driver_Func_can_internal_mailboxs_config,case1){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    //Act
    can_internal_mailboxs_config(&p_ctrl, &g_can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[0], can_mailbox_masks[0]);
    EXPECT_EQ(p_reg.MKR[1], can_mailbox_masks[1]);
    EXPECT_EQ(p_reg.MKIVLR, ~((uint32_t)0xFF));
    EXPECT_EQ(p_reg.MCTL_RX[3], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_EQ(p_reg.MCTL_RX[6], (uint8_t)0x0);
    EXPECT_NE(p_reg.MCTL_RX[4], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_NE(p_reg.MCTL_RX[1], (uint8_t)0x0);
}
TEST(CAN_Driver_Func_can_internal_mailboxs_config,case2){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_STANDARD_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl,
                         .p_can_mailbox_options = &can_mailbox_options};
    //Act
    can_internal_mailboxs_config(&p_ctrl, &can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[0], CAN_SID_MASK << R_CAN0_MB_ID_SID_Pos);
    EXPECT_EQ(p_reg.MKR[1], CAN_SID_MASK << R_CAN0_MB_ID_SID_Pos);
    EXPECT_EQ(p_reg.MKIVLR, ~((uint32_t)0xFF));
    EXPECT_EQ(p_reg.MCTL_RX[3], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_EQ(p_reg.MCTL_RX[6], (uint8_t)0x0);
    EXPECT_NE(p_reg.MCTL_RX[4], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_NE(p_reg.MCTL_RX[1], (uint8_t)0x0);
}
TEST(CAN_Driver_Func_can_internal_mailboxs_config,case3){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_EXTENDED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl,
                         .p_can_mailbox_options = &can_mailbox_options};
    //Act
    can_internal_mailboxs_config(&p_ctrl, &can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[0], CAN_XID_MASK);
    EXPECT_EQ(p_reg.MKR[1], CAN_XID_MASK);
    EXPECT_EQ(p_reg.MKIVLR, ~((uint32_t)0xFF));
    EXPECT_EQ(p_reg.MCTL_RX[3], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_EQ(p_reg.MCTL_RX[6], (uint8_t)0x0);
    EXPECT_NE(p_reg.MCTL_RX[4], (uint8_t)CAN_MAILBOX_RX);
    EXPECT_NE(p_reg.MCTL_RX[1], (uint8_t)0x0);
}

TEST(CAN_Driver_Func_can_internal_fifo_support,case1){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_STANDARD_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl,
                         .p_can_fifo_options = &can_fifo_options};
    //Act
    can_internal_fifo_support(&p_ctrl, &can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK1],CAN_SID_MASK << R_CAN0_MB_ID_SID_Pos);
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK2],CAN_SID_MASK << R_CAN0_MB_ID_SID_Pos);

    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_SID_Msk,(can_fifo_options.can_rx_fifo_id1.can_mailbox_id<< R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id1.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id1.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);

    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_SID_Msk,(can_fifo_options.can_rx_fifo_id2.can_mailbox_id<< R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id2.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id2.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}
TEST(CAN_Driver_Func_can_internal_fifo_support,case2){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_EXTENDED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl,
                         .p_can_fifo_options = &can_fifo_options};
    //Act
    can_internal_fifo_support(&p_ctrl, &can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK1],CAN_XID_MASK);
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK2],CAN_XID_MASK);
    
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_EID_Msk,(can_fifo_options.can_rx_fifo_id1.can_mailbox_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id1.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id1.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);

    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_EID_Msk,(can_fifo_options.can_rx_fifo_id2.can_mailbox_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id2.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id2.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}
TEST(CAN_Driver_Func_can_internal_fifo_support,case3){
    //Arrange
    R_CAN0_Type p_reg{};
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    can_cfg_ctrl_t fake_can_cfg_ctrl = {.can_g_id_mode = CAN_MIXED_ID_MODE};
    can_cfg_t can_cfg = {.p_can_cfg_ctrl = &fake_can_cfg_ctrl,
                         .p_can_fifo_options = &can_fifo_options};
    //Act
    can_internal_fifo_support(&p_ctrl, &can_cfg);
    //Assert
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK1],CAN_XID_MASK);
    EXPECT_EQ(p_reg.MKR[CAN_FIFO_RX_MASK2],CAN_XID_MASK);
    
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_SID_Msk,(can_fifo_options.can_rx_fifo_id1.can_mailbox_id << R_CAN0_MB_ID_SID_Pos) & R_CAN0_MB_ID_SID_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id1.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[0] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id1.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);

    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_EID_Msk,(can_fifo_options.can_rx_fifo_id2.can_mailbox_id << R_CAN0_MB_ID_EID_Pos) & R_CAN0_MB_ID_EID_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_RTR_Msk,(can_fifo_options.can_rx_fifo_id2.can_frame_type << R_CAN0_MB_ID_RTR_Pos) & R_CAN0_MB_ID_RTR_Msk);
    EXPECT_EQ(p_reg.FIDCR[1] & R_CAN0_MB_ID_IDE_Msk,(can_fifo_options.can_rx_fifo_id2.can_id_mode << R_CAN0_MB_ID_IDE_Pos) & R_CAN0_MB_ID_IDE_Msk);
}

TEST(CAN_Driver_Func_can_internal_mailbox_read,case1){
    //Arrange
    R_CAN0_Type p_reg{};
    uint32_t fake_mailbox = 2;
    can_frame_t p_frame;
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    
    p_reg.MB[2].DL_b.DLC = 0x08;
    p_reg.MB[2].ID_b.SID = 0x200;
    p_reg.MB[2].ID_b.RTR = 0;
    p_reg.MB[2].ID_b.IDE = 0;
    p_reg.MB[2].D[0] = 0x00;
    p_reg.MB[2].D[1] = 0x11;
    p_reg.MB[2].D[2] = 0x22;
    p_reg.MB[2].D[3] = 0x33;
    p_reg.MB[2].D[4] = 0x44;
    p_reg.MB[2].D[5] = 0x55;
    p_reg.MB[2].D[6] = 0x66;
    p_reg.MB[2].D[7] = 0x77;
    
    //Act
    can_internal_mailbox_read(&p_ctrl, fake_mailbox, &p_frame);

    //Assert
    EXPECT_EQ(p_frame.can_frame_type, 0);
    EXPECT_EQ(p_frame.can_frame_id_mode, 0);
    EXPECT_EQ(p_frame.can_frame_id, 0x200);
    EXPECT_EQ(p_frame.can_frame_dlc, 8);
    EXPECT_EQ(p_frame.can_frame_data[0], 0x00);
    EXPECT_EQ(p_frame.can_frame_data[5], 0x55);
    EXPECT_EQ(p_frame.can_frame_data[7], 0x77);
    EXPECT_EQ(p_reg.MCTL_RX[fake_mailbox], (uint8_t)CAN_MAILBOX_RX_COMPLETE);
    EXPECT_NE(p_reg.RFPCR,(uint8_t)CAN_FIFO_CPU_POINTER);
}
TEST(CAN_Driver_Func_can_internal_mailbox_read,case2){
//Arrange
    R_CAN0_Type p_reg{};
    uint32_t fake_mailbox = 2;
    can_frame_t p_frame;
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    
    p_reg.MB[2].DL_b.DLC = 0x06;
    p_reg.MB[2].ID_b.EID = 0x300;
    p_reg.MB[2].ID_b.RTR = 0;
    p_reg.MB[2].ID_b.IDE = 1;
    p_reg.MB[2].D[0] = 0x00;
    p_reg.MB[2].D[1] = 0x11;
    p_reg.MB[2].D[2] = 0x22;
    p_reg.MB[2].D[3] = 0x33;
    p_reg.MB[2].D[4] = 0x44;
    p_reg.MB[2].D[5] = 0x55;
    p_reg.MB[2].D[6] = 0x66;
    p_reg.MB[2].D[7] = 0x77;
    
    //Act
    can_internal_mailbox_read(&p_ctrl, fake_mailbox, &p_frame);

    //Assert
    EXPECT_EQ(p_frame.can_frame_type, 0);
    EXPECT_EQ(p_frame.can_frame_id_mode, 1);
    EXPECT_EQ(p_frame.can_frame_id, 0x300);
    EXPECT_EQ(p_frame.can_frame_dlc, 6);
    EXPECT_EQ(p_frame.can_frame_data[0], 0x00);
    EXPECT_EQ(p_frame.can_frame_data[5], 0x55);
    EXPECT_EQ(p_reg.MCTL_RX[fake_mailbox], (uint8_t)CAN_MAILBOX_RX_COMPLETE);
    EXPECT_NE(p_reg.RFPCR,(uint8_t)CAN_FIFO_CPU_POINTER);
}
TEST(CAN_Driver_Func_can_internal_mailbox_read,case3){
    //Arrange
    R_CAN0_Type p_reg{};
    uint32_t fake_mailbox = CAN_MAILBOX_ID_RX_FIFO;
    can_frame_t p_frame;
    can_ctrl_ins_t p_ctrl {.p_reg = &p_reg};
    
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].DL_b.DLC = 0x08;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].ID_b.SID = 0x300;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].ID_b.RTR = 0;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].ID_b.IDE = 0;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[0] = 0x00;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[1] = 0x11;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[2] = 0x22;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[3] = 0x33;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[4] = 0x44;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[5] = 0x55;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[6] = 0x66;
    p_reg.MB[CAN_MAILBOX_ID_RX_FIFO].D[7] = 0x77;
    
    //Act
    can_internal_mailbox_read(&p_ctrl, fake_mailbox, &p_frame);

    //Assert
    EXPECT_EQ(p_frame.can_frame_type, 0);
    EXPECT_EQ(p_frame.can_frame_id_mode, 0);
    EXPECT_EQ(p_frame.can_frame_id, 0x300);
    EXPECT_EQ(p_frame.can_frame_dlc, 8);
    EXPECT_EQ(p_frame.can_frame_data[0], 0x00);
    EXPECT_EQ(p_frame.can_frame_data[7], 0x77);
    EXPECT_NE(p_reg.MCTL_RX[fake_mailbox], (uint8_t)CAN_MAILBOX_RX_COMPLETE);
    EXPECT_EQ(p_reg.RFPCR,(uint8_t)CAN_FIFO_CPU_POINTER);
}

TEST(CAN_Driver_Func_can_internal_call_callback,case1){
    //Arrange
    can_frame_t fake_frame = {.can_frame_id = 0x90};
    can_callback_arg_t fake_callback_arg {.can_channel = CAN_CHANNEL_0,
                                        .can_event = CAN_EVENT_TX_COMPLETE,
                                        .can_mailbox = 0,
                                        .can_frame = fake_frame};
    can_callback_arg_t fake_callback_arg2 {.can_channel = CAN_CHANNEL_1,
                                        .can_event = CAN_EVENT_RX_COMPLETE,
                                        .can_mailbox = 2,
                                        .can_frame = fake_frame};
    can_ctrl_ins_t p_ctrl {.p_callback_memory = &fake_callback_arg2,
                           .p_callback = can_callback,
                           };
     //Act
    can_internal_call_callback(&p_ctrl, &fake_callback_arg);

    //Assert
    EXPECT_EQ(p_ctrl.p_callback_memory->can_channel,  fake_callback_arg2.can_channel);
    EXPECT_EQ(p_ctrl.p_callback_memory->can_event,  fake_callback_arg2.can_event);
    EXPECT_EQ(p_ctrl.p_callback_memory->can_mailbox,  fake_callback_arg2.can_mailbox);
    EXPECT_EQ(p_ctrl.p_callback_memory->can_frame.can_frame_id,  fake_frame.can_frame_id);
}
TEST(CAN_Driver_Func_can_internal_call_callback,case2){
    //Arrange
    can_frame_t fake_frame = {.can_frame_id = 0x90};
    can_callback_arg_t fake_callback_arg {.can_channel = CAN_CHANNEL_0,
                                        .can_event = CAN_EVENT_TX_COMPLETE,
                                        .can_mailbox = 0,
                                        .can_frame = fake_frame};
    can_ctrl_ins_t p_ctrl {.p_callback_memory = NULL,
                           .p_callback = can_callback,
                           };
     //Act
    can_internal_call_callback(&p_ctrl, &fake_callback_arg);

    //Assert
    EXPECT_EQ(p_ctrl.p_callback_memory->can_channel,  fake_callback_arg.can_channel);
    EXPECT_EQ(p_ctrl.p_callback_memory->can_event,  fake_callback_arg.can_event);
    EXPECT_EQ(p_ctrl.p_callback_memory->can_mailbox,  fake_callback_arg.can_mailbox);
    //EXPECT_EQ(p_ctrl.p_callback_memory->can_frame.can_frame_id,  0x90);
}

TEST(CAN_Driver_Func_HAL_CAN_Open,case1){
    //Arrange
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    //Act
    HAL_CAN_Open(p_ctrl_arg,&g_can_cfg);
    //Assert
    /* Check Op mode */
    EXPECT_EQ(ctrl_arg.can_operation_mode, CAN_OPERATION_MODE_NORMAL);
    EXPECT_EQ(ctrl_arg.can_open, CAN_OPEN);
    /* Check CTLR */
    EXPECT_EQ(p_reg.CTLR,(uint16_t)229);
    /* Check BTR */
    EXPECT_EQ(p_reg.BCR, (uint32_t)3490001920);
    EXPECT_EQ(p_reg.ECSR,(uint8_t)(1 << R_CAN0_ECSR_EDPM_Pos));
    EXPECT_EQ(ctrl_arg.can_clock_src, g_can_cfg.can_clock_src);
    /* Enable FIFO Transmit and Receive */
    EXPECT_EQ(p_reg.MIER_FIFO,(uint32_t)(301989887));
    EXPECT_EQ(p_reg.RFCR, (uint8_t)(1 << R_CAN0_RFCR_RFE_Pos));
    EXPECT_EQ(p_reg.TFCR, (uint8_t)(1 << R_CAN0_TFCR_TFE_Pos)); 
    EXPECT_EQ(p_reg.EIER, (uint8_t)CAN_ERROR_INTERRUPTS_ENABLE); 
}
TEST(CAN_Driver_Func_HAL_CAN_Open,case2){
    //Arrange
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    g_can_cfg.can_mailbox_mode = CAN_MAILBOX_NONMAL_MODE;
    //Act
    HAL_CAN_Open(p_ctrl_arg,&g_can_cfg);
    //Assert
    /* Check Op mode */
    EXPECT_EQ(ctrl_arg.can_operation_mode, CAN_OPERATION_MODE_NORMAL);
    EXPECT_EQ(ctrl_arg.can_open, CAN_OPEN);
    /* Check CTLR */
    EXPECT_EQ(p_reg.CTLR,(uint16_t)228);
    /* Check BTR */
    EXPECT_EQ(p_reg.BCR, (uint32_t)3490001920);
    EXPECT_EQ(p_reg.ECSR,(uint8_t)(1 << R_CAN0_ECSR_EDPM_Pos));
    EXPECT_EQ(ctrl_arg.can_clock_src, g_can_cfg.can_clock_src);
    /* Enable FIFO Transmit and Receive */
    EXPECT_EQ(p_reg.MIER_FIFO,(uint32_t)(0xFFFFFFFF));
    EXPECT_EQ(p_reg.RFCR, (uint8_t)(0 << R_CAN0_RFCR_RFE_Pos));
    EXPECT_EQ(p_reg.TFCR, (uint8_t)(0 << R_CAN0_TFCR_TFE_Pos));
    EXPECT_EQ(p_reg.EIER, (uint8_t)CAN_ERROR_INTERRUPTS_ENABLE); 

    g_can_cfg.can_mailbox_mode = CAN_MAILBOX_FIFO_MODE;
}

TEST(CAN_Driver_Func_HAL_CAN_Close,case1){
    //Arrange
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_cfg = &g_can_cfg, .p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    
    //Act
    HAL_CAN_Close(p_ctrl_arg);
    
    //Assert
    EXPECT_EQ(ctrl_arg.can_open, CAN_CLOSE);
    EXPECT_EQ(p_reg.RFCR, (uint8_t)(0));
    EXPECT_EQ(p_reg.TFCR, (uint8_t)(0)); 
    EXPECT_EQ(p_reg.MIER, (uint8_t)(0));
    EXPECT_EQ(p_reg.EIER, (uint8_t)(0)); 
}

TEST(CAN_Driver_Func_HAL_CAN_Write,case1){
    //Arrange
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_cfg = &g_can_cfg, .p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    uint32_t fake_mailbox = CAN_MAILBOX_ID_TX_FIFO;
    can_frame_t fake_can_frame 
    { 
        .can_frame_id = 0x29, 
        .can_frame_id_mode = CAN_STANDARD_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_DATA, 
        .can_frame_dlc = 8,
        .can_frame_data = {0,1,2,3,4,5,6,7}
    };
    //Act
    HAL_CAN_Write(p_ctrl_arg, fake_mailbox, &fake_can_frame);
    
    //Assert
    EXPECT_EQ(p_reg.MB[fake_mailbox].DL , 0x08);
    EXPECT_EQ(p_reg.MB[fake_mailbox].ID , 10747904);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[0],  0x00);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[1],  0x1);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[2],  0x2);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[3],  0x3);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[4],  0x4);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[5],  0x5);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[6],  0x6);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[7],  0x7);
    EXPECT_EQ(p_reg.TFPCR, (uint8_t)CAN_FIFO_CPU_POINTER);
}
TEST(CAN_Driver_Func_HAL_CAN_Write,case2){
    //Arrange
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_cfg = &g_can_cfg, .p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    uint32_t fake_mailbox = 2;
    p_reg.MCTL_TX[2] = 0;
    can_frame_t fake_can_frame 
    { 
        .can_frame_id = 0x292, 
        .can_frame_id_mode = CAN_EXTENDED_ID_MODE, 
        .can_frame_type = CAN_FRAME_TYPE_DATA, 
        .can_frame_dlc = 8,
        .can_frame_data = {0,1,2,3,4,5,6,7}
    };

    //Act
    HAL_CAN_Write(p_ctrl_arg, fake_mailbox, &fake_can_frame);
    
    //Assert
    EXPECT_EQ(p_reg.MB[fake_mailbox].DL , 0x08);
    // EXPECT_EQ(p_reg.MB[fake_mailbox].ID , 10747904);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[0],  0x00);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[1],  0x1);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[2],  0x2);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[3],  0x3);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[4],  0x4);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[5],  0x5);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[6],  0x6);
    EXPECT_EQ(p_reg.MB[fake_mailbox].D[7],  0x7);
    EXPECT_EQ(p_reg.MCTL_TX[fake_mailbox],  0x80);
}

TEST(CAN_Driver_Func_HAL_CAN_Read,case1){
    //Arrange
    fsp_err_t err;
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_cfg = &g_can_cfg, .p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    p_reg.MSSR = (uint8_t) 0x0A;
    can_frame_t fake_can_frame;

    p_reg.MB[10].DL_b.DLC = 0x08;
    p_reg.MB[10].ID_b.SID = 0x300;
    p_reg.MB[10].ID_b.RTR = 0;
    p_reg.MB[10].ID_b.IDE = 0;
    p_reg.MB[10].D[0] = 0x00;
    p_reg.MB[10].D[1] = 0x11;
    p_reg.MB[10].D[2] = 0x22;
    p_reg.MB[10].D[3] = 0x33;
    p_reg.MB[10].D[4] = 0x44;
    p_reg.MB[10].D[5] = 0x55;
    p_reg.MB[10].D[6] = 0x66;
    p_reg.MB[10].D[7] = 0x77;
    
    //Act
    err = HAL_CAN_Read(p_ctrl_arg,&fake_can_frame);
    
    //Assert
    EXPECT_EQ(fake_can_frame.can_frame_dlc , 0x08);
    EXPECT_EQ(fake_can_frame.can_frame_id, 0x300);
    EXPECT_EQ(fake_can_frame.can_frame_type, CAN_FRAME_TYPE_DATA);
    EXPECT_EQ(fake_can_frame.can_frame_id_mode, CAN_STANDARD_ID_MODE);
    EXPECT_EQ(fake_can_frame.can_frame_data[0],  0x00);
    EXPECT_EQ(fake_can_frame.can_frame_data[1],  0x11);
    EXPECT_EQ(fake_can_frame.can_frame_data[2],  0x22);
    EXPECT_EQ(fake_can_frame.can_frame_data[3],  0x33);
    EXPECT_EQ(fake_can_frame.can_frame_data[4],  0x44);
    EXPECT_EQ(fake_can_frame.can_frame_data[5],  0x55);
    EXPECT_EQ(fake_can_frame.can_frame_data[6],  0x66);
    EXPECT_EQ(fake_can_frame.can_frame_data[7],  0x77);
    EXPECT_EQ(err, FSP_SUCCESS);
}
TEST(CAN_Driver_Func_HAL_CAN_Read,case2){
    //Arrange
    fsp_err_t err;
    R_CAN0_Type p_reg{}; 
    can_ctrl_ins_t ctrl_arg{.p_cfg = &g_can_cfg, .p_reg = &p_reg};
    can_ctrl_t * p_ctrl_arg = &ctrl_arg;
    p_reg.MSSR = (uint8_t) 0x80;
    can_frame_t fake_can_frame;
    
    //Act
    err = HAL_CAN_Read(p_ctrl_arg,&fake_can_frame);
    
    //Assert
    EXPECT_EQ(err, FSP_ERR_CAN_DATA_UNAVAILABLE);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();  // Run all tests and get the result
    return result;
}