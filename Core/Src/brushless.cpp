#include "brushless.hpp"

BrushLess::BrushLess(CAN_HandleTypeDef *hcan){
    can = hcan;
}

void BrushLess::Init(){
    CAN_FilterTypeDef sFilterConfig;// 0x201 ~ 0x207

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x200 << 5;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x7F8 << 5;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(can, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    CAN_FilterTypeDef s1FilterConfig;// 0x208

    s1FilterConfig.FilterBank = 1;
    s1FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    s1FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    s1FilterConfig.FilterIdHigh = 0x208 << 5;
    s1FilterConfig.FilterIdLow = 0x0000;
    s1FilterConfig.FilterMaskIdHigh = 0x7FF << 5;
    s1FilterConfig.FilterMaskIdLow = 0x0000;
    s1FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    s1FilterConfig.FilterActivation = ENABLE;
    s1FilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(can, &s1FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_CAN_Start(can);
    HAL_CAN_ActivateNotification(can, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);
}

void BrushLess::CallBack(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		if(rxHeader.StdId < MIN_CAN_RECEIVE_ID || rxHeader.StdId > MAX_CAN_RECEIVE_ID)return;
		char data[8] = {rxData[1],rxData[0],rxData[3],rxData[2],rxData[5],rxData[4],rxData[6],rxData[7]};
		memcpy((ReceiveData*)&(R[rxHeader.StdId - MIN_CAN_RECEIVE_ID]),data,8);
	}
}

void BrushLess::SetSpeed(int id,float speed){
    float sp = speed;
    if (abs(sp) > 1)sp = sp/abs(sp);
    S.speed[id] = 10000 * sp;
}

bool BrushLess::Write(){
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    if (HAL_CAN_GetTxMailboxesFreeLevel(can) <= 0) return false;
    for (int i = 0; i < 2; i++) {
        uint8_t b[8];
        memcpy((char*)b, (&S)->speed + (4 * i), 8);
        uint8_t send[8] = {b[1], b[0], b[3],b[2],b[5],b[4],b[7],b[6]};
        
        TxHeader.StdId = CAN_SEND_ID[i];// 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8;//データ長を8byteに設定
        TxHeader.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(can, &TxHeader, send, &TxMailbox) != HAL_OK) {
            return false;
        }
    }
    return true;
}