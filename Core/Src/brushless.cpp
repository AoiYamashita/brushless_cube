#include "brushless.hpp"

BrushLess::BrushLess(CAN_HandleTypeDef *hcan){
    can = hcan;
}

void BrushLess::Init(){
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; // CAN1のみ使用時は無視されます

    if (HAL_CAN_ConfigFilter(can, &sFilterConfig) != HAL_OK) {
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
		for(int i = 0;i < 8;i++){
			if(rxHeader.StdId != CAN_RECEIVE_ID[i])continue;
			char data[8] = {rxData[1],rxData[0],rxData[3],rxData[2],rxData[5],rxData[4],rxData[6],rxData[7]};
			memcpy((ReceiveData*)&(R[i]),data,8);
		}
	}
}

void BrushLess::SetSpeed(int id,float speed){
    float sp = speed;
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    S.speed[id] = 10000 * sp;
}

bool BrushLess::Write(){
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(can)) {
      for (int i = 0; i < 2; i++) {
        char b[8];
        memcpy((char*)b, (&S)->speed + (4 * i), 8);
        uint8_t send[8] = {b[1], b[0], b[3],b[2],b[5],b[4],b[7],b[6]};
        
        TxHeader.StdId = CAN_SEND_ID[i];// 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8;//データ長を8byteに設定
        TxHeader.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(can, &TxHeader, send, &TxMailbox) != HAL_OK) {
          //Error_Handler();
          return false;
        }
      }
    }
    return true;
}