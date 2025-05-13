#ifndef BRUSHLESS_H
#define BRUSHLESS_H

// write by yamashita
// Referencing "brushless.h"
// need to call BrushLess::CallBack
// in "HAL_CAN_RxFifo0MsgPendingCallback"

#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"

#define DEGREE_MAX 8191
#define DEGREE_MIN 0

const int CAN_SEND_ID[] = {0x200, 0x1FF};
#define MAX_CAN_RECEIVE_ID 0x208
#define MIN_CAN_RECEIVE_ID 0x201



struct SendData {
    short speed[8];    
};

struct ReceiveData {
    short degree;
    short rpm;
    short TC;
    char temp;
    char none;
};

class BrushLess{
private:
    SendData S;
    CAN_HandleTypeDef *can;
public:
    ReceiveData R[8];
    BrushLess(CAN_HandleTypeDef *hcan);
    void Init();
    void CallBack(CAN_HandleTypeDef *hcan);
    void SetSpeed(int,float);
    bool Write();
};

#endif