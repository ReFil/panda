#include "llcan.h"

typedef struct CAN_message {
  uint32_t id; // can identifier
  bool ext; // identifier is extended
  uint32_t len; // length of Data
  uint8_t Data[8];
} CAN_message;

uint8_t bitSize(uint64_t v) {
  static const uint8_t lookup[64] = {
  0, // change to 1 if you want bitSize(0) = 1
  1,  2, 53,  3,  7, 54, 27, 4, 38, 41,  8, 34, 55, 48, 28,
  62,  5, 39, 46, 44, 42, 22,  9, 24, 35, 59, 56, 49, 18, 29, 11,
  63, 52,  6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
  51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12
};
  static const uint64_t multiplicator = 0x022fdd63cc95386dUL;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  v++;

  return lookup[(uint64_t)(v * multiplicator) >> 58];
}

uint8_t reverse8(uint8_t inputData) {
  inputData = ((inputData * 0x0802LU & 0x22110LU) | (inputData * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16; 
  return inputData;
}

uint64_t reverse64(uint64_t inputData) {
  //reverse bytes
  //Efficient knuth 64 bit reverse
  static const uint64_t m0 = 0x5555555555555555LLU;
  static const uint64_t m1 = 0x0300c0303030c303LLU;
  static const uint64_t m2 = 0x00c0300c03f0003fLLU;
  static const uint64_t m3 = 0x00000ffc00003fffLLU;
  inputData = ((inputData >> 1) & m0) | (inputData & m0) << 1;
  inputData = inputData ^ (((inputData >> 4) ^ inputData) & m1) ^ ((((inputData >> 4) ^ inputData) & m1) << 4);
  inputData = inputData ^ (((inputData >> 8) ^ inputData) & m2) ^ ((((inputData >> 8) ^ inputData) & m2) << 8);
  inputData = inputData ^ (((inputData >> 20) ^ inputData) & m3) ^ ((((inputData >> 20) ^ inputData) & m3) << 20);
  inputData = (inputData >> 34) | (inputData << 30);
  return inputData;
}

bool CAN_send(CAN_TypeDef *CAN_obj, CAN_message *msg) {
  if ((CAN_obj->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    CAN_obj->sTxMailBox[0].TDLR = msg->Data[0] | (msg->Data[1] << 8) | (msg->Data[2] << 16) | (msg->Data[3] << 24);
    CAN_obj->sTxMailBox[0].TDHR = msg->Data[4] | (msg->Data[5] << 8) | (msg->Data[6] << 16) | (msg->Data[7] << 24);
    CAN_obj->sTxMailBox[0].TDTR = msg->len;
    if(!msg->ext) {
      CAN_obj->sTxMailBox[0].TIR = (msg->id << 21) | 1U;
    }
    else{
      CAN_obj->sTxMailBox[0].TIR = (msg->id << 3) | 1U;
      CAN_obj->sTxMailBox[0].TIR |= 1 << 2;
    }
    return false;
  }
  return true;
}

void CAN_encode(CAN_message *msg, double inputDataDouble, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool sign, double Scale, double bias) { //bitOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000LU;
  //scale and bias
  inputDataDouble = (1/Scale) * (inputDataDouble - bias);
  uint64_t maxVal = 1;
  for(int i=0; i<bitLength; i++) {
    maxVal *= 2;
  }
  //Sign value if appropriate
  if(sign) {
    if(inputDataDouble < 0) {
      inputDataDouble += maxVal;
    }
    inputData = inputDataDouble;
  }
  else
    inputData = inputDataDouble;

  if(inputData > (maxVal-1)) {
    inputData = (maxVal-1);
  }
  else if(inputData < 0){
    inputData = 0;
  }
  //access input as byte array and evaluate length of array
  //take advantage of endianness
  uint8_t *bytePointer = (uint8_t *)&inputData;

  if(bitOrder) {
    //locate MSB
    uint8_t trueLen = bitSize(inputData);
    //if more bits are present than can be accomodated cut them off
    if(trueLen > bitLength) {
      inputData = inputData >> (trueLen - bitLength);
    }
    //Shift Data to 64th position
    inputData = inputData << (64 - bitLength);

    //Reverse int
    inputData = reverse64(inputData);
    //calculate altered start position and move
    uint8_t calcStartbit = (7 - startBit%8) + 8*(startBit/8);
    inputData = inputData << calcStartbit;
    
    //reverse each byte and move to appropriate place
    for(int i=0; i<8; i++) {
      msg->Data[i] |= reverse8(bytePointer[i]);
    }
  }else {
    //Shift Data to appropriate place
    inputData = inputData << startBit;
    //push to message struct
    for(int i=0; i<8; i++) {
      msg->Data[i] |= bytePointer[i];
    }
  }
}

void CAN_encode(CAN_message *msg, int inputDataInt, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool sign, double Scale, double bias) { //bitOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000LU;
  //scale and bias
  inputDataInt = (1/Scale) * (inputDataInt - bias);
  //Sign value if appropriate
  uint64_t maxVal = 1;
  for(int i=0; i<bitLength; i++) {
    maxVal *= 2;
  }
  
  if(sign) {

    if(inputDataInt < 0) {
      inputDataInt += maxVal;
    }
    inputData = inputDataInt;
  }
  else
    inputData = inputDataInt;

  if(inputData > (maxVal-1)) {
    inputData = (maxVal-1);
  }
  else if(inputData < 0){
    inputData = 0;
  }
  //access input as byte array
  //take advantage of endianness
  uint8_t *bytePointer = (uint8_t *)&inputData;

  if(bitOrder) {
    //locate MSB
    uint8_t trueLen = bitSize(inputData);
    //if more bits are present than can be accomodated cut them off
    if(trueLen > bitLength) {
      inputData = inputData >> (trueLen - bitLength);
    }
    //Shift Data to 64th position
    inputData = inputData << (64 - bitLength);

    //Reverse int
    inputData = reverse64(inputData);
    uint8_t calcStartbit = (7 - startBit%8) + 8*(startBit/8);
    inputData = inputData << calcStartbit;
    //reverse each byte and push to message struct
    for(int i=0; i<8; i++) {
      msg->Data[i] |= reverse8(bytePointer[i]);
    }
  }else {
    //Shift Data to appropriate place
    inputData = inputData << startBit;
    //push to message struct
    for(int i=0; i<8; i++) {
      msg->Data[i] |= bytePointer[i];
    }
  }
}

CAN_message CAN_receive(CAN_TypeDef *CAN_obj){
  CAN_message msg;
  while ((CAN_obj->RF0R & CAN_RF0R_FMP0) != 0) {
    if((CAN_obj->sFIFOMailBox[0].RIR >> 2) && 1U) {
      msg.id = CAN_obj->sFIFOMailBox[0].RIR >> 3;
      msg.ext = true;
    }
    else {
      msg.id = CAN_obj->sFIFOMailBox[0].RIR >> 21;
    }
    msg.len = CAN_obj->sFIFOMailBox[0].RDTR;
    msg.Data[0] = CAN_obj->sFIFOMailBox[0].RDLR;
    msg.Data[1] = CAN_obj->sFIFOMailBox[0].RDLR >> 8;
    msg.Data[2] = CAN_obj->sFIFOMailBox[0].RDLR >> 16;
    msg.Data[3] = CAN_obj->sFIFOMailBox[0].RDLR >> 24;
    msg.Data[4] = CAN_obj->sFIFOMailBox[0].RDHR;
    msg.Data[5] = CAN_obj->sFIFOMailBox[0].RDHR >> 8;
    msg.Data[6] = CAN_obj->sFIFOMailBox[0].RDHR >> 16;
    msg.Data[7] = CAN_obj->sFIFOMailBox[0].RDHR >> 24;
    CAN_obj->RF0R |= CAN_RF0R_RFOM0;
  }
  return *msg;
}

double CAN_decode(CAN_message *msg, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool sign, double Scale, double bias) { //bitorder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned
  //Create uint64
uint64_t dataOut;

  if(bitOrder) {
    uint8_t Bytes[8];
    
    //reverse all bytes
    for(int i=0; i<8; i++) {
      Bytes[i] = reverse8(msg->Data[i]);
    }
    //assemble to a uint64
    dataOut = *(uint64_t *)Bytes;
    //reverse to typical lsbfirst
    dataOut = reverse64(dataOut);
    //shift to isolate data
    uint8_t calcStartbit = (7 - (startBit%8)) + 8*(startBit/8);
    dataOut = dataOut << calcStartbit;
    //shift bits back
    dataOut = dataOut >> (64 - (bitLength));
  } else {
    //send message into uint64
    dataOut = *(uint64_t *)msg->Data;
    //Shift left then right to isolate the Data
    dataOut = dataOut << (64 - (startBit+bitLength));
    dataOut = dataOut >> (64 - bitLength);
  }
  double returnData;
  //Adjust if signed and scale and bias
  if(sign) {
    uint64_t maxVal = 1;
    for(int i=0; i<bitLength; i++) {
      maxVal *= 2;
    }
    if(dataOut > (maxVal/2)) {
      returnData = dataOut - maxVal;
      returnData = bias + (Scale * returnData);
    }
    else {
      returnData = bias + (Scale * dataOut);
    }
  }
  else {
    returnData = bias + (Scale * dataOut);
  }

  return(returnData);
}
