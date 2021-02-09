#include "llcan.h"

struct CAN_message {
  uint32_t id; // can identifier
  bool ext; // identifier is extended
  uin32_t len; // length of data
  uint8_t data[8];
};

bool CAN_send(CAN_TypeDef *CAN_obj, CAN_message *msg) {
  if ((CAN_obj->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    CAN_obj->sTxMailBox[0].TDLR = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24);
    CAN_obj->sTxMailBox[0].TDHR = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24);
    CAN_obj->sTxMailBox[0].TDTR = msg.len;
    if(!msg.ext) {
      CAN_obj->sTxMailBox[0].TIR = (msg.id << 21) | 1U;
    }
    else{
      CAN_obj->sTxMailBox[0].TIR = (msg.id << 3) | 1U;
      CAN_obj->sTxMailBox[0].TIR |= 1 << 2;
    }
    return false;
  }
  return true;
}

CAN_message CAN_encode(CAN_message *msg, double inputDataDouble, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool signed, double scale, double bias) { //bitOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000LU;
  inputDataDouble = (1/Scale) * (inputDataDouble - bias);
  //Sign value if appropriate
  if(signed) {

    if(inputDataDouble < 0) {
      uint64_t maxVal = 1;
      for(int i=0; i<bitLength; i++) {
        maxVal *= 2;
      }
      inputDataDouble += maxVal;
    }
    inputData = inputDataDouble;
  }
  else
    inputData = inputDataDouble;

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
    //Shift data to 64th position
    inputData = inputData << (64 - bitLength);

    //Reverse int
    inputData = reverse64(inputData);
  }

  //Shift data to appropriate place
  inputData = inputData << startBit;

  for(int i=0; i<8; i++) {
    msg.data[i] |= bytePointer[i];
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
    msg.data[0] = CAN_obj->sFIFOMailBox[0].RDLR;
    msg.data[1] = CAN_obj->sFIFOMailBox[0].RDLR >> 8;
    msg.data[2] = CAN_obj->sFIFOMailBox[0].RDLR >> 16;
    msg.data[3] = CAN_obj->sFIFOMailBox[0].RDLR >> 24;
    msg.data[4] = CAN_obj->sFIFOMailBox[0].RDHR;
    msg.data[5] = CAN_obj->sFIFOMailBox[0].RDHR >> 8;
    msg.data[6] = CAN_obj->sFIFOMailBox[0].RDHR >> 16;
    msg.data[7] = CAN_obj->sFIFOMailBox[0].RDHR >> 24;
    CAN_obj->RF0R |= CAN_RF0R_RFOM0;
  }
  return msg;
}

double CAN_decode(CAN_message *msg, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool signed, double scale, double bias) { //bitorder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned
  uint64_t dataOut = 0x0000000000000000LU;

  //read nessage into int;
  dataOut = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24) | (msg.data[4] << 32) | (msg.data[5] << 40) | (msg.data[6] << 48) | (msg.data[7] << 56);

  //Shift left then right to isolate the data
  dataOut = dataOut << (63 - startBit);
  dataOut = dataOut >> (64 - bitLength);

  if(bitOrder) {
    //reverse to typical lsbfirst
    dataOut = reverse64(dataOut);
    //shift bits back
    dataOut = dataOut >> (64 - bitLength);
  }

  if(signed) {
    uint64_t maxVal = 1;
    for(int i=0; i<bitLength; i++) {
      maxVal *= 2;
    }
    if(dataOut > maxVal) {
      double returnData = dataOut - maxVal;
      returnData = bias + (scale * returnData);
    }
    else {
      double returnData = bias + (scale * dataOut);
    }
  }
  else {
    double returnData = bias + (scale * dataOut);
  }

  return(returnData);
}

uint8_t bitSize(uint64_t v) {
  static const uint8_t lookup[64] = {
  0, // change to 1 if you want bitSize(0) = 1
  1,  2, 53,  3,  7, 54, 27, 4, 38, 41,  8, 34, 55, 48, 28,
  62,  5, 39, 46, 44, 42, 22,  9, 24, 35, 59, 56, 49, 18, 29, 11,
  63, 52,  6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
  51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12
  }
  static const uint64_t multiplicator = 0x022fdd63cc95386dUL;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  v++

  return lookup[(uint64_t)(v * multiplicator) >> 58];
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
