#include "llcan.h"

typedef struct CAN_message {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint8_t data[8];
} CAN_message;

void CAN_send(CAN_TypeDef *CAN_obj, CAN_message *message) {

}

CAN_message CAN_encode(CAN_message *msg, double inputDataDouble, uint8_t startBit, uint8_t bitLength, bool bitOrder, bool signed, double scale, double bias) { //bitOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000;
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
  uint8_t arrayLen = bitLength/8;

  if(bitLength % 8 != 0) {
    arrayLen += 1;
  }
  //Much simpler if LSB first
  if(!bitOrder) {
    //bitshift left to appropriate starting space
    inputData = inputData << startBit;
    uint8_t endByte = (startBit + bitLength)/8;
    if((startBit+bitLength) % 8 != 0) {
      endByte += 1;
    }
    for(int i=0; i<endByte; i++) {
      msg.data[i] |= bytePointer[i];
    }
  }
  else {
    //locate MSB
    uint8_t trueLen = bitSize(inputData);
    //if more bits are present than can be accomodated cut them off
    if(trueLen > bitLength) {
      inputData = inputData >> (trueLen - bitLength);
    }
    //Just shift data to 64th position
    inputData = inputData << (63 - bitLength);

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

    inputData = inputData << startBit;
    uint8_t endByte = (startBit + bitLength)/8;
    if((startBit+bitLength) % 8 != 0) {
      endByte += 1;
    }
    for(int i=0; i<endByte; i++) {
      msg.data[i] |= bytePointer[i];
    }

  }
}

CAN_message CAN_receive(CAN_TypeDef *CAN_obj){

}

double CAN_decode(CAN_message *msg, uint8_t startBit, uint8_t bitLength, bool byteOrder, bool signed, double scale, double bias) { //byteorder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

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
