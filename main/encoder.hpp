#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_
#include "main.hpp"

#define ENCL_A 32
#define ENCL_B 33
#define ENCR_A 39
#define ENCR_B 36
#define ENCC_A 35
#define ENCC_B 34

class Encoder
{
private:
    uint8_t enc_a, enc_b;

public:
    Encoder(uint8_t enc_a, uint8_t enc_b);

    byte pos;
    float count;

    void ReadEncoder();
};

#endif // _ENCODER_HPP_
