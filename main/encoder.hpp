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
    float pos, count;
    int enc_a, enc_b;

public:
    Encoder(int enc_a, int enc_b);

    void EncoderRead();
};

#endif // _ENCODER_HPP_