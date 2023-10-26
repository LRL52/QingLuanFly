#ifndef __GAUSSNEWTON_H__
#define __GAUSSNEWTON_H__

// calibrate v校正
typedef struct _cali {
    float Ox, Sx, Oy, Sy, Oz, Sz;     
} cali;

extern cali accCali;
extern cali magCali;
extern cali gyroCali;
extern float accData[][3];
extern float magData[][3];

void prepareData();
void gaussNewton(cali *caliVal, float (*data)[3]);

#endif // __GAUSSNEWTON_H__