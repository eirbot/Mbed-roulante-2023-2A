#ifndef brushless_hpp
#define brushless_hpp

#include ""

class Brushless{
    public:
    Brushless(TI)
    int init(void);
    float getVelocity(void);

    

    private:
    int Te_odometrie = 1000; //us
    float velocity;
    float Kp;
    float Ki;
};

#endif