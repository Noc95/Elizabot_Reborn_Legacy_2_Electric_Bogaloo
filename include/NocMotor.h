
// #ifndef NOCMOTOR_H
// #define NOCMOTOR_H


// class NocMotor
// {
    
// public:

//     int dummy = 0;
    
    
    
//     void init();
//     void test(int motorSignal);
    
//     // void handleEncoderInterrupt();
//     NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB);

//     // NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB) 
//     // {
//     //     // Callback instance mumbo jumbo
//     //     instance = this;

//     //     this->maxRPM = maxRPM;
//     //     this->motorPinA = motorPinA;
//     //     this->motorPinB = motorPinB;
//     //     this->encoderPinA = encoderPinA;
//     //     this->encoderPinB = encoderPinB;
//     // }

// private:
    
//     int maxRPM;
//     int motorPinA;
//     int motorPinB;
//     int encoderPinA;
//     int encoderPinB;

//     // Static instance pointer for callback
//     static NocMotor* instance;

//     // Static interrupt callback (Arduino attachInterrupt() requires a static function)
//     // static void encoderCallback() {
//     //     if (instance) {
//     //         instance->handleEncoderInterrupt();
//     //     }
//     // }

// };




// #endif