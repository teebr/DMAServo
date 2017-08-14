#ifndef DMAServo_hpp
#define DMAServo_hpp

class DMAServo
{
    public:
        DMAServo();
        ~DMAServo();
        void init();
        void init(bool displayPWMTiming);

        void writeMicroseconds(int servoNum,float pwmVal);
        void writeMicroseconds(int servoNum,int pwmVal);
        void writeAngle(int servoNum,float ang);
        void writeAngle(int servoNum,int ang);

        // //TODO: add to current value rather than setting absolute value
        // void incrementMicroseconds(int servoNum,float pwmVal);
        // void incrementMicroseconds(int servoNum,int pwmVal);
        // void incrementAngle(int servoNum,float ang);
        // void incrementAngle(int servoNum,int ang);

        void checkSerial(); //new PWM value
        
    private:
        void _initPDB();
        void _initDMA();
        void _initPins();
        static void _pinTime1();
        static void _pinTime2();
};

#endif
