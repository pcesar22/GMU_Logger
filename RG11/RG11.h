#ifndef _RG11_h
#define _RG11_h


class   RainSensor{
public:

    void init(void);
    void setRainStarted(void);
    void setRainStopped(void);
    int  isRaining(void);

    float getRainHeightInInches(void);
    float getRainHeightInMilimeters(void);
    void resetRainHeight(void);

    float addHeightInInches(float height);
    float addHeightInMilimeters(float height);


private:
    enum _state{rainStarted, rainStopped} _state;
    float _rainHeight; // default is in inches (sorry non-americans).
    float _convertInchesToMm(float value);
    float _convertMmToInches(float value);

};

#endif