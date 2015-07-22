#include "RG11.h"


void RainSensor::init(void){
    _state = rainStopped;
    resetRainHeight();
}
void RainSensor::setRainStarted(void){
    _state = rainStarted;
}
void RainSensor::setRainStopped(void){
    _state = rainStopped;
    resetRainHeight();
}
int  RainSensor::isRaining(void){
    return (_state == rainStarted);
}

float RainSensor::getRainHeightInInches(void){
    return _rainHeight;
}
float RainSensor::getRainHeightInMilimeters(void){
    return _convertInchesToMm(_rainHeight);
}
void  RainSensor::resetRainHeight(void){
    _rainHeight = 0;
}

float RainSensor::addHeightInInches(float height){
    if(_state == rainStarted){
        _rainHeight += height;
    }
}
float RainSensor::addHeightInMilimeters(float height){
    _rainHeight += _convertInchesToMm(height);
}

float _convertInchesToMm(float value){
    return value*25.4f;
}
float _convertMmToInches(float value){
    return value/25.4f;
}