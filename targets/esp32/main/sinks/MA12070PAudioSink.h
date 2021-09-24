#ifndef MA12070PAUDIOSINK_H
#define MA12070PAUDIOSINK_H

#include <vector>
#include <iostream>
#include "BufferedAudioSink.h"
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"

class MA12070PAudioSink : public BufferedAudioSink
{
public:
    MA12070PAudioSink();
    ~MA12070PAudioSink(); 
    esp_err_t ma_write_byte(uint8_t i2c_addr, uint8_t prot, uint16_t address,
                        uint8_t value);
    void volumeChanged(uint16_t volume);
private:
    
};

#endif