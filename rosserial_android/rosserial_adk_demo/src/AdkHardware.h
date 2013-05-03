#ifndef _ADK_HARDWARE_H_
#define _ADK_HARDWARE_H_

#include "AndroidAccessory/AndroidAccessory.h"
#include <WProgram.h>

#define __DEBUG__ 

class AdkHardware{
public:
    AdkHardware(): acc("Willow Garage", "ROSSerialADK",
		            "ROS ADK Bridge",  "1.0", "http://www.android.com",
		            "0000000012345678"),index(-1) 
{
    }
    void init(){
        acc.powerOn();
        index=0;
        len=0;
        Serial.begin(115200);
    }
    
    void write(uint8_t* data, int out_len){
        if (acc.isConnected()) acc.write(data, out_len);
       
       #ifdef __DEBUG__
        Serial.print("ADK : ");
        for(int i=0; i< out_len; i++) Serial.print(data[i]);
        Serial.println("");
        #endif
        
    }
    
    int read(){
        int ret =-1;
        if (index< len){
           ret = input_buff[index];
           ++index;

        }
        else{
            if (acc.isConnected()) len = acc.read(input_buff, 100, 1);
            index = 0;
            if (len>0){
                #ifdef __DEBUG__
                Serial.print("\nCell :");
                #endif
                ret = input_buff[index];
                index++;
            }
            
        }
        #ifdef __DEBUG__
        if (ret!=-1) Serial.print((char) ret);
        #endif 
        return ret;
    }
    
    unsigned long time(){return millis();};
private:
    AndroidAccessory acc;
    uint8_t input_buff[100];
    int index;
    int len;
};

#endif
