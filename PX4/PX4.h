//
//  PX4.h
//  
//
//  Created by Ariel Feliz on 1/5/17.
//
//

#ifndef PX4_h
#define PX4_h
#include <Arduino.h>
#include <Wire.h>

// 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2)
#define PX4FLOW_ADDRESS 0x42

// timeout in milliseconds for PX4Flow read
#define PX4FLOW_TIMEOUT 10

// If set to true, will print error messages on Serial
#define PX4FLOW_DEBUG true

// As described in the documentation
// http://pixhawk.org/modules/px4flow
typedef struct i2c_frame
{
    uint16_t frame_count;// counts created I2C frames
    int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
    int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
    int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
    int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
    int16_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate; //gyro x rate
    int16_t gyro_y_rate; //gyro y rate
    int16_t gyro_z_rate; //gyro z rate
    uint8_t gyro_range; // gyro range
    uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} i2c_frame;

typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} i2c_integral_frame;

class PX4
{
public:
    PX4(){}
    bool update()
    {
        //send 0x0 to PX4FLOW module and receive back 22 Bytes data
        Wire.beginTransmission(PX4FLOW_ADDRESS);
        Wire.write(0x0);
        Wire.endTransmission();
        
        // request 22 bytes from the module
        Wire.requestFrom(PX4FLOW_ADDRESS, 22);
        
        // wait for all data to be available
        if (!wait(22)) {
            return false;
        }
        
        // read the data
        frame.frame_count       = read16();
        frame.pixel_flow_x_sum  = read16();
        frame.pixel_flow_y_sum  = read16();
        frame.flow_comp_m_x     = read16();
        frame.flow_comp_m_y     = read16();
        frame.qual              = read16();
        frame.gyro_x_rate       = read16();
        frame.gyro_y_rate       = read16();
        frame.gyro_z_rate       = read16();
        frame.gyro_range        = read8();
        frame.sonar_timestamp   = read8();
        frame.ground_distance   = read16();
        
        // if too many bytes are available, we drain in order to be synched
        // on next read
        if(Wire.available())
        {
            #if PX4FLOW_DEBUG == true
            {
                Serial.println("ERROR [PX4Flow] : Too many bytes available.");
            }
            #endif
            while(Wire.available()) {Wire.read();}
        }
        
        return true;
    }
    
    bool update_integral()
    {
        //send 0x16 to PX4FLOW module and receive back 25 Bytes data
        Wire.beginTransmission(PX4FLOW_ADDRESS);
        Wire.write(0x16);
        Wire.endTransmission();
        
        // request 25 bytes from the module
        Wire.requestFrom(PX4FLOW_ADDRESS, 26);
        
        // wait for all data to be available
        // TODO we could manage a timeout in order not to block
        // the loop when no component is connected
        if (!wait(26))
        {
            return false;
        }
        
        // read the data
        iframe.frame_count_since_last_readout = read16();
        iframe.pixel_flow_x_integral  = read16();
        iframe.pixel_flow_y_integral  = read16();
        iframe.gyro_x_rate_integral   = read16();
        iframe.gyro_y_rate_integral   = read16();
        iframe.gyro_z_rate_integral   = read16();
        iframe.integration_timespan   = read32();
        iframe.sonar_timestamp        = read32();
        iframe.ground_distance        = read16();   
        iframe.gyro_temperature       = read16();
        iframe.quality                = read8();
        
        // This is due to the lack of structure packing
        // in the PX4Flow code.
        read8();
        
        // if too many bytes are available, we drain in order to be synched
        // on next read
        if(Wire.available())
        {
            #if PX4FLOW_DEBUG == true
            {
                Serial.println("ERROR [PX4Flow] : Too many bytes available.");
            }
            #endif
            while(Wire.available()) {Wire.read();}
        }
        
        return true;
    }
    
    // Simple frame
    uint16_t frame_count()
    {
        return frame.frame_count;
    }
    int16_t pixel_flow_x_sum()
    {
        return frame.pixel_flow_x_sum;
    }
    int16_t pixel_flow_y_sum()
    {
        return frame.pixel_flow_y_sum;
    }
    int16_t flow_comp_m_x()
    {
        return frame.flow_comp_m_x;
    }
    
    int16_t flow_comp_m_y()
    {
        return frame.flow_comp_m_y;
    }
    
    int16_t gyro_x_rate()
    {
        return frame.gyro_x_rate;
    }
    
    int16_t gyro_y_rate()
    {
        return frame.gyro_y_rate;
    }
    
    int16_t gyro_z_rate()
    {
        return frame.gyro_z_rate;
    }
    
    int16_t qual()
    {
        return frame.qual;
    }
    
    uint8_t sonar_timestamp()
    {
        return frame.sonar_timestamp;
    }
    
    int16_t ground_distance()
    {
        return frame.ground_distance;
    }

    
    // Integral frame
    uint16_t frame_count_since_last_readout()
    {
        return iframe.frame_count_since_last_readout;
    }
    
    int16_t pixel_flow_x_integral()
    {
        return iframe.pixel_flow_x_integral;
    }
    
    int16_t pixel_flow_y_integral()
    {
        return iframe.pixel_flow_y_integral;
    }
    
    int16_t gyro_x_rate_integral()
    {
        return iframe.gyro_x_rate_integral;
    }
    
    int16_t gyro_y_rate_integral()
    {
        return iframe.gyro_y_rate_integral;
    }
    
    int16_t gyro_z_rate_integral()
    {
        return iframe.gyro_z_rate_integral;
    }
    
    uint32_t integration_timespan()
    {
        return iframe.integration_timespan;
    }
    
    uint32_t sonar_timestamp_integral()
    {
        return iframe.sonar_timestamp;
    }
    
    int16_t ground_distance_integral()
    {
        return iframe.ground_distance;
    }
    
    int16_t gyro_temperature()
    {
        return iframe.gyro_temperature;
    }
    
    uint8_t quality_integral()
    {
        return iframe.quality;
    }
    
protected:
    struct i2c_frame frame;
    struct i2c_integral_frame iframe;
    
    uint32_t read32()
    {
        return (uint32_t) read16() + (uint32_t) (read16() << 16);
    }
    
    uint16_t read16()
    {
        return Wire.read() + (uint16_t) (Wire.read() << 8);
    }
    
    uint8_t read8()
    {
        return Wire.read();
    }
    
    bool wait(int count)
    {
        unsigned long now = millis();
        while(Wire.available() < count) {
            if ((millis() - now) > PX4FLOW_TIMEOUT)
            {
                #if PX4FLOW_DEBUG == true
                {
                    Serial.println("ERROR [PX4Flow] : Timeout reading PX4_Flow.");
                }
                #endif
                return false;
            }
            delay(1);
        }
        return true;
    }
};

#endif /* PX4_h */
