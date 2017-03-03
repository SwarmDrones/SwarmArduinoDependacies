/** @file DroneCom.h
 *   @brief Drone communication output class.
        - outputs the values needed for localization, navigation, m2m on all demos and final platform.
        - outputs are defined based on using either wireless or serial communication.
 *   @author Ariel Feliz(aafeliz)
 *   @date 11/17/16
 *   @todo Create the wireless ouputs
 *   @todo Create m2m outputs on both serial and/or wireless sections
 */

#ifndef DroneCom_h
#define DroneCom_h
#include <Arduino.h>

#define PROCESSING
//#define UNITY

//#define SERIAL_COM
#define WIRELESS

#ifdef SERIAL_COM
/** @brief IMU output functions. */
/** @brief outputs information about the sensor */
void IMUSensorDetails(sensor_t sensor)
{
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}
/** @brief outputs the orientation of the drone */
void printOrientation(const double& roll, const double& pitch, const double& yaw)
{
    /* The processing sketch expects data as roll, pitch, heading */
    Serial.print(F("Orientation: "));
    Serial.print((double)yaw);// yaw
    Serial.print(F(" "));
    Serial.print((double)pitch);// pitch
    Serial.print(F(" "));
    Serial.print((double)roll);// roll
    Serial.println(F(""));
}
/** @brief outputs the location of the drone */
void printLocation(const uint8_t& surfaceQuality, const double& x_cm, const double& y_cm, const double& z_cm)
{
    /* The processing sketch expects data as roll, pitch, heading */
    Serial.print(F("Location: "));
    Serial.print(surfaceQuality);
    Serial.print(F(" "));
    Serial.print((double)x_cm);
    Serial.print(F(" "));
    Serial.print((double)y_cm);
    Serial.print(F(" "));
    Serial.print((double)z_cm);
    Serial.println(F(""));
}
/** @brief outputs the speed */
/** @todo create speed ouput */
void printSpeed(const double& vx, const double& vy, const double& vz)
{
    
}
/** @brief used for calibration purposes the small motionflow camera */
void localizationDebugOutput(const uint8_t& surfaceQuality, const double& iy_cm, const int32_t& y_cam, const double& y_cam_comp, const double& roll, const double& ix_cm, const int32_t& x_cam, const double& x_cam_comp, const double& pitch)
{
    Serial.print("camQ: "); Serial.printf("%04d", surfaceQuality);
    Serial.print("\t iY: "); Serial.printf("%4f", iy_cm);
    Serial.print("\t\t Y_cam: "); Serial.printf("%4d", y_cam);
    Serial.print("\t Y_compensate: "); Serial.printf("%4f", y_cam_comp);
    Serial.print("\t\t roll: "); Serial.printf("%4f", roll);
    Serial.print("\t iX: "); Serial.printf("%4f", ix_cm);
    Serial.print("\t\t X_cam: "); Serial.printf("%4d", x_cam);
    Serial.print("\t X_compensate: "); Serial.printf("%4f", x_cam_comp);
    Serial.print("\t\t pitch: "); Serial.printf("%4f", pitch);
    Serial.println();
}
#endif

#ifdef WIRELESS
/**@todo Mike Xbee serial print out goes here*/
class DroneCom
{
 public:
    bool msgInFlag;
    String mIn;
    String mOut;
    DroneCom()
    {
        mIn = "";
        mOut = "";
        msgInFlag = false;
        messageOut = NULL;
        messageIn = NULL;
    }
    /**
     @brief : initiates drone communication with the teensy device out to pin 1 and 0 onto the xbee module using both soft serial and hw serial. 
    */
    void init()
    {
        Serial.begin(115200);
        Serial1.begin(115200);
        Serial.setTimeout(5);
        Serial1.setTimeout(5);
    }
    
    /**
     @brief : transmit to coordinator 
    */
    void transmit2Coor(String message)
    {
        messageOut = tx_headerGen(message, message.length());
        Serial1.write(messageOut, 18+message.length());
        Serial1.flush();
    }
    
    /**
     @brief : receive incoming messages and parses out the address and message 
    */
    void updateRXMsg()
    {
        mIn = "";
        Serial1.flush();
        Serial.flush();
        while(Serial1.available())
        {
            mIn += Serial1.readString();
        }
        
        //Serial.println(mIn);
    
        if(mIn.length() > 0)
        {
            if(verifyIncoming(mIn) == true)
            {
                mIn = rx_headerInter(mIn, mIn.length());
                msgInFlag = true;
                //Serial.println(mIn);
            }
        }
        
    }
    
    bool checkInFlag()
    {
        return msgInFlag;
    }
    String getMessage()
    {
        msgInFlag = false;
        return mIn;
    }
    /**
     @brief : varifies that the full package was recieved
    */
    bool verifyIncoming(String header)
    {
        // checksum where sum is from frame type to end of rfdata
        //Serial.println("verifyIncoming: ");
        for(int i = 0; i < header.length(); i++)
        {
            Serial.print(header[i], HEX);
            Serial.print(" ");
        }
        
        uint8_t sum = 0x00;
        //Serial.println("");
        uint8_t checking[header.length()-3];
        for(int i = 0; i < (header.length()-3); i++)
        {
                checking[i] = header[i+2];
                //Serial.print(checking[i], HEX);
                //Serial.print(" ");
        }
        //Serial.println("");
        sum = chksum8(checking, header.length()-3);
        uint8_t checksum = (0xFF) - (sum);
        //Serial.print(checksum, HEX);
        //Serial.print(" == ");
        //Serial.print(header[header.length()-1], HEX);
        //Serial.println("");
        uint8_t frameType = header[2];
        if((header[header.length()-1]== checksum)&&(frameType == 0x90))// make sure both checksum and frametype agree
        {
            return true;
            //Serial.println("true message");
        }
        else 
        {
            msgInFlag = false;
            //Serial.print("false message: ");
            //Serial.println(frameType,HEX);
            return false;
        }
    }
    
    /**
     @brief :send orientation messages over xbee to coordinator
    */
    void sendOrientation(const double& roll, const double& pitch, const double& yaw)
    {
        String msg = "Orientation: ";
        msg += roll;
        msg += " ";
        msg += pitch;
        msg += " ";
        msg += yaw;
        transmit2Coor(msg);
    }
    
    /**
     @brief : send incoming to processing
     */
    /*
    String incoming2Processing(String mIn)
    {
        String add = mIn.substring(3, 11);
        String data = mIn.substring(13, mIn.length()-1);
        String out2P = add + ":" + data;
        return out2P;
        
    }*/
    /**
     @brief : transmit to drones
     */
    void transmit2Drone(String message)
    {
        messageOut = tx_headerGen(message, message.length());
        messageOut[11] = char(0xFF);
        messageOut[12] = char(0xFF);
        /*Serial.print("sending:");
        for(int i = 0; i < message.length()+18; i++)
        {
            Serial.print(messageOut[i], HEX);
            Serial.print(" ");
        }
        Serial.println("");*/
        Serial1.write(messageOut, 18+message.length());
        Serial1.flush();
    }

    
 private:
    uint8_t* messageOut;
    char* messageIn;
    
    /**
     @brief: checksum byte for header generation
    */
    uint8_t chksum8(unsigned char *buff, size_t len)
    {
            uint8_t sum;       // nothing gained in using smaller types!
            for ( sum = 0 ; len != 0 ; len--)
                    sum += *(buff++);   // parenthesis not required!
            return (uint8_t)sum;
    }
    
    /**
     @brief : generates the header to output messages in API mode. 
    */
    uint8_t* tx_headerGen(String message, unsigned int sizet)
    {
        unsigned int nBytes = 17 + sizet + 1;
        uint8_t* header = new uint8_t[nBytes];
        //start delimeter
        header[0] = (uint8_t)(0x7E);
        
        // size of meassage is in two bytes
        header[1] = (uint8_t)(((nBytes-4) >> 8) & 0xFF);
        header[2] = (uint8_t)((nBytes-4) & 0xFF);
        
        // frame type and id
        header[3] = (uint8_t)(0x10);
        header[4] = (uint8_t)(0x00);

        // destination address
        for(int i = 0; i < 8; i++)
        {
            header[5+i] =(uint8_t)(0x00); 
        }
        header[13]= (uint8_t)(0xFF);
        header[14]= (uint8_t)(0xFE);

        //options
        header[15]= (uint8_t)(0x00);
        header[16]= (uint8_t)(0x00);

        //message
        for(int i = 0; i < sizet; i++)
        {
            header[17+i] = (uint8_t)message[i];
        }

        // checksum where sum is from frame type to end of rfdata
        uint8_t sum = 0x00;
        uint8_t checking[14+sizet];
        for(int i = 3; i < (17+sizet); i++)
        {
                    checking[i-3] = header[i];
        }
        sum = chksum8(checking, 14 + sizet);
        uint8_t checksum = ((0xFF) - (sum)) +(0x02);
        header[17+sizet]= (checksum);
        
        return header;
    }
    
    /**
     @brief : parse headers that come in from in API mode. 
    */
    String rx_headerInter(String rx_message, unsigned int sizet)
    {
            char buf[sizet];
            rx_message.toCharArray(buf, sizet);//getBytes(buf, sizet);
            /*Serial.print("incoming RX:");
            for(int i = 0; i < sizet; i++)
            {
                Serial.print(buf[i], HEX);
                Serial.print(" ");
            }*/
            //Serial.print('\t');
            //Serial.print(rx_message);
            
            Serial.println("");//*/
            const uint8_t dataLen = sizet - 12;// length of data in message
            const uint8_t addLen = 8; // length of address

            String out;
            out = "";
            //out.reserve(dataLen + addLen + 1);// = new char
            /*
            // parsing the address portion in the header
            for(int i = 0; i < addLen; i++)
            {
                    out+= rx_message[i+3];
            }
            out += ':';//out[addLen] = ':';
            // parsing the data portion of the header
            for(int i = 0; i < dataLen; i++)
            {
                    out+= rx_message[i+10];//out[i + addLen] = rx_message[i+16];
            }*/
            //int total = dataLen + addLen;
            //Serial.println(out);
        
            for(int i = 0; i < addLen + dataLen ; i++)
            {
                out+= rx_message[i+3];
            }
            out[8] = ':';
            return out;
        
    }
};

#endif

#endif /* DroneCom_h */
