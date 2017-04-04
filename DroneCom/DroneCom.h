/** @file DroneCom.h
 *   @brief Drone communication output class.
        - outputs the values needed for localization, navigation, m2m on all demos and final platform.
        - outputs are defined based on using either wireless or serial communication.
 *   @author Ariel Feliz(aafeliz)
 *   @date 11/17/16
 */

#ifndef DroneCom_h
#define DroneCom_h
#include <Arduino.h>
#include <SoftwareSerial.h>


#define S_PORT Serial1
#define PROCESSING
//#define COORDINATOR
#define DRONE
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
    #ifdef DRONE
    // booleans to check the type of message comming in
    // coordinator sends new PID values to DRONE
    bool PMsgIn;
    bool IMsgIn;
    bool DMsgIn;
    bool RollMsgIn;
    bool PitchMsgIn;
    bool YawMsgIn;
    bool PosMsgIn;
    bool OriMsgIn;
    // coordinator sends new Throttle values to Drone
    bool ThrottleMsgIn;
    // coordinator sends new Destination values to DRONE
    bool DestMsgIn;
    
    
    // strings to store the messages
    String pidVal; // P || I || D val
    String throttleVal; // throttle val
    String destVals; // x && y && x && r && p && y values
    
    //SoftwareSerial mySerial;
    #endif
    
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
        #ifdef DRONE
        // coordinator sends new PID values to DRONE
        PMsgIn = false;
        IMsgIn= false;
        DMsgIn = false;
        RollMsgIn = false;
        PitchMsgIn = false;
        YawMsgIn = false;
        PosMsgIn = false;
        OriMsgIn = false;
        
        ThrottleMsgIn = false;
        
        DestMsgIn = false;
        
        pidVal = "";
        throttleVal = "";
        destVals = "";
        
        //SoftwareSerial temp(0,1);
        //mySerial = temp;
        #endif
    }
    /**
     @brief : initiates drone communication with the teensy device out to pin 1 and 0 onto the xbee module using both soft serial and hw serial. 
    */
    void init()
    {
        
        Serial.begin(115200);
        S_PORT.begin(115200);
        Serial.setTimeout(5);
        S_PORT.setTimeout(5);
       
    }
    
    /**
     @brief : transmit to coordinator 
    */
    void transmit2Coor(String message)
    {
        messageOut = tx_headerGen(message, message.length());
        
        
        /*Serial.print("sending:");
        for(int i = 0; i < message.length()+18; i++)
        {
            Serial.print(messageOut[i], HEX);
            Serial.print(" ");
        }
        Serial.println("");*/
        S_PORT.flush();
        S_PORT.write(messageOut, 18+message.length());
        delete[] messageOut;
        //S_PORT.flush();
    }
    
    #ifdef COORDINATOR
    /**
     @brief : COORDINATOR receive incoming messages and parses out the address and message
    */
    void updateRXMsg()
    {
        mIn = "";
        mIn.trim();
        //S_PORT.flush();
        //Serial.flush();
        while(S_PORT.available())
        {
            mIn += S_PORT.readString();
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
    
    
    #else // other wise you must be a drone
    /**
     @brief : DRONE receive incoming messages and parses out the address and message
     */
    void updateRXMsg()
    {
        mIn = "";
        mIn.trim();
        //S_PORT.flush();
        //Serial.flush();
        while(S_PORT.available())
        {
            mIn += S_PORT.readString();
        }
        
        
        if(mIn.length() > 0)
        {
            msgInFlag = verifyIncoming(mIn);
        
        
            if(msgInFlag == true)
            {
                mIn = rx_headerInter(mIn, mIn.length());
                String msg = mIn;
                //Serial.print("incoming:");
                //Serial.print(mIn);
                //Serial.print(":");
                //Serial.println(msg);
                char firstChar = msg[0];// this can be either a number for PID change or Dest for change in destination
                // checking if its a PID value
                if(firstChar == '0' || firstChar == '1' || firstChar == '2')
                {
                    DestMsgIn = false;
                    ThrottleMsgIn = false
                    char axis = msg[0];
                    // checking which axis to change is X/Roll
                    if(firstChar == '0')
                    {
                        //Serial.println("Roll axis");
                        RollMsgIn = true;
                        PitchMsgIn = false;
                        YawMsgIn = false;
                        // checking which constant is being changed
                        char consType = msg[1];
                        if(consType == 'P')
                        {
                            //Serial.println("P constant");
                            PMsgIn = true;
                            IMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                //Serial.println("orientation PID");
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'I')
                        {
                            IMsgIn = true;
                            PMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'D')
                        {
                            DMsgIn = true;
                            IMsgIn = false;
                            PMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        
                    }
                    // checking which axis to change is Y/Pitch
                    if(firstChar == '1')
                    {
                        RollMsgIn = false;
                        PitchMsgIn = true;
                        YawMsgIn = false;
                        // checking which constant is being changed
                        char consType = msg[1];
                        if(consType == 'P')
                        {
                            PMsgIn = true;
                            IMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'I')
                        {
                            IMsgIn = true;
                            PMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'D')
                        {
                            DMsgIn = true;
                            IMsgIn = false;
                            PMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        
                    }
                    // checking which axis to change is Z/Yaw
                    if(firstChar == '2')
                    {
                        RollMsgIn = false;
                        PitchMsgIn = false;
                        YawMsgIn = true;
                        // checking which constant is being changed
                        char consType = msg[1];
                        if(consType == 'P')
                        {
                            PMsgIn = true;
                            IMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'I')
                        {
                            IMsgIn = true;
                            PMsgIn = false;
                            DMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        if(consType == 'D')
                        {
                            DMsgIn = true;
                            IMsgIn = false;
                            PMsgIn = false;
                            // checking whether is orientation or location
                            char poseType = msg[3];
                            if(poseType == 'p') // meaning position constant came in
                            {
                                PosMsgIn = true;
                                OriMsgIn = false;
                                
                            }
                            else if(poseType == 'o') // meaning orientation constant came in
                            {
                                OriMsgIn = true;
                                PosMsgIn = false;
                            }
                        }
                        
                    }


                }
                // Throttle command
                else if(firstChar == 'T')
                {
                    PMsgIn = false;
                    IMsgIn= false;
                    DMsgIn = false;
                    RollMsgIn = false;
                    PitchMsgIn = false;
                    YawMsgIn = false;
                    PosMsgIn = false;
                    OriMsgIn = false;
                    ThrottleMsgIn = true;
                    DestMsgIn = false;
                }
                //  destination new command
                else if(firstChar == 'D')
                {
                    PMsgIn = false;
                    IMsgIn= false;
                    DMsgIn = false;
                    RollMsgIn = false;
                    PitchMsgIn = false;
                    YawMsgIn = false;
                    PosMsgIn = false;
                    OriMsgIn = false;
                    ThrottleMsgIn = false;
                    DestMsgIn = true;
                }
                
                
                // set the value by type
                if(PosMsgIn == true || OriMsgIn == true)
                {
                    //pid val gets stored as string
                    pidVal = msg.substring(7);
                    //Serial.print("PID Value is:");
                    //Serial.println(pidVal);
                }
                // storing the throttle message that came in
                else if(ThrottleMsgIn == true)
                {
                    throttleVal = msg.substring(9);
                }
                else if(DestMsgIn == true)
                {
                    // dest get stored
                }
            }
        }
        
    }
    
    void resetFlags()
    {
        msgInFlag = false;
        PMsgIn = false;
        IMsgIn= false;
        DMsgIn = false;
        RollMsgIn = false;
        PitchMsgIn = false;
        YawMsgIn = false;
        PosMsgIn = false;
        OriMsgIn = false;
        DestMsgIn = false;
    }
    String getPIDMsgVal()
    {
        resetFlags();
        return pidVal;
    }
    String getThrottleMsgVal()
    {
        resetFlags();
        return throttleVal;
    }
    String getDestMsgVals()
    {
        resetFlags();
        return destVals;
    }
    #endif
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
     @brief :DRONE varifies that the full package was recieved
    */
    bool verifyIncoming(String header)
    {
        // checksum where sum is from frame type to end of rfdata
        //Serial.println("verifyIncoming: ");
        /*for(int i = 0; i < header.length(); i++)
        {
            Serial.print(header[i], HEX);
            Serial.print(" ");
        }*/
        //if(header.length() < 1){ return false; Serial.print("false message: ");}
        
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
            //Serial.println("true message");
            return true;
            
        }
        //else
        //{
            msgInFlag = false;
            //Serial.print("false message: ");
            //Serial.println(frameType,HEX);
            return false;
        //}
    }
    
    /**
     @brief :send orientation messages over xbee to coordinator
    */
    void sendOrientation(const double& roll, const double& pitch, const double& yaw)
    {
        String msg = "Orientation:";
        msg += roll;
        msg += " ";
        msg += pitch;
        msg += " ";
        msg += yaw;
        transmit2Coor(msg);
    }
    /**
     @brief :send orientation messages over xbee to coordinator
     */
    void sendLocation(const uint8_t& surfaceQuality, const double& x_cm, const double& y_cm, const double& z_cm)
    {
        String msg = "Location:";
        msg += x_cm;
        msg += " ";
        msg += y_cm;
        msg += " ";
        msg += z_cm;
        msg += " ";
        msg += surfaceQuality;
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
        S_PORT.write(messageOut, 18+message.length());
        delete[] messageOut;
        //S_PORT.flush();
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
    #ifdef COORDINATOR
        uint8_t checksum = ((0xFF) - (sum))+(0x02);
    #endif
    #ifdef DRONE
        uint8_t checksum = ((0xFF) - (sum));
    #endif
        header[17+sizet]= (checksum);
        
        return header;
    }
 
    /**
     @brief : parse headers that come in from in API mode. 
    */
    String rx_headerInter(String rx_message, unsigned int sizet)
    {
        char buf[sizet];
        //Serial.print("beforeFilter incoming:");
        //Serial.println(rx_message);
        /*rx_message.toCharArray(buf, sizet);//getBytes(buf, sizet);
        Serial.print("incoming RX:");
        for(int i = 0; i < sizet; i++)
        {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.print('\t');
        Serial.print(rx_message);
        
        Serial.println("");//*/
        #ifdef COORDINATOR
        const uint8_t dataLen = sizet - 12;// length of data in message
        const uint8_t addLen = 8; // length of address
        String out;
        out = "";
        //out.trim();
        for(int i = 0; i < addLen + dataLen ; i++)
        {
            out += rx_message[i+3];
        }
        out[7] = ':';
        
        #endif
        #ifdef DRONE
        const uint8_t dataLen = sizet - 12;// length of data in message
        String out;
        out = "";
        for(int i = 0; i < dataLen ; i++)
        {
            out += rx_message[i+10];
        }

        #endif
        
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
        }
         */
        //int total = dataLen + addLen;
        //Serial.println(out);
        
        
        return out;
        
    }
   

};

#endif

#endif /* DroneCom_h */
