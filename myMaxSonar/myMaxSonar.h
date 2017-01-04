/** @file myMaxSonar.h
*   @brief MaxSonar sensor class. 
    - Used for localization in combination with other MaxSonar sensors or the opticalflow sensor.
*   @author Ariel Feliz(aafeliz)
*   @date 11/17/16
*   @todo create setters and getters, and move attributes into private section
*   @todo have the update function also compensate for yaw
*/

#ifndef myMaxSonar_h
#define myMaxSonar_h

inline double cm2in(double a){return (a*0.393701);}

inline double in2cm(double a){return (a*2.54);}

class myMaxSonar
{
private:
    uint8_t sonarPin;
    double altOffset;
    
    
public:
    double alt_in;  // inches
    double alt_cm;  // centimeters
    
    double px_cm, py_cm; //previous values // not sure if needed
    double y_cm, x_cm;
    double dy_cm, dx_cm; // displacement
    double ix_cm, iy_cm;// integral
    
    /**@brief constructor can be used when sensor used in combinaion with optical flow sensor
     */
    myMaxSonar()
    {
        sonarPin = 9;
        altOffset = 10.0;
        
        alt_in = 0.00;  // inches
        alt_cm = 0.00;  // centimeters
        
        px_cm = 0.00; // not sure if needed
        py_cm = 0.00; // not sure if needed
        y_cm = 0.00;
        x_cm = 0.00;
        dy_cm = 0.00;
        dx_cm = 0.00;
        ix_cm = 0.00;
        iy_cm = 0.00;
        
        
    }
    /**@brief constructor used when sensor is used in combination with other maxsonar sensors
     */
    myMaxSonar(uint8_t sonarPin): sonarPin(sonarPin)
    {
        altOffset = 10.0;
        alt_in = 0.00;  // inches
        alt_cm = 0.00;  // centimeters
        
        px_cm = 0.00; // not sure if needed
        py_cm = 0.00; // not sure if needed
        y_cm = 0.00;
        x_cm = 0.00;
        dy_cm = 0.00;
        dx_cm = 0.00;
        ix_cm = 0.00;
        iy_cm = 0.00;
    }
    
    void updateLocation(const double& roll, const double& pitch, const double& droll, const double& dpitch)
    {
        alt_in = analogRead(sonarPin)/2.0;
        if(alt_in != 0)
        {
            alt_cm = in2cm(alt_in - altOffset); // 10usec = 1 cm of distance for LIDAR-Lite
            px_cm = x_cm; // not sure if needed
            py_cm = y_cm; // not sure if needed
            
            y_cm = alt_cm*sin(radians(roll));
            x_cm = alt_cm*sin(radians(pitch));
            
            dy_cm = alt_cm*sin(radians(droll));//x_xm - px_cm;
            dx_cm = alt_cm*sin(radians(dpitch));//y_cm - py_cm;
            
            ix_cm += dx_cm;
            iy_cm += dy_cm;
        }
    }
    
    
};

#endif /* myMaxSonar_h */
