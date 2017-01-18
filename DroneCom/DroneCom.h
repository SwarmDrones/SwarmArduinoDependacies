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

//#define PROCESSING
#define UNITY

#define SERIAL_COM
//#define WIRELESS

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

#elif WIRELESS
/**@todo Mike Xbee serial print out goes here*/
#endif

#endif /* DroneCom_h */
