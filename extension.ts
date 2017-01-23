namespace microbit_GY521 {
    /**
     * The initialise function for the GY521 must be called once
     * before any other GY521 function. Call it at the start of your program
     */
    //% shim=microbit_GY521::initialise
    export function initialise() { return }
    /**
     * Calibrates each sensor from a series of readings
     */
    //% shim=microbit_GY521::calibrate_Sensors    
    export function calibrate_Sensors() { return }  
    /**
     * Validates I2C connectivity with the device using default address
     */
    //% shim=microbit_GY521::testConnection    
    export function testConnection() { return false }  
    /**
     * A complementary filtered X angle value in degrees
     */
    //% shim=microbit_GY521::computeX    
    export function computeX() { return 0 }  
    /**
     * A complementary filtered Y angle value in degrees
     */
    //% shim=microbit_GY521::computeY    
    export function computeY() { return 0 }
    /**
     * Sets up the X Gyro offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setXGyroOffset    
    export function setXGyroOffset(value: number ) { return }
    /**
     * Sets up the Y Gyro offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setYGyroOffset    
    export function setYGyroOffset(value: number) { return }
    /**
     * Sets up the Z Gyro offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setZGyroOffset    
    export function setZGyroOffset(value: number) { return }
     /**
     * Sets up the Z accelerometer offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setZAccelOffset    
    export function setZAccelOffset(value: number) { return }   
     /**
     * Sets up the Y accelerometer offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setYAccelOffset    
    export function setYAccelOffset(value: number) { return }   
     /**
     * Sets up the X accelerometer offset of this device
     * Determine your device offset by running tests
     */
    //% shim=microbit_GY521::setXAccelOffset    
    export function setXAccelOffset(value: number) { return }   
     /**
     * Returns the Z gyro offset set on this device
     */
    //% shim=microbit_GY521::getZGyroOffset    
    export function getZGyroOffset() { return 0 }   
     /**
     * Returns the Y gyro offset set on this device
     */
    //% shim=microbit_GY521::getYGyroOffset    
    export function getYGyroOffset() { return 0 }   
     /**
     * Returns the X gyro offset set on this device
     */
    //% shim=microbit_GY521::getXGyroOffset    
    export function getXGyroOffset() { return 0 }   
    /**
     * Returns the Y accelerometer offset set on this device
     */
    //% shim=microbit_GY521::getYAccelOffset    
    export function getYAccelOffset() { return 0 }  
    /**
     * Returns the X accelerometer offset set on this device
     */
    //% shim=microbit_GY521::getXAccelOffset    
    export function getXAccelOffset() { return 0 }  
    /**
     * Returns the Z accelerometer offset set on this device
     */
    //% shim=microbit_GY521::getZAccelOffset    
    export function getZAccelOffset() { return 0 }  
     /**
     * Reads the gyro and accelerometer registers of the device
     * Use the Read functions to obtain the results
     */
    //% shim=microbit_GY521::getMotion6    
    export function getMotion6() { return }  
    /**
     * Returns integer container for accelerometer X-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readAccelX    
    export function readAccelX() { return 0 }  
     /**
     * Returns integer container for accelerometer Y-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readAccelY    
    export function readAccelY() { return 0 }  
    /**
     * Returns integer container for accelerometer Z-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readAccelZ    
    export function readAccelZ() { return 0 }  
        /**
     * Returns integer container for gyro X-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readGyroX    
    export function readGyroX() { return 0 }  
    /**
     * Returns integer container for gyro Y-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readGyroY    
    export function readGyroY() { return 0 }  
    /**
     * Returns integer container for gyro Z-axis value
     * use after a call to getMotion6
     */
    //% shim=microbit_GY521::readGyroZ    
    export function readGyroZ() { return 0 }  
}
