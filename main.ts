microbit_GY521.calibrate_Sensors(); // required to increase accuracy of readings

let y: number = 0
let x: number = 0
let speed: number = 0
let bigSwitch = true;
let releaseSwitch = false;

// PID state setup
let lastTime = 0;
let setPoint = 179
let motorMin = 161
let Kp = 110
let Ki = 11                                                                                                                                                                                                                                    
let Kd = 290; //14
let outMax = 1023;
let outMin = -1023;
let lastInput = 0;
let currentSpeed = 0;

let accumulative_error : number = 0
let last_error: number = 0
let Actuator_Output: number = 0

led.plotBarGraph(1000, 1023);
basic.pause(500)
led.plotBarGraph(750, 1023);
basic.pause(500)
led.plotBarGraph(500, 1023)
basic.pause(500)
led.plotBarGraph(250, 1023)
basic.pause(500)

basic.showLeds(`
	. # . . .
	# . . # .
	# . . . .
	# . . # .
	. # . . .
	`)


//setPoint = microbit_GY521.computeY();

while (1) {

    y = microbit_GY521.computeY()

    speed = PID(y)    
   
    if (y > 145 && y < 215) {
        // This prevents error in the PID from accumulating when robot laying down
        motorController(speed)
    } else {
        motorStop()
    }

}

function PID(input:number) : number
{

    let now = game.currentTime();
    let timeChange = (now - lastTime)/1000; //convert to seconds

/*
Actuator_Output =
    Kp * (distance from goal) 
  + Ki * (accumulative error)
  + Kd * (change in error)
        */

        let distance_from_goal: number = (setPoint - input)
        accumulative_error = accumulative_error + (distance_from_goal/timeChange)
        let change_in_error = last_error - distance_from_goal
        last_error = distance_from_goal
        Actuator_Output = Kp * distance_from_goal + Ki * accumulative_error + Kd * change_in_error

// A base motor speed to over come motor inertia    
        if (Actuator_Output < 0) {
            Actuator_Output = Actuator_Output - motorMin;
        }
        else {
            Actuator_Output = Actuator_Output + motorMin;    
    }
    
// Clamp actuator output to motor range        
        if (Actuator_Output > outMax) {
            Actuator_Output = outMax
        }
        if (Actuator_Output < outMin) {
            Actuator_Output = outMin
        }
               
        lastInput = input;
        lastTime = now;

   
    return Actuator_Output;
}

function motorStop() {
        pins.digitalWritePin(DigitalPin.P13, 0)
        pins.digitalWritePin(DigitalPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P15, 0)
        pins.digitalWritePin(DigitalPin.P16, 0)   
}

function motorController(speed: number)
{
    
    if (speed > 0) {
        pins.digitalWritePin(DigitalPin.P13, 1)
        pins.digitalWritePin(DigitalPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P15, 1)
        pins.digitalWritePin(DigitalPin.P16, 0)

    }
    if (speed < 0) {

        pins.digitalWritePin(DigitalPin.P13, 0)
        pins.digitalWritePin(DigitalPin.P14, 1)
        pins.digitalWritePin(DigitalPin.P15, 0)
        pins.digitalWritePin(DigitalPin.P16, 1)
    }
   
   pins.analogWritePin(AnalogPin.P0, Math.abs(speed))
   pins.analogSetPeriod(AnalogPin.P0,2000) 
   pins.analogWritePin(AnalogPin.P1, Math.abs(speed))
   pins.analogSetPeriod(AnalogPin.P1,2000) 
 
}