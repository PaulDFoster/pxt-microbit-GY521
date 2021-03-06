
//let offSets: number[] = [2791,593,1065,122,-29,1] // GY-521

let offSets: number[] = [-2694,593,485,75,15,21] // MPU-6050
let selfCal: boolean = microbit_GY521.calibrate_Sensors(offSets); // required to increase accuracy of readings

if (!selfCal) {
    basic.showLeds(`
	# . . # .
	# . # # #
	# . . # .
	# . # # #
	# . . # .
	`)
    basic.pause(30000);
}

let y: number = 0
let speed: number = 0

// Motor state setup
// Use to trim motors if one is stronger than the other
let leftMotorBias = 0;
let rightMotorBias = 0;
// Manage baseline motor speed ie the value from which motor values have an affect
let motorMin = 140;
// PID state setup
let setPoint = -1;// Now set dynamically on start up
let Kp = 100; //102
let Ki = 76;  //68 
let Kd = 52;  //60
let outMax = 1023;
let outMin = -1023;
let lastInput = 0;
let lastTime = 0;
let accumulative_error : number = 0
let last_error: number = 0
let Actuator_Output: number = 0

led.plotBarGraph(1000, 1023);
basic.pause(500)
led.plotBarGraph(750, 1023);
basic.pause(500)
led.plotBarGraph(500, 1023);
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

input.onButtonPressed(Button.A, () => {
    setPoint -= 1;
});

input.onButtonPressed(Button.B, () => {
    setPoint += 1;
});

let skipDebug : number = 0;
setPoint = microbit_GY521.computeY();

while (1) {

    basic.pause(2); // keep to 200hz
    y = microbit_GY521.computeY()

    if (y > 360) {
        basic.showLeds(`
            # . . . .
            . # . # .
            . # . . .
            . # . # .
            # . . . .
            `);
        
        break;
    }

    speed = PID(y)    

    skipDebug++;
    

    if (y > -80 && y < 80 ) {
        // This prevents error in the PID from accumulating when robot laying down
        motorController(speed)
    } else {
        motorStop()
    }

}

function PID(input:number) : number
{

    let now = game.currentTime(); // in milliseconds
    let timeChange = (now - lastTime)/1000; //convert to seconds

/*
//  PID algorithm
        Actuator_Output =
            Kp * (distance from goal) 
        + Ki * (accumulative error)
        + Kd * (change in error)
*/

        let distance_from_goal: number = (setPoint - input) // P
        accumulative_error = accumulative_error + (distance_from_goal*timeChange) // I
        let change_in_error = (distance_from_goal- last_error)/timeChange // D
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
   
   pins.analogWritePin(AnalogPin.P0, Math.abs(speed + rightMotorBias))
   pins.analogSetPeriod(AnalogPin.P0,2500) 
   pins.analogWritePin(AnalogPin.P1, Math.abs(speed + leftMotorBias))
   pins.analogSetPeriod(AnalogPin.P1,2500) 
 
}