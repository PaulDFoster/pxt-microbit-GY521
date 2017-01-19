basic.showLeds(`
    . . . # .
    . # . . #
    . . . . #
    . # . . #
    . . . # .
    `);


microbit_GY521.initialise();

microbit_GY521.calibrate_Sensors(); // required to increase accuracy of readings

let y: number = 0
let x: number = 0
let speed: number = 0
let bigSwitch = false;
let releaseSwitch = false;

// PID state setup
let lastTime = 0;
let SampleTime = 10;
let output = 0;

// ************************************
// Set point 190, Kp 70, Ki 275 Kd 0. Falls over
//           190, Kp 60, Ki 275 kd 0, 190 top far, over reaching
//           188, Kp 60, Ki 275 Kd 0, 188 to fat, still over reaching
//           186, Kp 60, Ki 275, kd0, 186 seemed almost too less
//           186, Kp 70, Ki 275, kd0 feels very close, under reaching
//           187, Kp 70, Ki 275, kd 0 less stable then 186
//           186, Kp 80, ki 275, kd 0 shorter oscillation of wheels to and fro
//           186, Kp 80, Ki 300, kd 0 almost
//           186, kp 90, Ki 0, kd 0 quite neat
//           186, kp 100 ki 0 kd 0 feels like set point should be greater
//           187, kp 100, ki 0 kd 0 most stable for Kp
//           187, kp 110, ki 0 kd 0 ditto
//           188, kp 110, ki 0 kd 0 not as good as 187
//           187, kp 120, ki, 0, kd 0 hmmm good
//           187, kp 130, ki 0, kd 0 needs greater angle
//           188, kp 130, ki 0, kd 0 ditto
//           187, kp 120, ki 0, kd 0 hmmm
//           187, kp 120, ki 0, kd 0
//           188, kp 120, ki 240, kd 0
//           190, kp 110, ki 280 aggitated, 260 better
//           190, kp 110, ki 220 interesting
//           190, kp 110, ki 230 interesting
//           190, kp 110, ki 235, kd 2 almost
//           190, kp 110, ki 235, kd 3?

let setPoint = 190;
let Kp = 110; 
let Ki = 235;                                                                                                                                                                                                                                     
let Kd = 0;
let ITerm = 0;
let outMax = 255;
let outMin = -255;
let lastInput = 0;

while (1)
{
    input.onButtonPressed(Button.A, () => { releaseSwitch = true; });
    input.onButtonPressed(Button.B, () => { releaseSwitch = false; });
    
    x = microbit_GY521.computeX()
    y = microbit_GY521.computeY()
    serial.writeNumber(x);
    serial.writeString(" ");
    serial.writeNumber(y);
    serial.writeString(" ");

    // Bug in Angle code is causing int overflow
    // and of device falls over, stop the motors. This simplifies having no off switch
    if (y > 140 && y < 230) {
        bigSwitch = true;
    }
    else {
        bigSwitch = false;
    }

    if (releaseSwitch) bigSwitch = true;

    speed = PID(x)
    serial.writeNumber(speed);
    serial.writeString(" ");

    speed = PID(y)
    serial.writeNumber(speed);
    serial.writeLine(" ");

    motorStop();

  /*  
    if(bigSwitch){
        motorController(speed);
    }
    else {
        serial.writeLine(" Angle overflow, motors stopped");
        motorStop();
    }
  */
}

function PID(input:number) : number
{

    let now = game.currentTime();
    let timeChange = (now - lastTime);

    if (timeChange >= SampleTime)
    {

        let error = setPoint - input;
        ITerm += (Ki * error);
        if (ITerm > outMax)
            ITerm = outMax;
        else if (ITerm < outMin)
            ITerm = outMin;
        let dInput = (input - lastInput);
        /*Compute PID Output*/
        output = Kp * error + ITerm - Kd * dInput;

        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;

        lastInput = input;
        lastTime = now;

   }
    return output;
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
 
    pins.servoWritePin(AnalogPin.P0, speed)
    pins.servoWritePin(AnalogPin.P1, speed)
 
}