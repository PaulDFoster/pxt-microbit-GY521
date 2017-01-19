// tests go here; this will not be compiled when this package is used as a library

// This code assumes the a microbit_GY521.initialise has been run in the main

if (microbit_GY521.testConnection())
    serial.writeLine("GY521 connected");
else
    serial.writeLine("GY521 not found");

// supply your own gyro offsets here, scaled for min sensitivity
// These don't appear to get set, as the get values differ
microbit_GY521.setXGyroOffset(-96);
let xgo = microbit_GY521.getXGyroOffset();
serial.writeString("xgo -96 ");
serial.writeNumber(xgo);
serial.writeLine(" ");
microbit_GY521.setYGyroOffset(54); //54
let ygo = microbit_GY521.getYGyroOffset();
serial.writeString("ygo 54 ");
serial.writeNumber(ygo);
serial.writeLine(" ");
microbit_GY521.setZGyroOffset(87); //128
let zgo = microbit_GY521.getZGyroOffset();
serial.writeString("zgo 87 ");
serial.writeNumber(zgo);
serial.writeLine(" ");
microbit_GY521.setZAccelOffset(1393); // 1688 factory default for my test chip
let zao = microbit_GY521.getZAccelOffset();
serial.writeString("zao 1393 ");
serial.writeNumber(zao);
serial.writeLine(" ");

//Test raw reads
microbit_GY521.getMotion6();
serial.writeNumber(microbit_GY521.readGyroX());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readGyroY());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readGyroZ());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelX());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelY());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelZ());
serial.writeLine(" ");

basic.pause(3000);