package com.team1701.lib.drivers;

import edu.wpi.first.wpilibj.I2C;

public class LEDController {

    private final I2C mI2c;

    public LEDController(I2C.Port port, int deviceAddress) {
        mI2c = new I2C(port, deviceAddress);

        sendColor(LightColor.BLACK);
    }

    public void sendColor(LightColor color) {
        // sendColor(LightColor.RED, LightColor.BLUE, LightColor.GREEN, LightColor.VIOLET);
        sendColor(color, color, color, color);
    }

    public void sendColor(LightColor front, LightColor back) {
        sendColor(front, front, back, back);
    }

    public void sendColor(LightColor frontLeft, LightColor frontRight, LightColor backLeft, LightColor backRight) {

        var dataToSend = new byte[] {
            4,
            frontLeft.red,
            frontLeft.green,
            frontLeft.blue,
            3,
            frontRight.red,
            frontRight.green,
            frontRight.blue,
            2,
            backLeft.red,
            backLeft.green,
            backLeft.blue,
            1,
            backRight.red,
            backRight.green,
            backRight.blue
        };

        mI2c.writeBulk(dataToSend, dataToSend.length);
    }

    public enum LightColor {
        BLACK((byte) 0, (byte) 0, (byte) 0),
        WHITE((byte) 255, (byte) 255, (byte) 255),
        RED((byte) 255, (byte) 0, (byte) 0),
        ORANGE((byte) 255, (byte) 55, (byte) 0),
        YELLOW((byte) 255, (byte) 200, (byte) 0),
        GREEN((byte) 0, (byte) 255, (byte) 0),
        CYAN((byte) 0, (byte) 255, (byte) 255),
        BLUE((byte) 0, (byte) 0, (byte) 255),
        VIOLET((byte) 127, (byte) 0, (byte) 255),
        MAGENTA((byte) 255, (byte) 0, (byte) 255);

        private final byte red;
        private final byte blue;
        private final byte green;

        LightColor(byte red, byte blue, byte green) {
            this.red = red;
            this.blue = blue;
            this.green = green;
        }
    }
}
