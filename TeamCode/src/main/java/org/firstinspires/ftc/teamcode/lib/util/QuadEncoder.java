package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class QuadEncoder {
    DcMotorEx motor;
    private double offset;
    public QuadEncoder(DcMotorEx motor, double startingPos) {
        this.motor = motor;
        this.offset = startingPos;
    }

    public double getCurrentVelo() {
        return motor.getVelocity();
    }

    public double getCurrentPos() {
        return motor.getCurrentPosition()-offset;
    }
}
