package org.firstinspires.ftc.teamcode.lib.wpilib;

import com.qualcomm.robotcore.hardware.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandGamepad {
    public Gamepad gamepad;

    public CommandGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public Trigger a() {
        return new Trigger(() -> gamepad.a);
    }

    public Trigger b() {
        return new Trigger(() -> gamepad.b);
    }

    public Trigger x() {
        return new Trigger(() -> gamepad.x);
    }

    public Trigger y() {
        return new Trigger(() -> gamepad.y);
    }

    public Trigger start() {
        return new Trigger(() -> gamepad.start);
    }

    public Trigger back() {
        return new Trigger(() -> gamepad.back);
    }

    public Trigger leftBumper() {
        return new Trigger(() -> gamepad.left_bumper);
    }

    public Trigger rightBumper() {
        return new Trigger(() -> gamepad.right_bumper);
    }

    public Trigger leftStick() {
        return new Trigger(() -> gamepad.left_stick_button);
    }

    public Trigger rightStick() {
        return new Trigger(() -> gamepad.right_stick_button);
    }

    public Trigger dpadUp() {
        return new Trigger(() -> gamepad.dpad_up);
    }

    public Trigger dpadDown() {
        return new Trigger(() -> gamepad.dpad_down);
    }

    public Trigger dpadLeft() {
        return new Trigger(() -> gamepad.dpad_left);
    }

    public Trigger dpadRight() {
        return new Trigger(() -> gamepad.dpad_right);
    }

    public Trigger leftTrigger() {
        return new Trigger(() -> gamepad.left_trigger > 0.2);
    }

    public Trigger rightTrigger() {
        return new Trigger(() -> gamepad.right_trigger > 0.2);
    }

    public double getLeftX() {
        return gamepad.left_stick_x;
    }

    public double getLeftY() {
        return gamepad.left_stick_y;
    }

    public double getRightX() {
        return gamepad.right_stick_x;
    }

    public double getRightY() {
        return gamepad.right_stick_y;
    }

    public double getLeftTrigger() {
        return gamepad.left_trigger;
    }

    public double getRightTrigger() {
        return gamepad.right_trigger;
    }
}
