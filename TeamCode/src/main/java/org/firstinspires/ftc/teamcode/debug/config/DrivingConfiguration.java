package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DrivingConfiguration {
    // An example of how different controls might be mapped to different enumerations,
    // and then referenced in other classes, so any changes are global

    // Driving
    public static final GamepadAnalog STRAFE_RIGHT = GamepadAnalog.LEFT_STICK_X;
    public static final GamepadAnalog STRAFE_UP = GamepadAnalog.LEFT_STICK_Y;

    public static final GamepadAnalog ROTATE_RIGHT = GamepadAnalog.RIGHT_STICK_X;

    public static final GamepadDigital GEAR_UP = GamepadDigital.LEFT_STICK_BUTTON;
    public static final GamepadDigital GEAR_DOWN = GamepadDigital.A;

    public static final GamepadDigital RESET_IMU = GamepadDigital.B;

    // Climbing
    public static final GamepadAnalog CLIMBER_POWER = GamepadAnalog.RIGHT_TRIGGER;

    public static double getValue(Gamepad gamepad, GamepadAnalog analog) {
        switch (analog) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                // I swear to God, this is always inverted
                // I don't know why
                // NEVER change this (consult with me first)
                return -gamepad.left_stick_y;
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                // I swear to God, this is always inverted
                // I don't know why
                // NEVER change this (consult with me first)
                return -gamepad.right_stick_y;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            default:
                return 0;
        }
    }

    public static boolean getValue(Gamepad gamepad, GamepadDigital digital) {
        switch (digital) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            default:
                return false;
        }
    }
}
