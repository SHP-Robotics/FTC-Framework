package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kClawClosed;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kClawName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kClawOpen;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kRightSlideName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideBottom;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideHigh;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideHub;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideLow;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideMiddle;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideStackDistance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.shplib.commands.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends Subsystem {
    private final Servo claw;
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;

    private double clawPosition;

    private int stackIndex = 4;

    public enum State {
        BOTTOM, HUB, LOW, MIDDLE, HIGH, STACK
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
        claw = (Servo) hardwareMap.get(kClawName);

        leftSlide = (DcMotor) hardwareMap.get(kLeftSlideName);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightSlide = (DcMotor) hardwareMap.get(kRightSlideName);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setState(State.BOTTOM);
    }

    public void setState(State state) {
        this.state = state;
    }

    public double getDriveBias() {
        return Math.abs(getSlidePosition() / kSlideHigh - 1.0);
    }

    public void openClaw() {
        claw.setPosition(kClawOpen);
        clawPosition = kClawOpen;
        if (atStacks()) stackIndex++;
    }

    public void closeClaw() {
        claw.setPosition(kClawClosed);
        clawPosition = kClawClosed;
        if (atStacks() && stackIndex > 0) stackIndex--;
    }

    public boolean clawClosed() {
        return clawPosition == kClawClosed;
    }

    public boolean atStacks() {
        return state == State.STACK;
    }

    public boolean atHub() {
        return state == State.HUB;
    }

    public boolean atSetpoint() {
        return !leftSlide.isBusy() && !rightSlide.isBusy();
    }

    public double getSlidePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
    }

    private void processState() {
        switch (state) {
            case BOTTOM:
                leftSlide.setTargetPosition((int) kSlideBottom);
                rightSlide.setTargetPosition((int) kSlideBottom);
                break;
            case HUB:
                leftSlide.setTargetPosition((int) kSlideHub);
                rightSlide.setTargetPosition((int) kSlideHub);
                break;
            case LOW:
                leftSlide.setTargetPosition((int) kSlideLow);
                rightSlide.setTargetPosition((int) kSlideLow);
                break;
            case MIDDLE:
                leftSlide.setTargetPosition((int) kSlideMiddle);
                rightSlide.setTargetPosition((int) kSlideMiddle);
                break;
            case HIGH:
                leftSlide.setTargetPosition((int) kSlideHigh);
                rightSlide.setTargetPosition((int) kSlideHigh);
                break;
            case STACK:
                leftSlide.setTargetPosition((int) (stackIndex * kSlideStackDistance + kSlideBottom));
                rightSlide.setTargetPosition((int) (stackIndex * kSlideStackDistance + kSlideBottom));
                break;
        }
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Claw Position: ", claw.getPosition());
        telemetry.addData("Num Cones Stacked: ", stackIndex + 1);
        telemetry.addData("Left Slide Position: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position: ", rightSlide.getCurrentPosition());
    }
}