package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Drive.kMaximumBias;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Drive.kMinimumBias;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Drive.kMotorNames;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
    private final SHPMecanumDrive drive;
    public final IMU imu;
    public final Encoder parallelEncoder, perpendicularEncoder;

    private double bias = kMaximumBias; // will always be between kMinimumBias and 1.0
//    final SHPMotor[] motors;
    final DcMotorEx motor0, motor1, motor2, motor3;
    final String[] motorNames = kMotorNames;
    //    private Encoder leftEncoder, rightEncoder, frontEncoder;
    //    private SHPIMU imu;
    //tracks - mod 2
    //odd is speed down - 0.3 factor
    //even is speed normal
    private int buttonClicks;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SHPMecanumDrive(hardwareMap, kMotorNames);
        //drive.enableBuiltInVelocityControl();

        // Change logo direction and USB direction according to your hub orientation
        // Reference pictures: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html#orthogonal-mounting
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));


        motor0 = (DcMotorEx) hardwareMap.get(motorNames[0]);
        motor1 = (DcMotorEx) hardwareMap.get(motorNames[1]);
        motor2 = (DcMotorEx) hardwareMap.get(motorNames[2]);
        motor3 = (DcMotorEx) hardwareMap.get(motorNames[3]);

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        motors = new SHPMotor[4];
//        for (int i = 0; i < motors.length; i++) {
//            motors[i] = new SHPMotor(hardwareMap, motorNames[i]);
//        }
//        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
//        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
//        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
//        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);


//        motors[0].disableEncoder();
//        motors[1].disableEncoder();
//        motors[2].disableEncoder();
//        motors[3].disableEncoder();
//
//        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
    }

    public void incrementButtonClicks(){
        buttonClicks++;
    }
    public void resetIMUAngle() {imu.resetYaw();}
    public void newMecanum(double leftY, double leftX, double rightX){

        Vector2d vector = new Vector2d(
                leftY,
                leftX
        ).rotated(-(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        //IF ITS NOT WORKING SWITCH VECTOR GETX AND VECTOR GETY
        double[] powers = {
                vector.getX() - vector.getY() - rightX,
                vector.getX() + vector.getY() - rightX,
                vector.getX() + vector.getY() + rightX,
                vector.getX() - vector.getY() + rightX,
        };
        motor0.setPower(powers[0]);
        motor1.setPower(powers[1]);
        motor2.setPower(powers[2]);
        motor3.setPower(powers[3]);
    }
    public void robotmecanum(double leftY, double leftX, double rightX){
        drive.mecanum(leftY, leftX, rightX);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
        double[] powers = {
                leftY - leftX - rightX,
                leftY + leftX - rightX,
                leftY + leftX + rightX,
                leftY - leftX + rightX,
        };
        Vector2d vector = new Vector2d(
                leftY,
                leftX
        ).rotated(-(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        //IF ITS NOT WORKING SWITCH VECTOR GETX AND VECTOR GETY
        drive.mecanum(vector.getX(), vector.getY(), rightX); // field oriented

//        Vector2d vector2d = new Vector2d(
//                leftY,
//                leftX
//        ).rotated(-imu.getYaw(AngleUnit.RADIANS));
//
//        motors[0].setPower(vector2d.getY() - vector2d.getX() - rightX);
//        motors[1].setPower(vector2d.getY() + vector2d.getX() - rightX);
//        motors[2].setPower(vector2d.getY() + vector2d.getX() + rightX);
//        motors[3].setPower(vector2d.getY() - vector2d.getX() + rightX);
//        if(buttonClicks%2==1) {
//
//            motors[0].setPower(0.3*(leftY + leftX - rightX));
//            motors[1].setPower(0.3*(leftY - leftX - rightX));
//            motors[2].setPower(0.3*(leftY + leftX + rightX));
//            motors[3].setPower(0.3*(leftY - leftX + rightX));
//        }
//        else{
//            motors[0].setPower(leftY + leftX - rightX);
//            motors[1].setPower(leftY - leftX - rightX);
//            motors[2].setPower(leftY + leftX + rightX);
//            motors[3].setPower(leftY - leftX + rightX);
//        }


        //        Vector2d vector = new Vector2d(
//                leftY,
//                leftX
//        ).rotated(-imu.getYaw());

//        drive.mecanum(vector.getX(), vector.getY(), rightX); // field oriented
//        drive.mecanum(leftY * bias, leftX * bias, rightX * bias); // robot oriented
//        drive.mecanum(leftY, leftX, rightX);
//        for(int i=0; i<motors.length; i++){
//            if(i==0||i==1){
//                motors[i].setPower(leftY+leftX);
//            }
//            else{
//                motors[i].setPower(leftY-leftX);
//            }
//
//        }
    }


    //    get speed up here?
    public void setDriveBias(double driveBias) {
        bias = Range.clip(driveBias, kMinimumBias, kMaximumBias);
    }
    //uncomment /* block */

//    public void enablePositionPID() {
//        drive.enablePositionPID(0.001, 0.0, 0.0);
//        drive.scheduleGains(
//                new GainSchedule(0.0005, 0.0, 0.0, 0.80),
//                new GainSchedule(0.0002, 0.0, 0.0, 0.40)
////                new GainSchedule(0.0001, 0.0, 0.0, 0.10)
//
////                new GainSchedule(0.00015, 0.0, 0.0, 0.20)
//        );
//        drive.enableFF(kFFs);
//    }

    public void setInitialPositions() {
        drive.setInitialPositions(MotorUnit.TICKS);
    }


//
//    public void driveTo(boolean usingPID, double ticks) {
//        if (usingPID) {
//            drive.setPosition(ticks);
//        } else {
//            while (drive.getWheelPositionsAveraged(MotorUnit.TICKS) <= ticks) {
//                drive.mecanum(0.5, 0.5, 0);
//            }
//            drive.mecanum(0.0, 0.0, 0);
//        }
//    }

    //    public boolean atPositionSetpoint() {
//        return drive.atPositionSetpoint();
//    }
    //uncomment later
    /*
    public void setPosition(double position) {
        drive.setPositions(position);
    }
    */
    @Override
    public void periodic(Telemetry telemetry) {
//        telemetry.addData("Bot Direction: ", Math.toDegrees(imu.getYaw()));
//        for (int i = 0; i < 4; i++) {
        // telemetry.addData("Motor " + i + " Position: ", drive.getPositions(MotorUnit.TICKS)[i]);
//        }
//        telemetry.addData("Drive at position setpoint: ", drive.atPositionSetpoint() ? "true" : "false");
//        telemetry.addData("YPos:",  encoderTicksToInches(parallelEncoder.getCurrentPosition()));
//        telemetry.addData("XPos:", encoderTicksToInches(perpendicularEncoder.getCurrentPosition()));
//        telemetry.addData("frontEncoderVal:", frontEncoder.getCurrentPosition());
        telemetry.addData("IMU ANGLE:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}
