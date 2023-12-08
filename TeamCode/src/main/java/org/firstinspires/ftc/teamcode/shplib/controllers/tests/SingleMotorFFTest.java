package org.firstinspires.ftc.teamcode.shplib.controllers.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.util.RegressionUtil;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

import java.util.ArrayList;

@Autonomous(group = "ff")
public class SingleMotorFFTest extends LinearOpMode {

    public static String MOTOR_NAME = "leftFront";
    public static double LIMIT_TICKS = 10000;
    public static double VOLTAGE_RAMP = 0.25; // V/s

    @Override
    public void runOpMode() {
        SHPMotor motor = new SHPMotor(hardwareMap, MOTOR_NAME);
        ElapsedTime timer = new ElapsedTime();

        ArrayList<Double> positions = new ArrayList<>();
        ArrayList<Double> powers = new ArrayList<>();
//        ArrayList<Double> velocities = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();

        telemetry.addLine("Press start to begin the test.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("Running test...");
        telemetry.addLine("Press stop to end the test.");
        telemetry.update();

        motor.resetEncoder();
        timer.reset();

        while (opModeIsActive() && motor.getPosition(MotorUnit.TICKS) < LIMIT_TICKS) {
            double seconds = timer.seconds();
            double power = seconds * VOLTAGE_RAMP / Constants.kNominalVoltage;

            times.add(timer.seconds());
            positions.add((double) motor.getPosition(MotorUnit.TICKS));
            powers.add(power);
//            double velocity = ANGLE_UNIT == null ? motor.getVelocityTicks() : motor.getVelocity(ANGLE_UNIT);
//            telemetry.addData("velocity: ", velocity);
//            velocities.add(velocity);
            motor.setPower(power);
        }
        motor.setPower(0);

        telemetry.clearAll();

        RegressionUtil.RampResult result = RegressionUtil.fitRampData(times, positions, powers, true, null);
        telemetry.addData("kV: ", result.kV);
        telemetry.addData("kS: ", result.kStatic);
        telemetry.addData("R^2 (Accuracy): ", result.rSquare);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();

//        LinearRegression graph = new LinearRegression(times.stream().mapToDouble(d -> d).toArray(), velocities.stream().mapToDouble(d -> d).toArray());
//        telemetry.addData("kV: ", graph.slope());
    }
}
