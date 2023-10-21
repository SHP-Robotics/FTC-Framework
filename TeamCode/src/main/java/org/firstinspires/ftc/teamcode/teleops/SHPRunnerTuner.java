package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.Constants.Drive.kMotorNames;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class SHPRunnerTuner extends BaseRobot {
    private SHPRunnerMecanumDrive runner;
    private final ArrayList<double[]> store = new ArrayList<>();
    private double debounce;

    @Override
    public void init() {
        configure();
        runner = new SHPRunnerMecanumDrive(hardwareMap, kMotorNames, 0.0, 0.0, 0.0, 0.0);
        runner.enableBuiltInVelocityControl();
    }

    @Override
    public void start() {
        debounce = Clock.now();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            runner.mecanum(0.2, 0.0, 0.0);
        } else if (gamepad1.dpad_down) {
            runner.mecanum(-0.2, 0.0, 0.0);
        } else if (gamepad1.dpad_left) {
            runner.mecanum(0.0, 0.2, 0.0);
        } else if (gamepad1.dpad_right) {
            runner.mecanum(0.0, -0.2, 0.0);
        } else if (gamepad1.left_bumper) {
            runner.mecanum(0.0, 0.0, 0.2);
        } else if (gamepad1.right_bumper) {
            runner.mecanum(0.0, 0.0, -0.2);
        } else {
            runner.mecanum(-gamepad1.left_stick_y / 2.0, -gamepad1.left_stick_x / 2.0, -gamepad1.right_stick_x / 2.0);
        }

        runner.log(telemetry);
        telemetry.addLine();
        for (int i = 0; i < store.size(); i++) {
            telemetry.addData("Step " + i + ": ", Arrays.toString(store.get(i)));
        }

        // press A to store/log step
        if (gamepad1.a && Clock.hasElapsed(debounce, 1.0)) {
            double[] positions = runner.getPositions(MotorUnit.TICKS);
//            store.add(positions);
//            if (store.size() == 0) store.add(positions);
//            else {
//            double[] positionDifferences = new double[positions.length];
//            for (int i = 0; i < positions.length; i++) {
////                    positionDifferences[i] = -(positions[i] - store.get(store.size() - 1)[i]);
//                positionDifferences[i] = positions[i];
//            }
            store.add(positions);
//            }
            debounce = Clock.now();
            runner.resetEncoders();
        }

        // press B to reset encoders
        if (gamepad1.b) runner.resetEncoders();

        // press X to clear all steps
        if (gamepad1.x) store.clear();
    }
}
