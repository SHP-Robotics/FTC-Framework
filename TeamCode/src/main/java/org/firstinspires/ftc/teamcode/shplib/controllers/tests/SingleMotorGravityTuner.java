package org.firstinspires.ftc.teamcode.shplib.controllers.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

@Autonomous(group = "ff")
public class SingleMotorGravityTuner extends LinearOpMode {

    private final String kMotorName = "slide";
    private final double kS = 0.035;

    private final double kDebounceTime = 100;

    @Override
    public void runOpMode() {
        ElapsedTime debouncer = new ElapsedTime();
        SHPMotor motor = new SHPMotor(hardwareMap, kMotorName);
        motor.enableFF(new FFController(kS));
        motor.reverseDirection(); // delete later, dependent on motor

        double power = 0.0;

        telemetry.addLine("Press start to begin tuning.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (debouncer.milliseconds() < kDebounceTime) continue;

            if (gamepad1.dpad_up) {
                power += 0.001;
                debouncer.reset();
            } else if (gamepad1.dpad_down) {
                power -= 0.001;
                debouncer.reset();
            }

            motor.setPower(power);

            telemetry.addData("Power (kG): ", power);
            telemetry.addData("Power (Total): ", motor.getPower());
            telemetry.addData("Velocity (ticks/s): ", motor.getVelocity(MotorUnit.TICKS));
            telemetry.update();
        }
    }
}
