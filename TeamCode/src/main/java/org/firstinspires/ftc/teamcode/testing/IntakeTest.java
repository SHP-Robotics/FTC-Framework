package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntakeSubsystem;

@Disabled
@TeleOp
public class IntakeTest extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.left_bumper == gamepad1.right_bumper)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
            else if (gamepad1.left_bumper)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTAKE);
            else
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
            intakeSubsystem.update();
        }
    }
}
