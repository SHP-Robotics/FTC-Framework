package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Field Centric")
public class FieldCentric extends BaseTeleOp {
    public void loopOpMode() {
        super.loopOpMode();
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void loopReturnToHome() {
        super.loopReturnToHome();
    }
}
