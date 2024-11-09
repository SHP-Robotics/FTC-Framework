package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Config
@TeleOp
public class SetMotor090 extends LinearOpMode {
    double floatOrBreak = 0.8;
    double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        DcMotor motor = (DcMotor) hardwareMap.get("motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (LynxModule hub : hubs) {
            LynxCommand command = new UnhingedSetMotorChannelModeCommand(hub, motor.getPortNumber(), motor.getMode(), floatOrBreak);
            try {
                command.send();
            } catch (InterruptedException | RuntimeException | LynxNackException e) {
                throw new RuntimeException(e);
            }
        }

        waitForStart();

        motor.setPower(power);
    }
}
