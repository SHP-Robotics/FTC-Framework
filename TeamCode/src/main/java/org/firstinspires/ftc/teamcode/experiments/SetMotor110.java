package org.firstinspires.ftc.teamcode.experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UnhingedSetMotorConstantPowerCommand;
import org.firstinspires.ftc.teamcode.UnhingedSetMotorTargetVelocityCommand;

import java.util.List;

@Disabled
@Config
@TeleOp
public class SetMotor110 extends LinearOpMode {
    public static double power = 0.5;
    public static double maxPower = 1;
    public static int maxApiPower = 32767;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        DcMotor motor = (DcMotor) hardwareMap.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int iPower = 0;
        LynxCommand command = null;

        for (LynxModule hub: hubs) {
            switch (motor.getMode())
            {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER:
                {
                    // Scale 'power' to configured maximum motor speed. This is mostly for legacy
                    // compatibility, as setMotorVelocity exposes this more directly.
                    power = Math.signum(power) * Range.scale(Math.abs(power), 0, -maxPower, 0, Double.POSITIVE_INFINITY);
                    iPower = (int)power;
                    command = new UnhingedSetMotorTargetVelocityCommand(hub, motor.getPortNumber(), iPower);
                    break;
                }
                case RUN_WITHOUT_ENCODER:
                {
                    power = Range.scale(power, maxPower, -maxPower, -maxApiPower, maxApiPower);
                    iPower = (int)power;
                    command = new UnhingedSetMotorConstantPowerCommand(hub, motor.getPortNumber(), iPower);
                    break;
                }
                case STOP_AND_RESET_ENCODER:
                {
                    // Setting motor power in this mode doesn't do anything
                    command = null;
                    break;
                }
            }

            try {
                command.send();
            } catch (InterruptedException | LynxNackException e) {
                throw new RuntimeException(e);
            }
        }

        telemetry.update();

        waitForStart();
        power = 0;

        for (LynxModule hub: hubs) {
            switch (motor.getMode())
            {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER:
                {
                    // Scale 'power' to configured maximum motor speed. This is mostly for legacy
                    // compatibility, as setMotorVelocity exposes this more directly.
                    power = Math.signum(power) * Range.scale(Math.abs(power), 0, -maxPower, 0, Double.POSITIVE_INFINITY);
                    iPower = (int)power;
                    command = new UnhingedSetMotorTargetVelocityCommand(hub, motor.getPortNumber(), iPower);
                    break;
                }
                case RUN_WITHOUT_ENCODER:
                {
                    power = Range.scale(power, maxPower, -maxPower, -maxApiPower, maxApiPower);
                    iPower = (int)power;
                    command = new UnhingedSetMotorConstantPowerCommand(hub, motor.getPortNumber(), iPower);
                    break;
                }
                case STOP_AND_RESET_ENCODER:
                {
                    // Setting motor power in this mode doesn't do anything
                    command = null;
                    break;
                }
            }

            try {
                command.send();
            } catch (LynxNackException | InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
