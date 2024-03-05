package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kRightSlideName;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous
public class FindBottom extends LinearOpMode {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private final double MAX_CURRENT = 0.8;
    private boolean running = true;
    @Override
    public void runOpMode(){
        leftSlide = (DcMotorEx) hardwareMap.get(kLeftSlideName);
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide = (DcMotorEx) hardwareMap.get(kRightSlideName);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        leftSlide.setPower(-1);
        rightSlide.setPower(-1);

        while (opModeIsActive() && !isStopRequested()) {
            if (!running || leftSlide.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT || leftSlide.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                break;
            }

            telemetry.addData("Left current", leftSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right current", rightSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("avg current", (leftSlide.getCurrent(CurrentUnit.AMPS) + rightSlide.getCurrent(CurrentUnit.AMPS)) / 2);
            telemetry.update();
        }
    }
}
