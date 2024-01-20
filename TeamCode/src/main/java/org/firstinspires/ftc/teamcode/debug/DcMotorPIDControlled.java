package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;

public class DcMotorPIDControlled extends DcMotorImplEx {
    private final double MAX_VELOCITY = 15;

    private ElapsedTime elapsedTime;

    private double lastError = 0;
    private double lastTime = -1;

    private final double kP = 0.5;
    private final double kD = -0.005;

    public DcMotorPIDControlled(DcMotorController controller, int portNumber) {
        super(controller, portNumber);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public DcMotorPIDControlled(DcMotor dcMotor) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double power) {
        double error = MAX_VELOCITY*power - this.getVelocity(AngleUnit.RADIANS);
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;
        double slope = error - lastError / (deltaTime);

        super.setPower(-clamp((error * kP * deltaTime) + (slope * kD * deltaTime)));

        lastError = error;
        lastTime = time;
    }
}