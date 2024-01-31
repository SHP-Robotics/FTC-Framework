package org.firstinspires.ftc.teamcode.autos.purepursuit;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class BasicTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");

        // input motors exactly as shown below
        MecanumDrive mecanumDrive = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);

        double trackwidth = 11.681529572799718;
        double centerWheelOffset = -6;
        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(trackwidth, centerWheelOffset);

        Waypoint p1 = new StartWaypoint(0, 0);
        Waypoint p2 = new GeneralWaypoint(10, 0);
        Waypoint p3 = new EndWaypoint();

        



        m_path.followPath(mecanumDrive, holonomicOdometry);
    }
}
