package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.AutonomousCommands.SpitPixels;
//import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
//import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;
//import org.firstinspires.ftc.teamcode.Subsystems.Arm;
//import org.firstinspires.ftc.teamcode.Subsystems.Band;
//import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
//import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import java.util.Arrays;

@Autonomous
public class AutonomousBasicCenter extends LinearOpMode {
    Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        chassis = new Chassis(hardwareMap);


        Trajectory basicCenter = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1, 2, Rotation2d.fromDegrees(90))),
                new TrajectoryConfig(1, 0.5));


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamseteCommand(chassis, basicCenter));
        waitForStart();

        chassis.resetPose(basicCenter.getInitialPose());

        CommandScheduler.getInstance().schedule(testCommandGroup);

        while (opModeIsActive ()){
            CommandScheduler.getInstance().run();

            Pose2d pose = chassis.getPose();

            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());
            telemetry.update();


        }
    }
}
//subiendo codigo a