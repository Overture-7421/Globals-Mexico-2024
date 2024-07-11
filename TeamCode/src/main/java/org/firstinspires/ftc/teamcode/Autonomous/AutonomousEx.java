package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
//robot core imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Teamcode imports

import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;


import java.util.Arrays;

@Autonomous
public class AutonomousEx extends LinearOpMode {
    Chassis chassis;

    //creo que vuela
    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        chassis = new Chassis(hardwareMap);


        TrajectoryConfig redWETopConfig = new TrajectoryConfig(0.5, 0.2);
        redWETopConfig.setReversed(false);
        Trajectory UP = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(2.6,0,Rotation2d.fromDegrees(0))), redWETopConfig
        );


        TrajectoryConfig hola = new TrajectoryConfig(0.5, 0.2);
        hola.setReversed(false);
        Trajectory Right = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(2.6,0,Rotation2d.fromDegrees(90)),
                new Pose2d(2.6,1,Rotation2d.fromDegrees(90))), hola
        );

        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamseteCommand(chassis, UP),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(90)),
                //new MoveSinglearm(singleArm, 30),
                new RamseteCommand(chassis, Right)
        );

        waitForStart();

        chassis.resetPose(UP.getInitialPose());

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