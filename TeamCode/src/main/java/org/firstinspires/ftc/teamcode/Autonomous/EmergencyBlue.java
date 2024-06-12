package org.firstinspires.ftc.teamcode.Autonomous;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Commands.MoveDoubleArm;
import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.AutonomousCommands.GoDown;


@Autonomous
public class EmergencyBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Claw claw = new Claw(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        DoubleArm armo = new DoubleArm(hardwareMap);


        Trajectory Forward = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1.3, 0.25, Rotation2d.fromDegrees(40))),
                new TrajectoryConfig(0.5, 0.2));

        Trajectory reverse = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1.25, 0.25, Rotation2d.fromDegrees(60))),
                new TrajectoryConfig(0.5, 0.2));


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new MoveDoubleArm(armo, -30,150),
                new RamseteCommand(chassis, Forward),
                new RamseteCommand(chassis, reverse)



        );

        waitForStart();
        chassis.resetPose(new Pose2d(0,0, new Rotation2d(0)));
        CommandScheduler.getInstance().schedule(testCommandGroup);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            Pose2d pose = chassis.getPose();

            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());
            telemetry.update();
        }
    }
}
