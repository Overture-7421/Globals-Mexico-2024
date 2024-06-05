package org.firstinspires.ftc.teamcode.Autonomous;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.GrabRestPosition;
import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Commands.MoveDoubleArm;
import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;


@Autonomous
public class Forward extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Claw claw = new Claw(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        DoubleArm armo = new DoubleArm(hardwareMap);


        Trajectory Forward = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(0.67, 0, Rotation2d.fromDegrees(0))),
                new TrajectoryConfig(1, 0.5));

        Trajectory Left = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0.67, 0, Rotation2d.fromDegrees(90)),
                        new Pose2d(0.67, 1.9, Rotation2d.fromDegrees(90))),
                new TrajectoryConfig(1, 0.9));

        Trajectory Right = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0.67, 1.9, Rotation2d.fromDegrees(179)),
                        new Pose2d(0.1, 1.9, Rotation2d.fromDegrees(180))),
                new TrajectoryConfig(1, 0.9));

        Trajectory Front = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0.2, 1.9, Rotation2d.fromDegrees(0)),
                        new Pose2d(1.3, 1.9, Rotation2d.fromDegrees(-1))),
                new TrajectoryConfig(1, 0.9));

        Trajectory Back = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(1.3, 1.9, Rotation2d.fromDegrees(-90)),
                        new Pose2d(1.3, -0.25, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(1, 0.9));

        Trajectory Score = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(1.3, -0.25, Rotation2d.fromDegrees(89)),
                        new Pose2d(1.4, 2.10, Rotation2d.fromDegrees(90))),
                new TrajectoryConfig(2, 0.9));

        Trajectory testing = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(1.4, 2.15, Rotation2d.fromDegrees(-90)),
                        new Pose2d(1.4, 1.8, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(2, 0.9));

        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new MoveClaw(claw, 0),
                new RamseteCommand(chassis, Forward),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(90)),
                new RamseteCommand(chassis, Left),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(179)),
                new MoveDoubleArm(armo, 15, 10),
                new RamseteCommand(chassis, Right),
                new MoveClaw(claw, 0.25),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new MoveDoubleArm(armo, -77, 150),
                new RamseteCommand(chassis, Front),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-90)),
                new RamseteCommand(chassis, Back),
                new MoveDoubleArm(armo, -77, 75),
                new GrabRestPosition(armo, claw),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(90)),
                new RamseteCommand(chassis, Score),
                new MoveDoubleArm(armo, 10, 10),
                new MoveClaw(claw, 0.20),
                new RamseteCommand(chassis, testing)


        );

        waitForStart();
        chassis.resetPose(Forward.getInitialPose());
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
