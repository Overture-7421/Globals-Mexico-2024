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

import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Commands.MoveDoubleArm;
import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;


@Autonomous
public class AutonomousMedium extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Claw claw = new Claw(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        DoubleArm armo = new DoubleArm(hardwareMap);


        Trajectory Forward = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(2.5, 0, Rotation2d.fromDegrees(0))),
                new TrajectoryConfig(0.5, 0.2));

        Trajectory Park = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(2.5, 0, Rotation2d.fromDegrees(90)),
                        new Pose2d(2.5, 1.2, Rotation2d.fromDegrees(90))),
                new TrajectoryConfig(0.5, 0.2));


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new MoveClaw(claw, 0.9),
                new RamseteCommand(chassis, Forward),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(95)),
                new MoveDoubleArm(armo, 10,20), //To be determined
                new RamseteCommand(chassis, Park),
                new MoveDoubleArm(armo, -10,10),
                new WaitCommand(1000),
                new MoveClaw(claw, 0.5)

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
