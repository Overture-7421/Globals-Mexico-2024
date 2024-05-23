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
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Commands.RamseteCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;


@Autonomous
public class L extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Finger finger = new Finger(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        Armo armo = new Armo(hardwareMap);


        Trajectory Forward = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(2.1, 0, Rotation2d.fromDegrees(0))),
                new TrajectoryConfig(1, 0.8));


        Trajectory Right = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(2.1, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(2.1, 2.2, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(1, 0.8));

        Trajectory FRight = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(2.1, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(2.1, 2.7, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(1, 0.8));

        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new MoveArmo(armo, 0.03),
                new MoveFinger(finger, 0),
                new RamseteCommand(chassis, Forward),
                new MoveArmo(armo, 0.007),
                new MoveFinger(finger, 0.15),
                new MoveArmo(armo, 0.03),
                new WaitCommand(1000),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-90)),
                new RamseteCommand(chassis, Right),
                new MoveArmo(armo, 0.07),
                new RamseteCommand(chassis, FRight),
                new MoveFinger(finger, 0)
        );

        waitForStart();

        chassis.resetPose(Forward.getInitialPose());

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

