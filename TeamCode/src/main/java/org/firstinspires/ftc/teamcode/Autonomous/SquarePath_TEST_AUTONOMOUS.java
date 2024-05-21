package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import java.util.Arrays;

@Autonomous
public class SquarePath_TEST_AUTONOMOUS extends LinearOpMode {

    Chassis chassis;


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Trajectory Longfwd = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.3528, 0, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(1, 0.8));

        Trajectory ShortFwd = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0,0, Rotation2d.fromDegrees(0)),
                        new Pose2d(0.0, 1.2192, Rotation2d.fromDegrees(-90))),
                new TrajectoryConfig(1, 0.8));



    }

}
