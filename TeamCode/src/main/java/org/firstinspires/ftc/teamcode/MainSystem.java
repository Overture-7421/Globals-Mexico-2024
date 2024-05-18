/*
    __  __                  __                     ______          __
   / / / /___ ___  ______ _/ /_  __  ___________ _/ ____/___  ____/ /__
  / /_/ / __ `/ / / / __ `/ __ \/ / / / ___/ __ `/ /   / __ \/ __  / _ \
 / __  / /_/ / /_/ / /_/ / /_/ / /_/ (__  ) /_/ / /___/ /_/ / /_/ /  __/
/_/ /_/\__,_/\__, /\__,_/_.___/\__,_/____/\__,_/\____/\____/\__,_/\___/
            /____/

This is the code to control team Overture 23619's robot "Hayabusa".
Future iterations may change the overall functionality, though it is all used for the 2024 CENTERSTAGE FTC competition by FIRST.
All rights reserved. Copyright Overture 23619. Overture holds the right to modify and distribute this code.
*/

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Commands Import
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;


// Subsystems Import

import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;


@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis     = new Chassis(hardwareMap);     // Create an instance of Chassis
        Arm arm            = new Arm(hardwareMap);         // Create an instance of Arm
        Armo armo          = new Armo(hardwareMap);         // Create an instance of Armo
        Finger finger      = new Finger(hardwareMap);      // Create an instance of Finger
        GamepadEx driverOp  = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad
        GamepadEx toolOp    = new GamepadEx(gamepad2);      // Create an instance of OperatorGamepad

        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));



        Button operatorButtonY= toolOp.getGamepadButton(GamepadKeys.Button.Y);
        operatorButtonY.whenHeld(new MoveArm(arm, 0.5));
        operatorButtonY.whenReleased(new MoveArm(arm, 0));

        Button operatorButtonX= toolOp.getGamepadButton(GamepadKeys.Button.X);
        operatorButtonX.whenHeld(new MoveArm(arm, -0.5));
        operatorButtonX.whenReleased(new MoveArm(arm, 0));

        Button operatorButtonA= toolOp.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenHeld(new MoveFinger(finger, 1));
        operatorButtonA.whenReleased(new MoveFinger (finger, 0));

        Button operatorDpadUP= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        operatorDpadUP.whenPressed(new MoveArmo(armo, 0.3));

        Button operatorDpadRIGHT= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        operatorDpadRIGHT.whenPressed(new MoveArmo(armo, 0.2));

        Button operatorDpadDOWN= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        operatorDpadDOWN.whenPressed(new MoveArmo(armo, 0.1));

        waitForStart();
        chassis.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();

            // -- ODOMETRY TELEMETRY -- //
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());

            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());


            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}