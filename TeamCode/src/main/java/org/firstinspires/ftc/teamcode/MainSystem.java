package org.firstinspires.ftc.teamcode;/*

   ____                  __                     ________      __          __
  / __ \_   _____  _____/ /___  __________     / ____/ /___  / /_  ____ _/ /____
 / / / / | / / _ \/ ___/ __/ / / / ___/ _ \   / / __/ / __ \/ __ \/ __ `/ / ___/
/ /_/ /| |/ /  __/ /  / /_/ /_/ / /  /  __/  / /_/ / / /_/ / /_/ / /_/ / (__  )
\____/ |___/\___/_/   \__/\__,_/_/   \___/   \____/_/\____/_.___/\__,_/_/____/


This is the code to control team Overture 23619's robot "INSERT ROBOT NAME HERE".
Future iterations may change the overall functionality, though it will be all used for the 2024 FIRST GLOBALS competition.
All rights reserved. Copyright Overture 23619. Overture holds the right to modify and distribute this code.
*/

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;



// Commands Import

import org.firstinspires.ftc.teamcode.Commands.GrabRestPosition;
import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Commands.MoveDoubleArm;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.AutonomousCommands.GoDown;


// Subsystems Import

import org.firstinspires.ftc.teamcode.Commands.MoveShooter;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;


@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis         = new Chassis(hardwareMap);     // Create an instance of Chassis
        DoubleArm armo          = new DoubleArm(hardwareMap);         // Create an instance of Armo
        Shooter Shoot_Servo   = new Shooter(hardwareMap);
        Claw claw               = new Claw(hardwareMap);
        GamepadEx driverOp      = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad





        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

        Button driverRightBumper= driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverRightBumper.whileHeld(new MoveDoubleArm(armo, -70,150));


  // -----------------------------------------------------------------------------------------

        // -- ARM MOVEMENT (with PID)-- //


        Button driverDpadDOWN= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        driverDpadDOWN.whenPressed(new MoveDoubleArm(armo, -70, 89.5)); //Down LISTOOOOOOOO

        Button driverDpadLeft= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        driverDpadLeft.whenPressed(new MoveDoubleArm(armo, -40, 63)); //UP

        Button driverLeftBumper= driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        driverLeftBumper.whenPressed(new MoveDoubleArm(armo, 90, 150)); //Cone

        Button driverDpadUP= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        driverDpadUP.whenPressed(new MoveDoubleArm(armo,  -40, 85)); //Climb

        Button driverDpadRIGHT= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        driverDpadRIGHT.whenPressed(new MoveDoubleArm(armo, -70,150)); //Close



           // -- FINGER MOVEMENT -- //

        Button driverButtonY= driverOp.getGamepadButton(GamepadKeys.Button.Y);
        driverButtonY.whenPressed(new MoveDoubleArm(armo, 90,85)); //Ready

        Button driverButtonX= driverOp.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenPressed(new MoveClaw(claw, 0.9)); //OpenClaw

        Button driverButtonB= driverOp.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenPressed(new MoveClaw(claw, 0.5));

        Button driverButtonA= driverOp.getGamepadButton(GamepadKeys.Button.A);
        driverButtonA.whenPressed(new MoveDoubleArm(armo, -30,75)); //Close, and rest position




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
            telemetry.addData("ServoLeft", claw.getLeftPosition());
            telemetry.addData("ServoRight", claw.getRightPosition());
            telemetry.addData("UpperPosition", armo.getUpperPosition() * 360);
            telemetry.addData("LowerPosition", armo.getLowerPosition() * 360);



            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}