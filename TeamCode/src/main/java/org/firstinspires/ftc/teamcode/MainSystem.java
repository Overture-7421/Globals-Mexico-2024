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

import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;


@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis         = new Chassis(hardwareMap);     // Create an instance of Chassis
        DoubleArm armo          = new DoubleArm(hardwareMap);         // Create an instance of Armo
        //ForeArm foreArm       = new ForeArm(hardwareMap);
        //Shooter Shoot_Servo   = new Shooter(hardwareMap);
        Claw claw               = new Claw(hardwareMap);
        GamepadEx driverOp      = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad
        //GamepadEx toolOp      = new GamepadEx(gamepad2);      // Create an instance of OperatorGamepad




        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

        /*Button driverRightBumper= driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverRightBumper.whenPressed(new MoveDoubleArm(armo, 150, 150));
*/
  // -----------------------------------------------------------------------------------------

        // -- ARM MOVEMENT (with PID)-- //

        Button driverDpadRIGHT= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        driverDpadRIGHT.whenPressed(new MoveDoubleArm(armo, 2, 0)); //Close

        Button driverDpadDOWN= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        driverDpadDOWN.whenPressed(new MoveDoubleArm(armo, -40, 88)); //Open */

        Button driverDpadUP= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        driverDpadUP.whenPressed(new MoveDoubleArm(armo,  35, 55)); //Up

        Button driverDpadLeft= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        driverDpadLeft.whenPressed(new MoveDoubleArm(armo, -40, 175)); //Rest position

        Button driverLeftBumper= driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        driverLeftBumper.whenPressed(new MoveDoubleArm(armo, 2, -55));



           // -- FINGER MOVEMENT -- //

        Button driverButtonA= driverOp.getGamepadButton(GamepadKeys.Button.A);
        driverButtonA.whenPressed(new GoDown(armo, claw)); //Ready

        Button driverButtonB= driverOp.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenPressed(new MoveClaw(claw, 0.5)); //OpenClaw

        Button driverButtonY= driverOp.getGamepadButton(GamepadKeys.Button.Y);
        driverButtonY.whenPressed(new MoveClaw(claw, 0.1)); //Close, and rest position

        /*Button driverButtonX= driverOp.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenPressed(new MoveClaw(claw, 0));*/


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