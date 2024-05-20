/*

   ____                  __                     ________      __          __
  / __ \_   _____  _____/ /___  __________     / ____/ /___  / /_  ____ _/ /____
 / / / / | / / _ \/ ___/ __/ / / / ___/ _ \   / / __/ / __ \/ __ \/ __ `/ / ___/
/ /_/ /| |/ /  __/ /  / /_/ /_/ / /  /  __/  / /_/ / / /_/ / /_/ / /_/ / (__  )
\____/ |___/\___/_/   \__/\__,_/_/   \___/   \____/_/\____/_.___/\__,_/_/____/


This is the code to control team Overture 23619's robot "INSERT ROBOT NAME HERE".
Future iterations may change the overall functionality, though it will be all used for the 2024 FIRST GLOBALS competition.
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
//import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.MoveClaw;


// Subsystems Import

import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
//import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis     = new Chassis(hardwareMap);     // Create an instance of Chassis
        //Arm arm            = new Arm(hardwareMap);          // Create an instance of Arm
        Armo armo          = new Armo(hardwareMap);         // Create an instance of Armo
        Claw claw          = new Claw(hardwareMap);
        Finger finger      = new Finger(hardwareMap);       // Create an instance of Finger
        Intake intake      = new Intake (hardwareMap);
        GamepadEx driverOp  = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad
        GamepadEx toolOp    = new GamepadEx(gamepad2);      // Create an instance of OperatorGamepad



        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

        // -- Intake MOVEMENT -- //
        Button driverRightBumper = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverRightBumper.whileHeld(new MoveIntake(intake,1));

        Button driverLeftBumper = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        driverLeftBumper.whileHeld(new MoveIntake(intake,-1));

  /*-----------------------------------------------------------------------------------------*/


        // -- ARM MOVEMENT (without PID)-- //
       /* Button operatorButtonY= toolOp.getGamepadButton(GamepadKeys.Button.Y);
        operatorButtonY.whenHeld(new MoveArm(arm, 0.5));
        operatorButtonY.whenReleased(new MoveArm(arm, 0.1));

        Button operatorButtonX= toolOp.getGamepadButton(GamepadKeys.Button.X);
        operatorButtonX.whenHeld(new MoveArm(arm, -0.5));
        operatorButtonX.whenReleased(new MoveArm(arm, -0.1));
*/

        // -- ARM MOVEMENT (with PID)-- //

        Button operatorDpadRIGHT= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        operatorDpadRIGHT.whenPressed(new MoveArmo(armo, 0.07));

        Button operatorDpadDOWN= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        operatorDpadDOWN.whenPressed(new MoveArmo(armo, 0.03));

        Button operatorDpadUP= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        operatorDpadUP.whenPressed(new MoveArmo(armo, 0.005));


        // -- Claw MOVEMENT -- //
        Button operatorRightBumper= toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        operatorRightBumper.whenPressed(new MoveClaw(claw,-1));

        Button operatorLeftBumper= toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        operatorLeftBumper.whenPressed(new MoveClaw(claw,0.56));


        // -- FINGER MOVEMENT -- //
        Button operatorButtonA= toolOp.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new MoveFinger(finger, 0.25));

        Button operatorButtonB= toolOp.getGamepadButton(GamepadKeys.Button.B);
        operatorButtonB.whenPressed(new MoveFinger(finger, 0));

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

            telemetry.addData("ArmiHeight", armo.ArmigetCurrentHeight());


            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}