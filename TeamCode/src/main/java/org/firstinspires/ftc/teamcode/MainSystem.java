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
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Commands Import

import org.firstinspires.ftc.teamcode.Commands.MoveChassis;


// Subsystems Import

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;


@Config
@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis     = new Chassis(hardwareMap);     // Create an instance of Chassis
        //Armo armo          = new Armo(hardwareMap);         // Create an instance of Armo
        //Shooter Shoot_Servo = new Shooter(hardwareMap);
        //Claw claw          = new Claw(hardwareMap);
        //Finger finger      = new Finger(hardwareMap);       // Create an instance of Finger
        //Intake intake      = new Intake (hardwareMap);
        GamepadEx driverOp  = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad
        //GamepadEx toolOp    = new GamepadEx(gamepad2);      // Create an instance of OperatorGamepad




        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

     /*   // -- Intake MOVEMENT -- //
        Button driverRightBumper = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverRightBumper.whileHeld(new MoveIntake(intake,1));

        Button driverLeftBumper = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        driverLeftBumper.whileHeld(new MoveIntake(intake,-1)); */

  // -----------------------------------------------------------------------------------------

        // -- ARM MOVEMENT (with PID)-- //
/*
        Button driverDpadRight= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        driverDpadRight.whenPressed(new MoveArmo(armo, 0.16));

        Button driverDpadDOWN= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        driverDpadDOWN.whenPressed(new MoveArmo(armo, 0.04));

        Button driverDpadLeft= driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        driverDpadLeft.whenPressed(new MoveArmo(armo, 0.007));

           // -- FINGER MOVEMENT -- //

        Button driverButtonY= driverOp.getGamepadButton(GamepadKeys.Button.Y);
        driverButtonY.whenPressed(new GoDown(armo, finger));

        Button driverButtonB= driverOp.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenPressed(new DropPiece(armo, finger));

        Button driverButtonX= driverOp.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenPressed(new RestPosition(armo, finger));

        Button driverButtonA= driverOp.getGamepadButton(GamepadKeys.Button.A);
        driverButtonA.whenPressed(new MoveFinger(finger, 0.30));


*/

        //Button driverButtonY= toolOp.getGamepadButton(GamepadKeys.Button.Y);
        //driverButtonY.whenPressed(new MoveShooter(Shoot_Servo, 1));


/*
        // -- Claw MOVEMENT -- //
        Button driverRightBumper= toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        operatorRightBumper.whenPressed(new MoveClaw(claw,-1));

        Button driverLeftBumper= toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        operatorLeftBumper.whenPressed(new MoveClaw(claw,0.56));

*/      FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


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


/*
            telemetry.addData("Armi1Height", armo.ArmoMotor1getCurrentHeight());
            telemetry.addData("Armi2Height", armo.ArmoMotor2getCurrentHeight());


*/
            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}