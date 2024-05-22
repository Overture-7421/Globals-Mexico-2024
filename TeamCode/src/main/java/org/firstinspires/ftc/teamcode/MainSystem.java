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

// Other Imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.MoveClaw;

// Subsystems Import
import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
@TeleOp
public class MainSystem extends LinearOpMode {


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

    //    Chassis chassis     = new Chassis(hardwareMap);     // Create an instance of Chassis
        Armo armo           = new Armo(hardwareMap);         // Create an instance of Armo
    /*    Claw claw           = new Claw(hardwareMap);         // Create an instance of Claw
        Finger finger       = new Finger(hardwareMap);       // Create an instance of Finger
        Intake intake       = new Intake (hardwareMap);      // Create an instance of Intake */
        GamepadEx driverOp  = new GamepadEx(gamepad1);      // Create an instance of DriverGamepad
        GamepadEx toolOp    = new GamepadEx(gamepad2);      // Create an instance of OperatorGamepad



        // -- CHASSIS MOVEMENT -- //
    //    chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

        // -- Intake MOVEMENT -- //
    /*    Button driverRightBumper = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverRightBumper.whileHeld(new MoveIntake(intake,1));

        Button driverLeftBumper = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        driverLeftBumper.whileHeld(new MoveIntake(intake,-1));
    */
  /*-----------------------------------------------------------------------------------------*/

        // -- ARM MOVEMENT -- //

        Button operatorDpadRIGHT= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        operatorDpadRIGHT.whenPressed(new MoveArmo(armo, 0.07));

        Button operatorDpadDOWN= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        operatorDpadDOWN.whenPressed(new MoveArmo(armo, 0.03));

        Button operatorDpadUP= toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        operatorDpadUP.whenPressed(new MoveArmo(armo, 0.005));


        // -- Claw MOVEMENT -- //
    /*    Button operatorRightBumper= toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        operatorRightBumper.whenPressed(new MoveClaw(claw,-1));

        Button operatorLeftBumper= toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        operatorLeftBumper.whenPressed(new MoveClaw(claw,0.56));
    */

        // -- FINGER MOVEMENT -- //
    /*
        Button operatorButtonA= toolOp.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new MoveFinger(finger, 0.25));

        Button operatorButtonB= toolOp.getGamepadButton(GamepadKeys.Button.B);
        operatorButtonB.whenPressed(new MoveFinger(finger, 0));
    */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        //chassis.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()) {
            //timer.start();

            CommandScheduler.getInstance().run();
        //    Pose2d pose = chassis.getPose();

            // -- ODOMETRY TELEMETRY -- //
        /*    telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());

            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());
        */
            //Esto ni idea
            telemetry.addData("ArmiHeight", armo.ArmigetCurrentHeight());

            //No funciona verdaderamente
            telemetry.addData("Armi Target height", armo.goalHeight);
            telemetry.addData("ArmiCurrentPos", armo.ArmigetCurrentHeight);




            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}