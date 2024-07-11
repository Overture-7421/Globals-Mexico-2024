/*

BIENVENIDOS AL CODIGO DEL ROBOT.

 */
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.SingleArm;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;



// Commands Import


import org.firstinspires.ftc.teamcode.Commands.MoveClaw;



// Subsystems Import


import org.firstinspires.ftc.teamcode.Commands.MoveSingleArm;



@TeleOp
public class MainSystem extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        /* --------------------- SUBSYSTEM INSTANCES --------------------- */

                Chassis chassis = new Chassis(hardwareMap);
                Claw claw = new Claw(hardwareMap);
                SingleArm singleArm = new SingleArm(hardwareMap);
                GamepadEx DriverOp = new GamepadEx(gamepad1);



        /* --------------------- BUTTONBINDINGS --------------------- */
                                

      chassis.setDefaultCommand(new MoveChassis(chassis,gamepad1));

      Button driverButtonX = DriverOp.getGamepadButton(GamepadKeys.Button.X);
      driverButtonX.whenPressed(new MoveClaw(claw, 0.9));
        Button driverButtonB = DriverOp.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenPressed(new MoveClaw(claw, 0.5));
        Button driverDpadUP= DriverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        driverDpadUP.whenPressed(new MoveSingleArm(singleArm, 10));
        Button driverDpadRIGHT= DriverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        driverDpadRIGHT.whenPressed(new MoveSingleArm(singleArm, 0));
        Button driverDpadDOWN= DriverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        driverDpadDOWN.whenPressed(new MoveSingleArm(singleArm, -10));




        // -- CHASSIS MOVEMENT -- //
        // -- ARM MOVEMENT (WITH PID) -- //
        // -- ARM MOVEMENT -- //
        // -- FINGER MOVEMENT -- //

        /* --------------------- OPMODE EXECUTION --------------------- */
        waitForStart();
        


        chassis.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(0)));


        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());
            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());
            telemetry.addData("Position", singleArm.getPosition());
            telemetry.update();
            /*Por último, esto nos permite hacer que la telemtry se este actualizando
            constantemente, en tiempo real*/
        }
    }
}
