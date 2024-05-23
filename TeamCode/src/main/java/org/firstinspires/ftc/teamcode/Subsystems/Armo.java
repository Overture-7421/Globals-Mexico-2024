package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FRCProfiledPIDController;
import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;

public class Armo extends SubsystemBase {

    private DcMotorEx ArmoMotor1;
    private DcMotorEx ArmoMotor2;

    private FRCProfiledPIDController ArmoMotor1PID;

    public static final double COUNTS_PER_REV = 288;

    public static final double Arm_Gear_Ratio = 0.10868277;

    private int leftMotorOffset = 0;
    private int rightMotorOffset = 0;

    public Armo(HardwareMap hardwareMap) {
        ArmoMotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "ArmoMotor1");
        ArmoMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "ArmoMotor2");

        ArmoMotor1PID = new FRCProfiledPIDController(120, 0, 0.0, new FRCTrapezoidProfile.Constraints(2, 3));

        ArmoMotor1.setDirection(DcMotorEx.Direction.REVERSE);

        ArmoMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmoMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmoMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmoMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetZero();

        ArmoMotor1PID.reset(getHeight());
        ArmoMotor1PID.setGoal(getHeight());
    }

    public void resetZero() {
        rightMotorOffset = ArmoMotor2.getCurrentPosition();
        leftMotorOffset = ArmoMotor1.getCurrentPosition();
    }
    public double ArmoMotor1getCurrentHeight() {
        double ArmoMotor1Ticks = ArmoMotor1.getCurrentPosition() - leftMotorOffset;
        double ArmoMotor1getCurrentHeight = ArmoMotor1Ticks / COUNTS_PER_REV * Arm_Gear_Ratio;

        return ArmoMotor1getCurrentHeight;
    }

    public double ArmoMotor2getCurrentHeight() {
        double ArmoMotor2Ticks = ArmoMotor2.getCurrentPosition() - rightMotorOffset;
        double ArmoMotor2CurrentHeight = ArmoMotor2Ticks / COUNTS_PER_REV * Arm_Gear_Ratio;

        return ArmoMotor2CurrentHeight;
    }

    public double getHeight() {
        double ArmoMotor1Height = ArmoMotor1getCurrentHeight();
        double ArmoMotor2Height = ArmoMotor2getCurrentHeight();

        return (ArmoMotor1Height + ArmoMotor2Height) / 2.0;
    }
    public void setGoal(double goalHeight) {
        if (ArmoMotor1PID.getGoal().position != goalHeight) {
            ArmoMotor1PID.reset(getHeight());
            ArmoMotor1PID.setGoal(goalHeight);
        }
    }
    @Override
    public void periodic() {
        double outputMotor1 = ArmoMotor1PID.calculate(getHeight());
        ArmoMotor1.setPower(outputMotor1);
        ArmoMotor2.setPower(outputMotor1);

    }
}

