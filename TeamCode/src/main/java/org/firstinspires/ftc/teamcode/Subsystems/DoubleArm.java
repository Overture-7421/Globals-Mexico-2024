package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FRCProfiledPIDController;
import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;

public class DoubleArm extends SubsystemBase {

    private DcMotorEx lowerMotor;

    private DcMotorEx lowerMotorTwo;
    private DcMotorEx upperMotor;

    private FRCProfiledPIDController lowerPID;
    private FRCProfiledPIDController upperPID;

    public static final double COUNTS_PER_REV = 288;

    public static final double LOWER_GEAR_RATIO = 1.3333333333;
    public static final double UPPER_GEAR_RATIO = 1.3333333333;

    private double lowerOffset = 64;

    private double upperOffset = -168; //-253

    public DoubleArm(HardwareMap hardwareMap) {
        lowerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ArmoMotor1");
        lowerMotorTwo = (DcMotorEx) hardwareMap.get(DcMotor.class, "ArmoMotor2");
        upperMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ArmoMotor3");

        lowerPID = new FRCProfiledPIDController(12, 0, 0.0, new FRCTrapezoidProfile.Constraints(0.7, 0.5));
        upperPID = new FRCProfiledPIDController(5.5, 0, 0.0, new FRCTrapezoidProfile.Constraints(1, 0.7));

        lowerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        lowerMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        //upperMotor.setDirection(DcMotorEx.Direction.REVERSE);

        lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // resetZero();

        lowerPID.reset(getLowerPosition());
        lowerPID.setGoal(getLowerPosition());

        upperPID.reset(getUpperPosition());
        upperPID.setGoal(getUpperPosition());
    }

    public void resetZero() {
        lowerOffset = lowerMotor.getCurrentPosition();
        upperOffset = upperMotor.getCurrentPosition();
    }
    public double getLowerPosition() {
        double currentTicks = lowerMotor.getCurrentPosition();
        // Sin Enconder
         //double currentPosition = (currentTicks / COUNTS_PER_REV * LOWER_GEAR_RATIO) - (lowerOffset/360);
        // Con encoder
        double currentPosition = (currentTicks / 8192) - (lowerOffset / 360);
        return currentPosition;
    }

    public double getUpperPosition() {
        double currentTicks = upperMotor.getCurrentPosition();
        // Sin Enconder
         //double currentPosition = (currentTicks / COUNTS_PER_REV * UPPER_GEAR_RATIO) - (upperOffset/360);
        // Con encoder
        double currentPosition = (currentTicks / 8192) - (upperOffset / 360);
        return currentPosition;
    }

    public void setTargets(double lowerTarget, double upperTarget) {
        if (lowerPID.getGoal().position != lowerTarget) {
            lowerPID.reset(getLowerPosition());
            lowerPID.setGoal(lowerTarget);
        }

        if (upperPID.getGoal().position != upperTarget) {
            upperPID.reset(getUpperPosition());
            upperPID.setGoal(upperTarget);
        }
    }

    @Override
    public void periodic() {
         double lowerOutput = lowerPID.calculate(getLowerPosition());
         double upperOutput = upperPID.calculate(getUpperPosition());
         lowerMotor.setPower(lowerOutput);
         lowerMotorTwo.setPower(lowerOutput);
         upperMotor.setPower(upperOutput);
    }
}

