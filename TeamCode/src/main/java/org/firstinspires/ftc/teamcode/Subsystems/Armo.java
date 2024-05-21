package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FRCProfiledPIDController;
import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;

public class Armo extends SubsystemBase {

    private DcMotorEx Armi;

    private FRCProfiledPIDController ArmoPID;

    public static final double COUNTS_PER_REV = 288;

    public static final double Arm_Gear_Ratio = 0.10868277;

    public static double ArmiHeight;
    public static double goalHeight;

    private int ArmoMotorOffset = 0;

    public Armo(HardwareMap hardwareMap) {
        Armi = hardwareMap.get(DcMotorEx.class, "Armi");

        ArmoPID = new FRCProfiledPIDController(30, 0, 0.0, new FRCTrapezoidProfile.Constraints(2, 1));

        Armi.setDirection(DcMotorEx.Direction.REVERSE);

        Armi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Armi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetZero();

        ArmoPID.reset(getHeight());
        ArmoPID.setGoal(getHeight());


    }

    public void resetZero() {
        ArmoMotorOffset = Armi.getCurrentPosition();
    }
    public double ArmigetCurrentHeight() {
        double ArmiTicks = Armi.getCurrentPosition() - ArmoMotorOffset;
        double ArmigetCurrentHeight = ArmiTicks / COUNTS_PER_REV * Arm_Gear_Ratio;

        return ArmigetCurrentHeight;
    }

    public double getHeight() {
        double ArmiHeight = ArmigetCurrentHeight();

        return (ArmiHeight) / 2.0;
    }
    public void setGoal(double goalHeight) {
        if(ArmoPID.getGoal().position != goalHeight) {
            ArmoPID.setGoal(goalHeight);
        }
    }
    @Override
    public void periodic() {
        double outputMotor1 = ArmoPID.calculate(getHeight());
        Armi.setPower(outputMotor1);
    }
}

