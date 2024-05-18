package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {

    private DcMotorEx Arm_Move;

    public Arm (HardwareMap hardwareMap) {
        Arm_Move = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "Arm_Move");
        Arm_Move.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //public void Voltage(double ArmMotorVoltage) { Arm_Move.setPower(ArmMotorVoltage);}
    public void Voltage(double ArmMotorVoltage) { Arm_Move.setPower(ArmMotorVoltage);}
}
