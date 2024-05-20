package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake extends SubsystemBase {

    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void Voltage (double IntakeMotorSpeed){
        intake.setPower(IntakeMotorSpeed);
    }
}