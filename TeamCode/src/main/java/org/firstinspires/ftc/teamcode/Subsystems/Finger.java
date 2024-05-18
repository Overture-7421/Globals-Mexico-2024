package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Finger extends SubsystemBase {
    private ServoEx grab_Finger;

    public Finger(HardwareMap hardwareMap) {
     grab_Finger = new SimpleServo(hardwareMap, "grab_Finger", 0,1);
    }

    public void FingerPosition(double FingerMotorPosition) {
        grab_Finger.setPosition(FingerMotorPosition);
    }
}
