package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    // Motor Declaration
    private ServoEx shooter;

    public Shooter (HardwareMap hardwareMap){
        shooter = new SimpleServo(hardwareMap, "Shoot_Servo", 0, 1);

        //shooter.setInverted(true);
    }

    public double getPosition(){
        return shooter.getPosition();
    }

    public void setPosition(double shooterPosition) {
        shooter.setPosition(shooterPosition);
    }

}