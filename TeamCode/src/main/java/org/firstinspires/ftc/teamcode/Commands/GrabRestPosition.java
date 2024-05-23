package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;


public class GrabRestPosition extends SequentialCommandGroup {

    public GrabRestPosition(Armo armo, Finger finger) {
        addCommands(
                new MoveFinger(finger, 0),
                new MoveArmo(armo, 0.04)

        );
    }
}