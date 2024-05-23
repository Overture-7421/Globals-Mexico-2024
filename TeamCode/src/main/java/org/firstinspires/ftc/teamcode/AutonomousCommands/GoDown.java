package org.firstinspires.ftc.teamcode.AutonomousCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;

public class GoDown extends ParallelRaceGroup {

    public GoDown(Armo armo, Finger finger) {
        addCommands(
                new MoveArmo(armo, 0.005),
                new MoveFinger(finger, 0.30)
        );
    }
}