package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
public class ClimbUp extends ParallelRaceGroup {

    public ClimbUp(DoubleArm armo, Claw claw) {
        addCommands(
                new MoveDoubleArm(armo, 0.09, 0),
                new MoveClaw(claw, 0.20)
        );
    }
}