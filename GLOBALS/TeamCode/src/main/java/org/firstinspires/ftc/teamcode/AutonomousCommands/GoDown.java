package org.firstinspires.ftc.teamcode.AutonomousCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Commands.MoveDoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class GoDown extends ParallelRaceGroup {

    public GoDown(DoubleArm armo,  Claw claw) {
        addCommands(
                new MoveDoubleArm(armo, -40, 40),
                new MoveClaw(claw, 0.20)
        );
    }
}