package org.firstinspires.ftc.teamcode.AutonomousCommands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.MoveClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;


public class DropPiece extends SequentialCommandGroup {

    public DropPiece(DoubleArm armo, Claw claw) {
        addCommands(
                new MoveClaw(claw, 0),
                //new MoveDoubleArm(armo, 0.16, 0),
                new WaitCommand(1000),
                new MoveClaw(claw, 0.20),
                new WaitCommand(1000)
                //new MoveDoubleArm(armo, 0.03,0)

        );
    }
}