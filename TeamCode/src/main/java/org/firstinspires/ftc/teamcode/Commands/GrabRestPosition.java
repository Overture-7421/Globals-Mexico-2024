package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;



public class GrabRestPosition extends SequentialCommandGroup {

    public GrabRestPosition(DoubleArm armo, Claw claw) {
        addCommands(
                new MoveClaw(claw, 0),
                new WaitCommand(1000),
                new MoveDoubleArm(armo, -77, 150)

        );
    }
}