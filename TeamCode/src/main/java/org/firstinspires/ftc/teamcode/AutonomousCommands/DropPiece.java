package org.firstinspires.ftc.teamcode.AutonomousCommands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;


public class DropPiece extends SequentialCommandGroup {

    public DropPiece(Armo armo, Finger finger) {
        addCommands(
                new MoveArmo(armo, 0.07),
                new WaitCommand(1000),
                new MoveFinger(finger, 0.25),
                new WaitCommand(1000),
                new MoveArmo(armo, 0.03)

        );
    }
}