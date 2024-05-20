package org.firstinspires.ftc.teamcode.AutonomousCommands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;

public class Salute extends SequentialCommandGroup {
    public Salute(Armo armo) {
        addCommands(
                new MoveArmo(armo, 0.03).withTimeout(1000),
                new MoveArmo(armo, 0.005)

        );
    }
}