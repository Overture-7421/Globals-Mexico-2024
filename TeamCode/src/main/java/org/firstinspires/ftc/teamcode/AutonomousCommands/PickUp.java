package org.firstinspires.ftc.teamcode.AutonomousCommands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveArmo;
import org.firstinspires.ftc.teamcode.Commands.MoveFinger;
import org.firstinspires.ftc.teamcode.Subsystems.Armo;
import org.firstinspires.ftc.teamcode.Subsystems.Finger;


public class PickUp extends SequentialCommandGroup {

    public PickUp(Armo armo, Finger finger) {
        addCommands(
                new MoveFinger(finger,25),
                new MoveArmo(armo,0.005),
                new WaitCommand(2000),
                new MoveFinger(finger, 0),
                new MoveArmo(armo, 0.03)



                );
    }
}