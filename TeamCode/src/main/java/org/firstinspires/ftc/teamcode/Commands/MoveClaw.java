

package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import java.util.concurrent.TimeUnit;

public class MoveClaw extends CommandBase {

 private Claw claw;
 private double clawPosition;

   public MoveClaw(Claw subsystem, double clawPosition) {
     this.claw = subsystem;
     this.clawPosition = clawPosition;

      addRequirements(claw);

    }

    @Override
    public void initialize() {
        claw.setPosition(clawPosition);
    }


    @Override
    public boolean isFinished() {
        /* Para terminar tenemos que usar el método "isFinished", siendo este el opuesto de initialize, nos indica
        que es lo que va a ocurrir una vez el comando haya sido finalizado o haya concluido con el movimiento.
         */
        double leftPosition = claw.getLeftPosition();
        double rightPosition = claw.getRightPosition();
        return (Math.abs(clawPosition - leftPosition) < 0.01) && (Math.abs(clawPosition - rightPosition) < 0.01);
    }
}

