/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/

package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class MoveClaw extends CommandBase {

    private Claw claw;
    private double clawPosition;

    public MoveClaw(Claw subsystem, double clawPosition) {
         this.clawPosition = clawPosition;
         claw = subsystem;
         addRequirements(subsystem);
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

