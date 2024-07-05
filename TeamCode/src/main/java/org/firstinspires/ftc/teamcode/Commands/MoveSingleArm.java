/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/

package org.firstinspires.ftc.teamcode.Commands;

/*
Es importante que para que tu Comando funcione importes todas las librerías necesarias
que vayan a ayudar a la ejecución del subsistema o comando. Java y OnBotJava te irá
avisando cuando necesites importar algo, puesto que te indicará que falta una "librería".
En este primer ejemplo para el desarrollo del subsistema del chassis te daremos las
librerías necesarias para la ejecución. Nota como tiene una estructura:

import -> indica que vas a importar una librería.
com.arcrobotics.ftclib.command
                      .geometry -> indica la ubicación de la librería.
                      .kinematics
Finalmente damos el nombre del archivo especifico dentro de la librería.
*/


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.SingleArm;

public class MoveSingleArm extends CommandBase {

    private final SingleArm singleArm;

    private final double targetPosition;

    public MoveSingleArm(SingleArm subsystem, double targetPosition) {
        this.singleArm = subsystem;
        this.targetPosition = targetPosition / 360;
        addRequirements(singleArm);
    }

    @Override
    public void initialize() {
        singleArm.setTarget(targetPosition);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = singleArm.getPosition();
        return Math.abs(targetPosition - currentPosition) < 0.05;
    }
}
