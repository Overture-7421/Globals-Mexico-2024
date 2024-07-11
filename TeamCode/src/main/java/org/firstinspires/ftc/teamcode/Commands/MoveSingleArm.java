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
/*
Java funciona principalmente a base de Clases; estas clases a parte de ser un archivo
dentro del programa del robot, es una estructura de Programación Orientada a Objetos
donde podemos declarar, en este caso, motores, sensores, variables y constantes que
ayuden a la correcta ejecución del prorgama. A continuación se muestra la correcta
forma de iniciar una clase de nombre "MoveSingleArm"; debemos de agregar "extends CommandBase"
puesto que es un comando ya que nos permiten añadir las opciones, funciones y herramientas
de los comandos.
*/

public class MoveSingleArm extends CommandBase {


    private SingleArm singleArm;
    private double targetPosition;


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


