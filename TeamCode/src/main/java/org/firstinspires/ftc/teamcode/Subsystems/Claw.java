/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/
package org.firstinspires.ftc.teamcode.Subsystems;


/*
Es importante que para que tu subsistema funcione importes todas las librerías necesarias
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

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
Java funciona principalmente a base de Clases; estas clases a parte de ser un archivo
dentro del programa del robot, es una estructura de Programación Orientada a Objetos
donde podemos declarar, en este caso, motores, sensores, variables y constantes que
ayuden a la correcta ejecución del prorgama. A continuación se muestra la correcta
forma de iniciar una clase de nombre "Claw"; debemos de agregar "extends SubsystemBase"
puesto que es un subsystema ya que nos permiten añadir las opciones, funciones y herramientas
de los subsistemas.
*/

public class Claw extends SubsystemBase {
    /*
    El HardwareMap, como su nombre lo dice, es una forma sencilla de ubicar el Hardware del robot en
    sus conexiónes al ControlHub, usualmente, los motores funcionan por medio de IDs, pero elementos como
    el IMU operan internamente aunque se tiene que "declarar" su ubicación dentro el mismo HardwareMap.
    */

    /* -- MOTOR DECLARATION -- */
   // private ServoEx rightServo;


    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */

    public Claw (HardwareMap hardwareMap) {
        /* -- SERVO IDs --*/
       // rightServo = new SimpleServo(hardwareMap, "grab_RightServo", 0, 180);

        /* -- MOTOR DIRECTION -- */
        //leftServo.setInverted(true);
    }

    /* -- GET RIGHT POSITION -- */
    /* -- GET LEFT POSITION -- */
    /* -- SET POSITION -- */

}