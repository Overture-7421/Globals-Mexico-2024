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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Utils.JoystickHandler;


/*
Java funciona principalmente a base de Clases; estas clases a parte de ser un archivo
dentro del programa del robot, es una estructura de Programación Orientada a Objetos
donde podemos declarar, en este caso, motores, sensores, variables y constantes que
ayuden a la correcta ejecución del prorgama. A continuación se muestra la correcta
forma de iniciar una clase de nombre "MoveChassis"; debemos de agregar "extends CommandBase"
puesto que es un comando ya que nos permiten añadir las opciones, funciones y herramientas
de los comandos.
*/

public class MoveChassis extends CommandBase {

          /*El siguiente paso para la realización de los comandos, es declarar las variables y las
    constantes necesarias para la realización del comando. Estas ya que sólo serán usadas en el comando
    suelen ser "private final" es decir que se usa exclusivamente en esta clase.

    Ej. private final Chassis chassis;
     */

    /* Declaración de constantes o variables*/

    /* Posteriormente se tiene que crear el "Constructor" del comando. Un constructor
     inicializa y configura un objeto de comando con parámetros específicos para controlar el comportamiento
     del robot. Es importante recordar que un comando puede que ser llamado desde un OpMode o un Autónomo, es
     por eso que debe ser "public", esto lo hace accesible desde otras partes dentro del código.

     Ej. public MoveShooter(Shooter subsystem, double Voltage) {
     */

        /* Junto a este paso también es relevante especificarle al sistema que no confunda los objetos con los
    requerimientos del constructor, es por eso que agregamos "this." a las constantes, para especificar que
    lo que queremos afectar es al objeto.

    Ej. this.shooter = subsystem
        this.voltage = voltage
     */

    /* Al final tenemos que agregar "addRequirements" y al subsistema que queremos que se vea afectado para
    que los cambios se vean afectados.

    Ej. addRequirements(Shooter);
     */

    @Override
    public void execute(){
        /* Esta última indicación "execute" nos indica precisamente como se va a ejecutar el comando
        además de que nos da unas últimas explicaciones sobre como va a ser el output (controlado) del mismo.
         */
        double right = -driverGamepad.right_stick_x;
        double left = -driverGamepad.left_stick_y;

        right = JoystickHandler.handleJoystickInput(right);
        left = JoystickHandler.handleJoystickInput(left);


        chassis.setSpeed(left, right);
    }
}