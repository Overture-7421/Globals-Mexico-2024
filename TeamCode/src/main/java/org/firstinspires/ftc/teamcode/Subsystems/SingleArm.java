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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Controllers.FRCProfiledPIDController;
import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;

/*
Java funciona principalmente a base de Clases; estas clases a parte de ser un archivo
dentro del programa del robot, es una estructura de Programación Orientada a Objetos
donde podemos declarar, en este caso, motores, sensores, variables y constantes que
ayuden a la correcta ejecución del prorgama. A continuación se muestra la correcta
forma de iniciar una clase de nombre "SingleArm"; debemos de agregar "extends SubsystemBase"
puesto que es un subsystema ya que nos permiten añadir las opciones, funciones y herramientas
de los subsistemas.
*/

public class SingleArm extends SubsystemBase {
    /* -- MOTOR DECLARATION -- */

    /*
    Un controlador PID (Proporcional-Integral-Derivativo) es una técnica utilizada para controlar
    la velocidad y posición de un motor DC de manera precisa y estable. La lógica del PID se basa
    en tres componentes principales:

    1. Proporcional (P): Esta parte del controlador produce una salida que es proporcional al error
    actual (la diferencia entre el valor deseado y el valor actual). Si el error es grande, la salida
    proporcional será grande, lo que permite una corrección rápida.

    2. Integral (I): La parte integral considera la acumulación de errores pasados para eliminar errores
    residuales que pueden no ser corregidos por la acción proporcional sola. Ayuda a reducir el error
    a cero con el tiempo.

    3. Derivativo (D): La parte derivativa anticipa errores futuros basándose en la tasa de cambio
    del error. Proporciona una acción de amortiguación, reduciendo la sobreoscilación y mejorando
    la estabilidad del sistema.

    En el contexto de un motor DC, el controlador PID ajusta la potencia enviada al motor para alcanzar
    y mantener una velocidad o posición deseada. La salida del PID (la señal de control) modula la
    velocidad del motor de manera que minimiza el error entre la velocidad/posición deseada y la real.
    Esto resulta en un control preciso y suave del movimiento del motor.
    */

    /* -- PID DECLARATION -- */
    private FRCProfiledPIDController armPID;

    /*
    Tenemos que tener constantes de encoder relacionadas con los motores a los que les asignaremos un
    control por PID. Estas constantes tienen que ver con el funcionamiento fisico del mecanismo o
    operación del motor dentro del mismo.
    */

    /* -- MOTOR ENCODER CONSTANTS -- */
    public static final double COUNTS_PER_REV = 288;
    public static final double MOTOR_GEAR_RATIO = 1;

    /*
    Recordemos que tenemos que tener un offset para el motor para que el PID siempre sepa donde esta
    ubicado después de un reinicio. Pensemos que es el error que le damos al codigo para corregirse al
    motor y el encoder.
    */

    /* -- MOTOR OFFSET -- */
    private double motorOffset = -176;

    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */
    public SingleArm(HardwareMap hardwareMap) {
        /* -- MOTOR ID -- */

        /* -- PID CONSTRUCTOR -- */
        armPID = new FRCProfiledPIDController(0.0, 0.0, 0.0, new FRCTrapezoidProfile.Constraints(0, 0));
        /*
        Este constructor de PID es la línea del programa encargada de asignar valores tanto a la proporcional,
        integral y derivada. En la mayor parte de los casos sólo se mueve la proporcional ya que es la encargada de
        llegar al objetivo deseado. Recuerden que el PID siempre se debe calibrar empezando en 0.0.
        */

        /* -- SET MOTOR MODE AND ZERO POWER BEHAVIOR -- */
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /* Esta configuración nos indica que el motor
                                                                está configurado para alcanzar el nivel de velocidad
                                                                indicado sin la necesidad de un encoder.*/

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /* Esta configuración nos indica que cuando
                                                                        el motor no reciba ningún tipo de poder
                                                                        va a cambiar su comportamiento a frenarse
                                                                        resistiendo cualquier tipo de fuerza externa.*/

        /* -- ARMPID RES -- */
        armPID.reset(getPosition()); //Indica que el PID reinicia el comando "getPosition"
        armPID.setGoal(getPosition());//Indica que el PID le da un objetivo al comando "getPosition"
    }

    /* -- RESET ZERO FUNCTION -- */
    public void resetZero() {
        motorOffset = motor.getCurrentPosition();
    }

    /* -- GET POSITION FUNCTION -- */
    public double getPosition() {
        double currentTicks = motor.getCurrentPosition();
        double currentPosition = (currentTicks / COUNTS_PER_REV * MOTOR_GEAR_RATIO)  - (motorOffset/360);
        return currentPosition;
    }

    /* SET TARGET FUNCTION*/
    public void setTarget(double targetHeight) {
        if (armPID.getGoal().position != targetHeight) {
            armPID.reset(getPosition());
            armPID.setGoal(targetHeight);
        }
    }

    /* -- MOTOR OUTPUT AND PERIODIC FUNCTION -- */
    @Override
    public void periodic() {
        double motorOutput = armPID.calculate(getPosition());
        motor.setPower(motorOutput);

    }
}