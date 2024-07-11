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

public class SingleArm extends SubsystemBase {

    private DcMotorEx rightArm;
    private DcMotorEx leftArm;
    /* -- MOTOR DECLARATION -- */

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

    /* -- MOTOR OFFSET -- */
    private double rightMotorOffset = -176;
    private double leftMotorOffset = -176;

    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */
    public SingleArm(HardwareMap hardwareMap) {
        rightArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightArm");
        leftArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftArm");

           /* -- MOTOR ID -- */

        /* -- PID CONSTRUCTOR -- */
        armPID = new FRCProfiledPIDController(0.0, 0.0, 0.0, new FRCTrapezoidProfile.Constraints(0, 0));
        /*
        Este constructor de PID es la línea del programa encargada de asignar valores tanto a la proporcional,
        integral y derivada. En la mayor parte de los casos sólo se mueve la proporcional ya que es la encargada de
        llegar al objetivo deseado. Recuerden que el PID siempre se debe calibrar empezando en 0.0.
        */

        /* -- SET MOTOR MODE AND ZERO POWER BEHAVIOR -- */
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /* Esta configuración nos indica que el motor
                                                              está configurado para alcanzar el nivel de velocidad
                                                               indicado sin la necesidad de un encoder.*/

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /* Esta configuración nos indica que cuando
                                                                        el motor no reciba ningún tipo de poder
                                                                        va a cambiar su comportamiento a frenarse
                                                                        resistiendo cualquier tipo de fuerza externa.*/


        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        /* -- ARMPID RES -- */
        armPID.reset(getPosition()); //Indica que el PID reinicia el comando "getPosition"
        armPID.setGoal(getPosition());//Indica que el PID le da un objetivo al comando "getPosition"
    }

    /* -- RESET ZERO FUNCTION -- */
    public void resetZero() {leftMotorOffset = leftArm.getCurrentPosition();
        rightMotorOffset = rightArm.getCurrentPosition();
    }


    /* -- GET POSITION FUNCTION -- */
    public double getPosition() {
        double currentTicks = leftArm.getCurrentPosition();
        double currentPosition = (currentTicks / COUNTS_PER_REV * MOTOR_GEAR_RATIO)  - (leftMotorOffset/360);
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
        double limitOutput = armPID.calculate(getPosition());
        leftArm.setPower(limitOutput);
        rightArm.setPower(limitOutput);

    }
}