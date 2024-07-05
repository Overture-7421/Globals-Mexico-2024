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
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
Java funciona principalmente a base de Clases; estas clases a parte de ser un archivo
dentro del programa del robot, es una estructura de Programación Orientada a Objetos
donde podemos declarar, en este caso, motores, sensores, variables y constantes que
ayuden a la correcta ejecución del prorgama. A continuación se muestra la correcta
forma de iniciar una clase de nombre "Chassis"; debemos de agregar "extends SubsystemBase"
puesto que es un subsystema ya que nos permiten añadir las opciones, funciones y herramientas
de los subsistemas.
*/

public class Chassis extends SubsystemBase {
    /*
    El HardwareMap, como su nombre lo dice, es una forma sencilla de ubicar el Hardware del robot en
    sus conexiónes al ControlHub, usualmente, los motores funcionan por medio de IDs, pero elementos como
    el IMU operan internamente aunque se tiene que "declarar" su ubicación dentro el mismo HardwareMap.
    */

    /* -- MOTOR DECLARATION --*/
    /* -- CHASSIS CONSTANTS --*/
    /* -- DIFFERENTAL DRIVE ODOMETRY DECLARATION*/
        /*
        DDO es una clase para odometría de accionamiento diferencial. La odometría te permite
        rastrear la posición del robot en el campo durante el transcurso de un partido utilizando
        lecturas de 2 encoders y un giroscopio. Los equipos pueden utilizar la odometría durante
        el período autónomo para tareas complejas como seguir un "path". Además, la odometría se
        puede utilizar para compensar la latencia cuando se utilizan sistemas de visión.
        Es importante que restablezca sus codificadores a cero antes de usar esta clase.
        Cualquier restablecimiento de pose posterior también requiere que los codificadores
        se restablezcan a cero.
        */
    /* -- IMU & OFFSETS --*/
        /*
        Una unidad de medición inercial (IMU) es un dispositivo electrónico que mide e informa
        la aceleración, orientación, velocidades angulares y otras fuerzas gravitacionales de un objeto.
        Los offsets son constantes que operan en cada motor y encoder que permiten resetear los mismos a 0.
        */

    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */
    public Chassis(HardwareMap hardwareMap){
        /* -- MOTOR ID -- */
        /* -- MOTOR DIRECTION -- */
        /* -- ODOMETRY INITIALIZATION -- */
        /* -- IMU PARAMETERS -- */
            /*
            La declaración del IMU depende mucho de la orientación del ControlHub en el robot. Además, se
            tiene que dar su ubicación en el HardwareMap. Y finalmente se resetea la dirección del mismo.
            */
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORDWARD)
        );
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    /* -- SET SPEED -- */
    /* -- GET RIGHT DISTANCE (POSITION) -- */
    /* -- GET LEFT DISTANCE (POSITION) -- */
    /* -- POSE2D RESET-- */
    /* -- GET POSE (DIFFODOM) -- */

    @Override
    /* -- PERIODICALLY UPDATE DIFFODOM, IMUHEADING, LEFTDISTANCE & RIGHTDISTANCE -- */

    private Rotation2d getIMUHeading(){
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        return Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES));
    }
}


