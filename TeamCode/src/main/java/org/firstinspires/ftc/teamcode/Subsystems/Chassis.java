/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Chassis extends SubsystemBase {
    /*
    El HardwareMap, como su nombre lo dice, es una forma sencilla de ubicar el Hardware del robot en
    sus conexiónes al ControlHub, usualmente, los motores funcionan por medio de IDs, pero elementos como
    el IMU operan internamente aunque se tiene que "declarar" su ubicación dentro el mismo HardwareMap.
    */

    /* -- MOTOR DECLARATION --*/
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    //Ex. private DcMotorEx rightDrive;

    /* -- CHASSIS CONSTANTS --*/
    private final double M_PER_TICK = 288;
    static final double TRACKWIDTH = 0.0891286;
    static final double GEAR_REDUCTION = 12;



    /* -- DIFFERENTAL DRIVE ODOMETRY DECLARATION*/

     private DifferentialDriveOdometry diffOdom;

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
        private IMU imu;
        private int leftOffset = 0, rightOffset = 0;


    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */
    public Chassis(HardwareMap hardwareMap){

        /* -- MOTOR ID -- */
        leftDrive = (DcMotorEx) hardwareMap.get(DcMotor.class,"left_Drive" );
        rightDrive = (DcMotorEx) hardwareMap.get(DcMotor.class,"right_Drive");

        // Ex. rightDrive = (DcMotorEx) hardware.get(DcMotor.class, deviceName);

        /* -- MOTOR DIRECTION -- */
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        /* -- ODOMETRY INITIALIZATION -- */

        diffOdom = new DifferentialDriveOdometry(new Rotation2d());
        imu = hardwareMap.get(IMU.class, "imu");

        /* -- IMU PARAMETERS --

            La declaración del IMU depende mucho de la orientación del ControlHub en el robot. Además, se
            tiene que dar su ubicación en el HardwareMap. Y finalmente se resetea la dirección del mismo.*/


        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    /* -- SET SPEED -- */
    public void setSpeed(double linearSpeed, double angularSpeed) {
        rightDrive.setPower((linearSpeed - angularSpeed) / 2);
        leftDrive.setPower((linearSpeed + angularSpeed) / 2);
    }
    /* -- GET RIGHT DISTANCE (POSITION) --*/

    public double rightDistance(){
        return ((rightDrive.getCurrentPosition() / M_PER_TICK) * TRACKWIDTH * Math.PI) / GEAR_REDUCTION;
       }


    /* -- GET LEFT DISTANCE (POSITION) --*/

    public double leftDistance(){
            return ((leftDrive.getCurrentPosition() / M_PER_TICK) * TRACKWIDTH * Math.PI) / GEAR_REDUCTION;
        }


    /* -- POSE2D RESET--*/

    public void resetPose(Pose2d pose){
        leftOffset = leftDrive.getCurrentPosition();
        rightOffset = rightDrive.getCurrentPosition();
        diffOdom.resetPosition(pose, getIMUHeading());
     }


    /* -- GET POSE (DIFFODOM) --*/

    public Pose2d getPose() {
            return diffOdom.getPoseMeters();
        }


    @Override
    /* -- PERIODICALLY UPDATE DIFFODOM, IMUHEADING, LEFTDISTANCE & RIGHTDISTANCE -- */
    public void periodic() {
        diffOdom.update(getIMUHeading(), leftDistance(), rightDistance());
    }

    private Rotation2d getIMUHeading(){
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        return Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES));
    }
}


