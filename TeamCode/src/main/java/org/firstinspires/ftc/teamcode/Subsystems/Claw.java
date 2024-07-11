/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends SubsystemBase {
    /*
    El HardwareMap, como su nombre lo dice, es una forma sencilla de ubicar el Hardware del robot en
    sus conexiónes al ControlHub, usualmente, los motores funcionan por medio de IDs, pero elementos como
    el IMU operan internamente aunque se tiene que "declarar" su ubicación dentro el mismo HardwareMap.
    */

    /* -- MOTOR DECLARATION -- */
    private ServoEx rightServo;
    private ServoEx leftServo;

    /*
    Está función permite al programa asignar y dar sentido al Hardware y donde está ubicado en los IDs del
    ControlHub los motores de cada Subsistema. Además, daremos información al Hardware sobre su comportamiento,
    como orientación o inicialización de la odometría y el IMU.
    */

    public Claw (HardwareMap hardwareMap) {
        /* -- SERVO IDs --*/
        rightServo = new SimpleServo(hardwareMap, "grab_Rightservo", 0, 180);
        leftServo = new SimpleServo(hardwareMap, "grab_Leftservo", 0, 180);
        /* -- MOTOR DIRECTION -- */
        leftServo.setInverted(true);
        //rightServo.setInverted(true);

    }

    /* -- GET RIGHT POSITION -- */
    public double getRightPosition() {
        return rightServo.getPosition();
    }
    /* -- GET LEFT POSITION -- */
    public double getLeftPosition () {
        return leftServo.getPosition();
    }
    /* -- SET POSITION -- */
    public void setPosition(double clawPosition) {
        rightServo.setPosition(clawPosition);
        leftServo.setPosition(clawPosition);
    }

}