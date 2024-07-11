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


    /* -- MOTOR DECLARATION -- */
    private ServoEx rightServo;
    private ServoEx leftServo;


    public Claw (HardwareMap hardwareMap) {
        /* -- SERVO IDs --*/
        rightServo = new SimpleServo(hardwareMap, "grab_RightServo", 0, 180);
        leftServo = new SimpleServo(hardwareMap, "grab_LeftServo", 0, 180);
        /* -- MOTOR DIRECTION -- */
        leftServo.setInverted(true);
    }

    /* -- GET RIGHT POSITION -- */
    public double getRightPosition(){
        return rightServo.getPosition();
    }

    /* -- GET LEFT POSITION -- */
    public double getLeftPosition() {
        return leftServo.getPosition();
    }
    /* -- SET POSITION -- */

        public void setPosition(double clawPosition) {
            rightServo.setPosition(clawPosition);
            leftServo.setPosition(clawPosition);

        }
}
