/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/

package org.firstinspires.ftc.teamcode.Commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Utils.JoystickHandler;

public class MoveChassis extends CommandBase {

    private final Chassis chassis;
    private final Gamepad driverGamepad;

            public MoveChassis(Chassis subsystem, Gamepad driverGamepad){
                chassis = subsystem;
                this.driverGamepad = driverGamepad;
                addRequirements(subsystem);
            }






    @Override
    public void execute(){

        double right = -driverGamepad.right_stick_x;
        double left = -driverGamepad.left_stick_y;

        right = JoystickHandler.handleJoystickInput(right);
        left = JoystickHandler.handleJoystickInput(left);


        chassis.setSpeed(left, right);
    }
}