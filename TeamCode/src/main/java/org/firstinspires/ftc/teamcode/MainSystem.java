package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*Esto es un OpMode. Este es el archivo principal que se ejecuta en el robot, para
el periodo TeleOperado. Todos los comandos, librerias y subsistemas tienen que ser 
importados. */


@TeleOp
public class MainSystem extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        /* --------------------- SUBSYSTEM INSTANCES --------------------- */

        
        /*
        Es importante crear instancias de cada subsistema que nos gustaría incluir dentro del "MainSystem",
        de esta manera, el programa reconocerá los subsistems y podremos trabajar con ellos. 
        
        Ej. Shooter shooter = new Shooter(hardwareMap); 
        */


        /* --------------------- BUTTONBINDINGS --------------------- */
                                
        /* 
        Como su nombre indica, esta sección del código se dedica a hacer 
        relaciones entre los comandos previamente realizados y los botones 
        disponibles del controller. El formto para realizar los Button Bindings
        nomalmente respeta un mismo orden... 
      
      
        Ej. Button driverRightBumper= driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            driverRightBumper.whileHeld(new Shoot(shooter, 1));
      
        En este ejemplo podemos ver como sigue un orden en dónde primero se asigna el botón
        deseado, y después se declara y se asocia al controller de preferencia ("driver" o "driverOp").
        Posteriormente se vuelve a seleccionar el botón y despué´s se le asigna una acción 
        (whileHeld, whenPressed, etc). Por último se le asigna el comando con su respectivo constructor,
        "(new Shoot(shooter, 1))".  
        */

        // -- CHASSIS MOVEMENT -- //
        // -- ARM MOVEMENT (WITH PID) -- //
        // -- ARM MOVEMENT -- //
        // -- FINGER MOVEMENT -- //

        /* --------------------- OPMODE EXECUTION --------------------- */
        waitForStart();
        
        /* 
        "waitForStart" nos permite realizar acciones específicamente cuando se inicializa el 
        periodo teleoperado, siendo estas, transmitir o reiniciar la telemetria y odometria del 
        robot en el DriverHub.
        */

        chassis.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
        /* Cuando se inicializa se reinicia la posición del Chassis */

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();
            
            /*
            Implementar la odometria nos permite saber las "posiciones" de todos los motores
            en tiempo real, esto es útil ya que nos ayuda a saber si el código está respondiendo
            de la forma esperada.
    
             Ej. telemetry.addData("X", pose.getX());
             
             La palabra o letra ingresada entre comillas, será como sea presentado
             en el DriverHub es decir en este caso se verá presentado así:
             
             X: (Dato que se le pida proveer) 
             */


            // -- ODOMETRY / TELEMETRY -- //
            // -- UPDATE TELEMETRY -- //
            telemetry.update();
            /*Por último, esto nos permite hacer que la telemtry se este actualizando
            constantemente, en tiempo real*/
        }
    }
}
