/*
Esta primera línea de todos los subsystemas y comandos indica el folder y ubicación
donde se encuentra el archivo actual donde trabajas. En este caso estamos dentro de
la carpeta de "Subsystems". Nota como existe una diferencia entre "package" e "import".
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Controllers.FRCProfiledPIDController;
import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;

public class SingleArm extends SubsystemBase {
    /* -- MOTOR DECLARATION -- */
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;

    private FRCProfiledPIDController armPID;

    /* -- MOTOR ENCODER CONSTANTS -- */
    public static final double COUNTS_PER_REV = 288;
    public static final double MOTOR_GEAR_RATIO = 1;

    /* -- MOTOR OFFSET -- */
    private double motorOffset = 35;


    public SingleArm(HardwareMap hardwareMap) {
        /* -- MOTOR ID -- */
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");

        /* -- PID CONSTRUCTOR -- */
        armPID = new FRCProfiledPIDController(0.0, 0.0, 0.0, new FRCTrapezoidProfile.Constraints(3, 2));

        /* -- SET MOTOR MODE AND ZERO POWER BEHAVIOR -- */
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /* Esta configuración nos indica que el motor
                                                          está configurado para alcanzar el nivel de velocidad
                                                                indicado sin la necesidad de un encoder.*/
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); /* Esta configuración nos indica que cuando
                                                                        el motor no reciba ningún tipo de poder
                                                                        va a cambiar su comportamiento a frenarse
                                                                        resistiendo cualquier tipo de fuerza externa.*/
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* -- ARMPID RES -- */
        armPID.reset(getPosition()); //Indica que el PID reinicia el comando "getPosition"
        armPID.setGoal(getPosition());//Indica que el PID le da un objetivo al comando "getPosition"
    }

    /* -- RESET ZERO FUNCTION -- */
    public void resetZero() {
        motorOffset = rightMotor.getCurrentPosition();
    }

    /* -- GET POSITION FUNCTION -- */
    public double getPosition() {
        double currentTicks = leftMotor.getCurrentPosition();
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
        rightMotor.setPower(motorOutput);
        leftMotor.setPower(motorOutput);

    }
}