package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.Finger;

import java.util.concurrent.TimeUnit;

public class MoveFinger extends CommandBase {

    private Finger finger;

    private double FingerMotorPosition;

    private Timing.Timer timer;

    public MoveFinger(Finger subsystem, double FingerMotorPosition) {
        finger = subsystem;
        this.FingerMotorPosition = FingerMotorPosition;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        finger.FingerPosition(FingerMotorPosition);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
