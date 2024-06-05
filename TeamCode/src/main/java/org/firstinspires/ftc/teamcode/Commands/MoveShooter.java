package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.util.concurrent.TimeUnit;


public class MoveShooter extends CommandBase {
    private Shooter shooter;
    private double position;

    public MoveShooter(Shooter subsystem, double shooterPosition) {
        this.position = shooterPosition;
        shooter = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        shooter.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        double shooterPosition = shooter.getPosition();
        return (Math.abs(position - shooterPosition) < 0.01);
    }
}