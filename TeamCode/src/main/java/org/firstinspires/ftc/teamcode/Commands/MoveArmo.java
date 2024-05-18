package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Armo;

public class MoveArmo extends CommandBase {

    private final Armo armo;

    private final double targetHeight;

    public MoveArmo(Armo subsystem, double targetHeight) {
        this.armo = subsystem;
        this.targetHeight = targetHeight;
        addRequirements(armo);
    }

    @Override
    public void initialize() {
        armo.setGoal(targetHeight);
    }

    @Override
    public boolean isFinished() {
        double currentHeight = armo.getHeight();
        return Math.abs(targetHeight - currentHeight) < 0.05;
    }
}
