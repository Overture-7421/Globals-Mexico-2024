package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DoubleArm;

public class MoveDoubleArm extends CommandBase {

    private final DoubleArm doubleArm;

    private final double lowerTarget;
    private final double upperTarget;

    public MoveDoubleArm(DoubleArm doubleArm, double lowerTarget, double upperTarget) {
        this.doubleArm = doubleArm;
        this.lowerTarget = lowerTarget / 360;
        this.upperTarget = upperTarget / 360;
        addRequirements(doubleArm);
    }

    @Override
    public void initialize() {
        doubleArm.setTargets(this.lowerTarget, this.upperTarget);
    }

    @Override
    public boolean isFinished() {
        double lowerPosition = doubleArm.getLowerPosition();
        double upperPosition = doubleArm.getUpperPosition();
        return (Math.abs(lowerTarget - lowerPosition) < 0.05) && (Math.abs(upperTarget - upperPosition) < 0.05);

    }
}
