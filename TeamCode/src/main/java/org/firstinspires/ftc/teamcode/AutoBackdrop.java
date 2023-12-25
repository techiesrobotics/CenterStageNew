package org.firstinspires.ftc.teamcode;

abstract public class AutoBackdrop extends AutoParent{
    @Override
    public double adjustTurn(double angle) {
        return 0;
    }

    @Override
    public double adjustTrajectorydistance(double distance) {
        return 0;
    }

    @Override
    protected void park() {

    }
}
