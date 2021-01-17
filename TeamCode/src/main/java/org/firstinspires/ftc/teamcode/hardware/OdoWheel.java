package org.firstinspires.ftc.teamcode.hardware;

public class OdoWheel
{

    private final double CPR = 8192;
    private double CPMM;
    private Motor encoder;
    private double last;

    public OdoWheel(boolean reversed, double wheel_diameter, Motor encoder)
    {
        this.encoder = encoder;
        CPMM = CPR / (wheel_diameter * Math.PI) * (reversed ? -1 : 1);
        last = 0;
    }

    public double getMM(){
        return encoder.getPosition() / CPMM;
    }

    public double getDeltaMM(){
        double curr = encoder.getPosition() / CPMM;
        double delta = curr - last;
        last = curr;
        return delta;
    }

}
