package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Motor
{

    private DcMotorEx motor;
    private double lastUsedPower;
    private final double powerDifferenceToUpdate = 0.01;

    public Motor(LinearOpMode opMode, String label){
        motor = opMode.hardwareMap.get(DcMotorEx.class, label);
    }

    public void setConstants(boolean speedMode, boolean brakeMode, boolean resetEncoder, boolean forward){
        if(resetEncoder)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(speedMode){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(brakeMode){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if(!forward){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setPower(double power){
        if(Math.abs(power - lastUsedPower) > powerDifferenceToUpdate)
        {
            motor.setPower(power);
            lastUsedPower = power;
        }
        if(power == 0 && lastUsedPower != 0){
            motor.setPower(power);
            lastUsedPower = power;
        }
    }

    public double getPosition(){
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setPID(double p, double i, double d)
    {
        motor.setVelocityPIDFCoefficients(p, i, d, 0);
    }

}
