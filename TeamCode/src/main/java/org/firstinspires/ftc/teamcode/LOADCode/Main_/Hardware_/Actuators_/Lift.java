package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/*

Garagebot wishlist

- Get/set target height
- Get/set actual height
- Get/set rotations
Automatic lift function
Wraparound tracking



Axon rotation scale 0-3.3

 */


public class Lift {
    private final Devices.AxonClass Lift1 = new Devices.AxonClass();
    private final Devices.AxonClass Lift2 = new Devices.AxonClass();

    private double lift1InitialAngle;
    private double lift2InitialAngle;

    private boolean liftIsActivated = false;

    public static double targetRotationCount = 2.5;
    public static double liftPower = 1;

    /**
     @param opMode The current OpMode
     */
    public void init(OpMode opMode){
        Lift1.init(opMode, "lift1");
        Lift2.init(opMode, "lift2");
        lift1InitialAngle = Lift1.getTotalRotations();
        lift2InitialAngle = Lift2.getTotalRotations();
    };

    /**
    @param power Double value from [-1,1]
    <br><br>
    Sets the power for Lift1 & Lift2.
     */
    public void setLiftPower(double power){
        Lift1.setPower(power);
        Lift2.setPower(power);
    }
    public void activate(){
        liftIsActivated = true;
    }

    public double getLiftPercentage(){
        double lift1Percent = (int) ((Lift1.getTotalRotations() - lift1InitialAngle) / targetRotationCount) * 100;
        double lift2Percent = (int) ((Lift2.getTotalRotations() - lift2InitialAngle) / targetRotationCount) * 100;
        return (lift1Percent + lift2Percent)/2;
    }

    public void update(){
        Lift1.update();
        Lift2.update();

        if (liftIsActivated){
            if (Lift1.getTotalRotations() - lift1InitialAngle < targetRotationCount){
                Lift1.setPower(liftPower);
            }
            if (Lift2.getTotalRotations() - lift2InitialAngle < targetRotationCount){
                Lift2.setPower(liftPower);
            }
        }
    }
}
