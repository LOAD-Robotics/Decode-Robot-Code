package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Lift {
    private final Devices.AxonClass Lift1 = new Devices.AxonClass();
    private final Devices.AxonClass Lift2 = new Devices.AxonClass();

    public boolean liftIsActivated = false;
    public boolean lift1IsDone = false;
    public boolean lift2IsDone = false;


    private double initialLift1Angle = 0;
    private double initialLift2Angle = 0;

    public static double targetRotationCount = 2.8;
    public static double liftPower = 1;

    /**
     @param opMode The current OpMode
     */
    public void init(OpMode opMode){
        Lift1.init(opMode, "lift1");
        Lift2.init(opMode, "lift2");
        update();
        initialLift1Angle = Lift1.getTotalRotations();
        initialLift2Angle = Lift2.getTotalRotations();
        setLiftPower(0);
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
        double lift1Percent = (int) ((Lift1.getTotalRotations() - initialLift1Angle) / targetRotationCount) * 100;
        double lift2Percent = (int) ((Lift2.getTotalRotations() - initialLift1Angle) / targetRotationCount) * 100;
        return (lift1Percent + lift2Percent)/2;
    }
    public double getLift1Rotations(){
        return (Lift1.getTotalRotations() - initialLift1Angle);
    }
    public double getLift2Rotations(){
        return (Lift2.getTotalRotations() - initialLift2Angle);
    }

    public void update(){
        Lift1.update();
        Lift2.update();

        if (liftIsActivated){
            if (Lift1.getTotalRotations() - initialLift1Angle < targetRotationCount && !lift1IsDone){
                Lift1.setPower(liftPower);
            }else{
                Lift1.setPower(0);
                lift1IsDone = true;
            }
            if (Lift2.getTotalRotations() - initialLift2Angle < targetRotationCount && !lift2IsDone){
                Lift2.setPower(liftPower);
            }else{
                Lift2.setPower(0);
                lift2IsDone = true;
            }
        }
    }
}
