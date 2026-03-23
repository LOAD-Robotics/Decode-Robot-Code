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

    private int rotationCount;
    private int targetRotationCount;

    /**
     @param opMode The current OpMode
     */
    public void init(OpMode opMode){
        Lift1.init(opMode, "lift1");
        Lift2.init(opMode, "lift2");
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



    /**
     @param target Int value for target rotations.
     <br><br>
     Sets the target position for the Garage-Bot in rotations.
     */
    public void setTargetRotations(int target){
        targetRotationCount = target;
    }

    /**
     @return <code>target</code> - Int value for the current target rotation.
     <br><br>
     Gets the current target position for the Garage-Bot in rotations.
     */
    public int getTargetRotations() {
        return targetRotationCount;
    }

    // Finish after making AXON class
    public boolean checkWraparound(){
        return false;
    }

    public double getLift1Rotations(){
        return Lift1.getTotalRotations();
    }

    public void update(){
        Lift1.update();
        Lift2.update();
    }
}
