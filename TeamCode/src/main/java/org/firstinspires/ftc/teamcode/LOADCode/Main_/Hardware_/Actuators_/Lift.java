package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.AprilTagVisionSystem;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Utils_;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;


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
    private final Devices.CRServoClass Lift1 = new Devices.CRServoClass();
    private final Devices.CRServoClass Lift2 = new Devices.CRServoClass();

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
     Public method for setting lift servo power.
     <br>
     <code><b><i>WARNING:</i> THIS METHOD HAS NO BUILT IN SAFETY, OUTREACH USE ONLY!</b></code>
     */
    public void UNSAFESetLiftPower(double power){
        Lift1.setPower(power);
        Lift2.setPower(power);
    }

    /**
    @param power Double value from [-1,1]
    <br><br>
    Sets the power for Lift1 & Lift2.
     */
    private void setLiftPower(double power){
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
}
