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


public class Lift {
    private final Devices.CRServoClass Lift1 = new Devices.CRServoClass();
    private final Devices.CRServoClass Lift2 = new Devices.CRServoClass();


    public void init(OpMode opMode){
        Lift1.init(opMode, "Lift1");
        Lift2.init(opMode, "Lift2");

    };
}
