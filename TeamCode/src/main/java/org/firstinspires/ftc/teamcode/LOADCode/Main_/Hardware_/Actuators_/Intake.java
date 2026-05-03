package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Intake {
    // RESET THESE TO PRIVATE AFTER DECEMBER 6TH!
    private final Devices.DcMotorExClass intake = new Devices.DcMotorExClass();
    private final Devices.CRServoClass belt = new Devices.CRServoClass();
    private final Devices.ServoClass transfer = new Devices.ServoClass();
    public final Devices.DualProximitySensorClass topSensor = new Devices.DualProximitySensorClass();
    public final Devices.DualProximitySensorClass bottomSensor = new Devices.DualProximitySensorClass();

    public enum intakeMode {
        OFF,
        REVERSE,
        ON
    }

    public enum transferState {
        UP,
        DOWN
    }

    public static double topProximitySensorThreshold = 50;
    public static double bottomProximitySensorThreshold = 30;

    public void init(OpMode opmode){
        intake.init(opmode, "intake");
        belt.init(opmode, "belt");
        transfer.init(opmode, "transfer");
        topSensor.init(opmode, "color1", "color2");
        bottomSensor.init(opmode, "color3", "color4");


        intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        topSensor.setGain(2);
        topSensor.units = DistanceUnit.MM;
        topSensor.threshold = topProximitySensorThreshold;
        bottomSensor.setGain(2);
        bottomSensor.units = DistanceUnit.MM;
        bottomSensor.threshold = bottomProximitySensorThreshold;
    }

    /**
     * Sets the current mode of the intake
     * @param intakeDirection the current mode of the Intake
     * @param transferDirection the current mode of the Transfer
     */
    public void setMode(intakeMode intakeDirection, intakeMode transferDirection) {
        switch (intakeDirection){
            case OFF:
                intake.setPower(0);
                break;
            case ON:
                intake.setPower(1);
                break;
            case REVERSE:
                intake.setPower(-1);
                break;
        }
        switch (transferDirection){
            case OFF:
                belt.setPower(0);
                break;
            case ON:
                belt.setPower(1);
                break;
            case REVERSE:
                belt.setPower(-1);
                break;
        }
    }

    public double getCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Outputs one of the following modes
     * <ul>
     *     <li><code>intakeMode.INTAKE_ALL</code></li>
     *     <li><code>intakeMode.INTAKE_NOINTAKE</code></li>
     *     <li><code>intakeMode.REVERSE_ALL</code></li>
     *     <li><code>intakeMode.OFF</code></li>
     * </ul>
     */
    public intakeMode getIntakeMode(){
        double intakePower = intake.getPower();
        switch ((int) intakePower){
            case 0:
                return intakeMode.OFF;
            case 1:
                return intakeMode.ON;
            case -1:
                return intakeMode.REVERSE;
        }
        return intakeMode.OFF;
    }

    public void setTransfer(transferState state) {
        switch (state){
            case UP:
                transfer.setAngle(0.2);
                return;
            case DOWN:
                transfer.setAngle(.475);
        }
    }

    public boolean getTopSensorState(){
        topSensor.threshold = topProximitySensorThreshold;
        return topSensor.objectDetected();
    }
    public boolean getBottomSensorState(){
        bottomSensor.threshold = bottomProximitySensorThreshold;
        return bottomSensor.objectDetected();
    }
}
