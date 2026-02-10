package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.AprilTagVisionSystem;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Utils_;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@Configurable
public class Turret {

    // Hardware definitions
    public final AprilTagVisionSystem vision = new AprilTagVisionSystem();
    public final Devices.DcMotorExClass rotation = new Devices.DcMotorExClass();
    public final Devices.DcMotorExClass flywheel = new Devices.DcMotorExClass();
    private final Devices.DcMotorExClass flywheel2 = new Devices.DcMotorExClass();
    private final Devices.ServoClass hood = new Devices.ServoClass();
    private final Devices.ServoClass gate = new Devices.ServoClass();
    public final Devices.REVHallEffectSensorClass hall = new Devices.REVHallEffectSensorClass();

    // Turret PID coefficients
    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.07, 0.00000000001, 0.003); // 223RPM Motor
    public static PIDCoefficients cameraCoefficients = new PIDCoefficients(0, 0, 0);

    // Flywheel PID coefficients for various speeds
    //public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0.0002, 0, 0); // 4500 RPM
    public static PIDCoefficients flywheelCoefficients4200 = new PIDCoefficients(0.0004, 0, 0); // 4200 RPM
    public static PIDCoefficients flywheelCoefficients3500 = new PIDCoefficients(0.00025, 0, 0); // 3500 RPM
    //public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0.00025, 0, 0); // 3000 RPM

    // Flywheel FF coefficients for various speeds
    //public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.000026,0,0); // 4500 RPM
    public static BasicFeedforwardParameters flywheelFFCoefficients4200 = new BasicFeedforwardParameters(0.000032,0,0); // 4200 RPM
    public static BasicFeedforwardParameters flywheelFFCoefficients3500 = new BasicFeedforwardParameters(0.000032,0,0); // 3500 RPM
    //public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.000031,0,0); // 3000 RPM

    // Actual Flywheel Coefficients
    private PIDCoefficients actualFlywheelCoefficients = flywheelCoefficients3500;
    private BasicFeedforwardParameters actualFlywheelFFCoefficients = flywheelFFCoefficients3500;

    // Define any Enums here
    public enum gatestate {
        OPEN,
        CLOSED,
    }
    public enum flywheelState {
        OFF,
        ON
    }

    // Define any state variables or important parameters here
    /** Stores the current state of the flywheel.*/
    public flywheelState flywheelMode = flywheelState.OFF;
    double targetRPM = 0;
    /** Controls the target speed of the flywheel when it is on.*/
    public static double flywheelNearSpeed = 3500;
    public static double flywheelFarSpeed = 4200;
    /** Controls the upper software limit of the hood.*/
    public static double upperHoodLimit = 260;
    /**
     * Stores the offset of the turret's rotation
     */
    public static double turretOffset = 116;
    /**
     * Stores the zeroing state of the turret
     */
    public static boolean zeroed = false;
    /**
     * Controls which aiming system to use.
     */
    public boolean useCameraAim = false;

    // Stores important objects for later access
    OpMode opMode = null;
    LoadHardwareClass Robot = null;
    PolygonZone robotZone = new PolygonZone(15, 15);

    // The variable to store the InterpLUT table for turret hood aimbot
    public Utils_.InterpLUT hoodLUTnear = new Utils_.InterpLUT();
    public Utils_.InterpLUT hoodLUTfar = new Utils_.InterpLUT();

    public void init(OpMode opmode, LoadHardwareClass robot){
        // Store important objects in their respective variables
        opMode = opmode;
        Robot = robot;

        // Initialize AprilTag vision system
        vision.initAprilTag(opmode);

        // Initialize hardware objects
        rotation.init(opmode, "turret", 751.8 * ((double) 131 / 36));
        flywheel.init(opmode, "flywheel", 28);
        flywheel2.init(opmode, "flywheel2", 28);
        hood.init(opmode, "hood");
        gate.init(opmode, "gate");
        hall.init(opmode, "hall");

        // Set servos to initial positions
        setGateState(gatestate.CLOSED);
        // Set servo directions
        hood.setDirection(Servo.Direction.REVERSE);

        // Flywheel Motor Settings
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);

        // Rotation  Motor Settings
        rotation.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation.setOffsetDegrees(turretOffset);

        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(actualFlywheelCoefficients);
        flywheel.setFFCoefficients(actualFlywheelFFCoefficients);
        flywheel2.setPidCoefficients(actualFlywheelCoefficients);
        flywheel2.setFFCoefficients(actualFlywheelFFCoefficients);

        // TODO Build hood InterpLUT for autoaim
        // Safety points for LUTs
        hoodLUTnear.add(0, 0);
        hoodLUTfar.add(0, 0);

        // --------------------------------------------------------

        // Near zone measurements
        hoodLUTnear.add(53.5,108);
        hoodLUTnear.add(71,168);
        hoodLUTnear.add(77, 181);
        hoodLUTnear.add(88,185);
        hoodLUTnear.add(94.5,185);
        hoodLUTnear.add(96, 180);

        // Far zone measurements
        hoodLUTfar.add(103, 200);
        hoodLUTfar.add(204, 200);

        // --------------------------------------------------------

        // Safety points for LUTs
        hoodLUTnear.add(300, 200);
        hoodLUTfar.add(300, 200);

        // Generate Lookup Table & Initialize servo position
        hoodLUTnear.createLUT();
        hoodLUTfar.createLUT();
        setHood(0);
    }

    /** Sets the value of the internal motor PID coefficients */
    public void updatePIDs(){
        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(actualFlywheelCoefficients);
        flywheel.setFFCoefficients(actualFlywheelFFCoefficients);
        flywheel2.setPidCoefficients(actualFlywheelCoefficients);
        flywheel2.setFFCoefficients(actualFlywheelFFCoefficients);
        rotation.setOffsetDegrees(turretOffset);
    }

    /**
     * Runs the aimbot program to control the turret rotation and hood angle. </br>
     * Must be called every loop to function properly.
     * @param turret If TRUE, enables the rotation autoaim.
     *               Otherwise, sets the turret to face forwards.
     * @param hood If TRUE, enables the hood autoaim.
     *             Otherwise, sets the hood to the highest launch angle.
     * @param hoodOffset a offset to apply to the hood angle in degrees of servo rotation.
     */
    public void updateAimbot(boolean turret, boolean hood, double hoodOffset){

        robotZone.setPosition(Robot.drivetrain.follower.getPose().getX(), Robot.drivetrain.follower.getPose().getY());
        robotZone.setRotation(Robot.drivetrain.follower.getPose().getHeading());

        if (turret){
            updateRotationalAimbot();
        }else{
            rotation.setAngle(90);
        }
        if (hood){
            updateHoodAimbot(hoodOffset);
        }else{
            setHood(0);
        }
    }
    private void updateRotationalAimbot(){
        int targetID = 24;
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.BLUE){
            targetID = 20;
        }

        if (!useCameraAim && vision.tagDetected(targetID) && Math.abs(rotation.target - rotation.getAngle()) < 10){
            useCameraAim = true;
        }
        if (useCameraAim && !vision.tagDetected(targetID) || rotation.target > 360 || rotation.target < 0){
            useCameraAim = false;
        }

        if (useCameraAim){
            rotation.target = rotation.getAngleAbsolute();
            ControlSystem pid = ControlSystem.builder()
                    .posPid(cameraCoefficients)
                    .build();
            pid.setGoal(new KineticState(0));
            rotation.setPower(pid.calculate(new KineticState(vision.getRBE(targetID).b)));
        }else{
            if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED){
                rotation.setAngle(Math.min(Math.max(0, rotationalAimbotLocalizer()-2), 360));
            }else{
                rotation.setAngle(Math.min(Math.max(0, rotationalAimbotLocalizer()+2), 360));
            }
        }
    }
    private void updateHoodAimbot(double offset){
        // Set the hood angle
        Pose goalPose = new Pose(0,144,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {goalPose = new Pose(144, 144, 0);}
        if (robotZone.isInside(LoadHardwareClass.FarLaunchZone)){
            setHood(hoodLUTfar.get(Robot.drivetrain.follower.getPose().distanceFrom(goalPose)));
        }else{
            setHood(hoodLUTnear.get(Robot.drivetrain.follower.getPose().distanceFrom(goalPose)));
        }
        setHood(getHood() + offset);
    }

    /**
     * Calculates the target angle to rotate the turret to in order to aim at the correct goal. </br>
     * Currently uses Pinpoint Odometry and trigonometry to get the angle.
     */
    public double rotationalAimbotLocalizer (){
        Pose goalPose = calcGoalPose();

        return (Math.toDegrees(Math.atan2(
                goalPose.getY()-Robot.drivetrain.follower.getPose().getY(),
                goalPose.getX()-Robot.drivetrain.follower.getPose().getX())
        ) - Math.toDegrees(Robot.drivetrain.follower.getPose().getHeading()) + 90)%360;
    }

    /**
     * Calculates the proper goal pose
     * @return a pose of the rotational aimbot's target position.
     */
    public Pose calcGoalPose(){
        robotZone.setPosition(Robot.drivetrain.follower.getPose().getX(), Robot.drivetrain.follower.getPose().getY());
        robotZone.setRotation(Robot.drivetrain.follower.getPose().getHeading());

        Pose nearGoalPose = new Pose(8,140,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {nearGoalPose = new Pose(140, 140, 0);}
        Pose farGoalPose = new Pose(16,140,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {farGoalPose = new Pose(136, 140, 0);}

        if(robotZone.isInside(LoadHardwareClass.FarLaunchZone)){
            return farGoalPose;
        }else{
            return nearGoalPose;
        }
    }

    /**
     * Sets the state of the turret gate.
     */
    public void setGateState(gatestate state){
        if (state == gatestate.CLOSED){
            gate.setAngle(0.47);
        }else if (state == gatestate.OPEN){
            gate.setAngle(0.5);
        }
    }

    /**
     * Sets the angle of the hood.
     * @param angle An angle in degrees that is constrained to between 0 and the upper hood limit.
     */
    public void setHood(double angle){
        hood.setAngle(Math.min(Math.max(angle, 0), upperHoodLimit)/(360*5));
    }

    /**
     * Gets the last set position of the turret hood.
     * @return The angle of the hood in degrees.
     */
    public double getHood(){
        return hood.getAngle() * 360 * 5;
    }

    /**
     * Gets the current state of the turret gate.
     * Outputs one of the following modes.
     * <ul>
     *     <li><code>gatestate.OPEN</code></li>
     *     <li><code>gatestate.CLOSED</code></li>
     * </ul>
     */
    public gatestate getGate(){
        if (gate.getAngle() == 0.47){
            return gatestate.OPEN;
        } else {
            return gatestate.CLOSED;
        }
    }

    /**
     * Sets the RPM of the flywheel.
     * @param rpm
     * Range [0,6000]
     */
    private void setFlywheelRPM(double rpm){
        if (rpm == 0){
            flywheel.target = 0;
            flywheel.setPower(0);
            flywheel2.setPower(0);
        }else{
            flywheel.setRPM(rpm);
            flywheel2.setPower(flywheel.getPower());
        }
    }

    /**
     * Gets the current RPM of the flywheel.
     */
    public double getFlywheelRPM(){
        return flywheel.getRPM();
    }

    /**
     * Sets the target state of the Flywheel. </br>
     * <code>updateFlywheel()</code> must be called every loop for this to be effective.
     * @param state The state to set the flywheel to (ON/OFF)
     */
    public void setFlywheelState(flywheelState state){
        flywheelMode = state;
    }

    public double getFlywheelCurrentMaxSpeed(){
        return targetRPM;
    }


    public boolean zeroTurret(){
        if (!zeroed){
            rotation.setPower(1);
            if (hall.getTriggered()){
                rotation.setPower(0);
                rotation.resetEncoder();
                zeroed = true;
            }
        }
        return !zeroed;
    }

    /**
     * Updates the flywheel PID. Must be called every loop.
     */
    public void updateFlywheel(){
        robotZone.setPosition(Robot.drivetrain.follower.getPose().getX(), Robot.drivetrain.follower.getPose().getY());
        robotZone.setRotation(Robot.drivetrain.follower.getPose().getHeading());

        opMode.telemetry.addData("In Far Zone", robotZone.isInside(LoadHardwareClass.FarLaunchZone));
        opMode.telemetry.addData("In Near Zone", robotZone.isInside(LoadHardwareClass.ReallyNearLaunchZoneRed));

        if (robotZone.isInside(LoadHardwareClass.FarLaunchZone)){
            targetRPM = flywheelFarSpeed;
            actualFlywheelCoefficients = flywheelCoefficients4200;
            actualFlywheelFFCoefficients = flywheelFFCoefficients4200;
        }else{
            targetRPM = flywheelNearSpeed;
            actualFlywheelCoefficients = flywheelCoefficients3500;
            actualFlywheelFFCoefficients = flywheelFFCoefficients3500;
        }

        if (flywheelMode == flywheelState.ON){
            setFlywheelRPM(targetRPM);
        }else if (flywheelMode == flywheelState.OFF){
            setFlywheelRPM(0);
        }
    }
}
