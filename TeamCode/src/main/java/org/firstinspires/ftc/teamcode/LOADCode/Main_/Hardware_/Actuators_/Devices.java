package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

public class Devices {
    public static class CRServoClass {
        // CR for Continuous Rotation
        protected CRServo servo;
        protected DcMotorSimple.Direction servoDirection;
        public void init(@NonNull OpMode opmode, String servoName) {
            servo = opmode.hardwareMap.get(CRServo.class, servoName);
        }
        /**
         * @return The power the servo was last set to.
         */
        public double getPower() {
            return servo.getPower();
        }
        /**
         * @param power The power to set the servo to. Must be a value between -1 and 1.
         */
        public void setPower(double power) {
            servo.setPower(power);
        }
        /**
         * @return servoDirection : The direction the servo is currently set to.
         */
        public DcMotorSimple.Direction getDirection(){
            return servoDirection;
        }
        /**
         * @param direction The direction to set the servo to.
         */
        public void setDirection(DcMotorSimple.Direction direction) {
            servo.setDirection(direction);
            servoDirection = direction;
        }


    }

    /**
     * Extension of the CRServo class for Axons specifically and equipped to track rotations outside of the [0,360] range.
     */
    public static class AxonClass extends CRServoClass {

        private AnalogInput encoderObject;

        // THE UNITS FOR THE FOLLOWING VARIABLES ARE IN VOLTS ON A SCALE OF 0-3.3
        private double rawAngleV;
        private double deltaAngleV;
        private double totalAngleV;
        private double targetAngleV;


        public void init(@NonNull OpMode opmode, String servoName) {
            super.init(opmode, servoName);
            encoderObject = opmode.hardwareMap.get(AnalogInput.class, servoName + "-encoder");
            rawAngleV = encoderObject.getVoltage();
        }

        /**
         * This should run every loop to count rotations
         */
        public void update() {
            deltaAngleV = encoderObject.getVoltage() - rawAngleV;
            rawAngleV = encoderObject.getVoltage();

            if (deltaAngleV > 3.3 / 2) deltaAngleV -= 3.3;
            else if (deltaAngleV < -3.3 / 2) deltaAngleV += 3.3;

            totalAngleV += deltaAngleV;
        }

        /**
         * Resets the <code>totalAngleV</code> to zero
         */
        public void resetZeroPos(){
            totalAngleV = 0;
        }

        /**
         * @param angle angle in degrees in range [0,360]
         * @return <code>angle</code> - angle in volts in range [0:3.3]
         */
        public double degreesToVolts(double angle){return angle * (3.3/360);}
        /**
         * @param angle angle in volts in range [0,3.3]
         * @return <code>angle</code> - angle in degrees in range [0:360]
         */
        public double voltsToDegrees(double angle){return angle * (360/3.3);}
        /**
         * @return absolute rotations count
         */
        public double getTotalRotations(){return totalAngleV/3.3;}


    }
    public static class DcMotorExClass {

        // Old PID Coefficients
        PIDCoefficients old_pidCoefficients = new PIDCoefficients(0, 0, 0);
        BasicFeedforwardParameters old_ffCoefficients = new BasicFeedforwardParameters(0,0,0);

        // PID Coefficients
        PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);
        BasicFeedforwardParameters ffCoefficients = new BasicFeedforwardParameters(0,0,0);
        /**
         * <h4>Encoder ticks/rotation:</h4><br>
         *      1620rpm Gobilda - 103.8<br>
         *      1150rpm Gobilda - 145.1<br>
         *      223rpm Gobilda - 751.8<br>
         */
        public double ticksPerRotation = 103.8;
        // Target position/velocity of the motor
        public double target = 0;
        // Offset position of the motor
        public double offset = 0;
        // Maximum acceptable error of the angle PID
        public KineticState maxAcceptableError = new KineticState(1, 5);
        // Motor object
        private DcMotorEx motorObject = null;

        /**
         * Initializes the motor hardware
         * @param opmode Allows this class to access the robot hardware objects.
         * @param motorName The name of the motor in the robot's configuration.
         * @param encoderResolution The resolution of the motor's encoder in ticks/rotation.
         */
        public void init (@NonNull OpMode opmode, String motorName, double encoderResolution){
            // Initialize the motor object
            motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
            ticksPerRotation = encoderResolution;
        }
        /**
         * Initializes the motor hardware.
         * The resolution of the motor encoder will default to
         *      103.8 ticks/rotation, the value for a 1620RPM Gobilda motor.
         * @param opmode Allows this class to access the robot hardware objects.
         * @param motorName The name of the motor in the robot's configuration.
         */
        public void init (@NonNull OpMode opmode, String motorName){
            // Initialize the motor object
            motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
        }

        ControlSystem velPID = null;
        ControlSystem posPID = null;

        /**
         * Rebuilds the PIDs if they have changed from the last run of the method.
         */
        private void buildPIDs(){
            if (old_pidCoefficients != pidCoefficients || old_ffCoefficients != ffCoefficients){
                posPID = ControlSystem.builder().posPid(pidCoefficients).build();
                velPID = ControlSystem.builder()
                        .velPid(pidCoefficients)
                        .basicFF(ffCoefficients)
                        .build();
            }
            old_pidCoefficients = pidCoefficients;
            old_ffCoefficients = ffCoefficients;
        }

        /**
         * Sets the value of the PID coefficients of the motor.
         * @param coefficients The values to set the coefficients to.
         */
        public void setPidCoefficients(PIDCoefficients coefficients) {
            pidCoefficients = coefficients;
        }
        /**
         * Sets the value of the FeedForward coefficients of the motor.
         * @param coefficients The values to set the coefficients to.
         */
        public void setFFCoefficients(BasicFeedforwardParameters coefficients) {
            ffCoefficients = coefficients;
        }
        /**
         * Resets the internal encoder of the motor to zero.
         */
        public void resetEncoder(){
            motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorObject.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /**
         * Sets a position offset on the motor in encoder ticks.
         */
        public void setOffsetTicks(double ticks){
            offset = ticks;
        }

        /**
         * Sets a position offset on the motor in degrees
         */
        public void setOffsetDegrees(double degrees){
            setOffsetTicks(degrees * (ticksPerRotation/360));
        }
        /**
         * @return The current offset of the motor in encoder ticks.
         */
        public double getOffsetTicks(){
            return offset;
        }
        /**
         * @return The current offset of the motor in degrees.
         */
        public double getOffsetDegrees(){
            return offset/(ticksPerRotation/360);
        }
        /**
         * Sets the runMode of the motor.
         */
        public void setRunMode(DcMotor.RunMode runMode){
            motorObject.setMode(runMode);
        }
        /**
         * Sets the zeroPowerBehaviour of the motor.
         */
        public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour){
            motorObject.setZeroPowerBehavior(behaviour);
        }
        /**
         * Sets the direction of the motor
         */
        public void setDirection(DcMotorSimple.Direction direction){
            motorObject.setDirection(direction);
        }
        /**
         * @return The current position of the motor in encoder ticks. Can be any value.
         */
        public double getEncoderTicks(){
            return motorObject.getCurrentPosition() + offset;
        }
        /**
         * @param power A value between -1 and 1 that the motor's power will be set to.
         */
        public void setPower(double power){
            motorObject.setPower(power);
        }
        /**
         * @return The angle of the motor in degrees. Can be any value.
         */
        public double getAngleAbsolute(){
            return (getEncoderTicks()/ticksPerRotation*360);
        }
        /**
         * @return The angle of the motor in degrees. Can be any value between 0 and 360.
         */
        public double getAngle(){
            return getAngleAbsolute()%360;
        }
        /**
         * @return The velocity of the motor in encoder ticks/second.
         */
        public double getVelocity(){
            return motorObject.getVelocity();
        }
        /**
         * @return The velocity of the motor in degrees/second.
         */
        public double getDegreesPerSecond(){
            return (getVelocity()/ticksPerRotation)*360;
        }
        /**
         * @return The velocity of the motor in RPM.
         */
        public double getRPM(){
            return (getVelocity()/ticksPerRotation)*60;
        }
        /**
         * @return The power that the motor has been set to.
         */
        public double getPower(){
            return motorObject.getPower();
        }
        /**
         * Uses a PID controller to move the motor to the desired position.
         * Must be called every loop to function properly.
         * @param angle The angle in degrees to move the motor to. Can be any number. </br>
         */
        public void setAngle(double angle){
            setAngle(angle, 0);
        }
        /**
         * Uses a PID controller to move the motor to the desired position and velocity. <br>
         * Must be called every loop to function properly.
         * @param angle The angle in degrees to move the motor to. Can be any number.
         * @param velocity The velocity in degrees/sec that the motor should be spinning at when it reaches the target point.
         */
        public void setAngle(double angle, double velocity){
            target = angle;
            buildPIDs();
            KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
            posPID.setGoal(new KineticState(target, velocity));
            if (!posPID.isWithinTolerance(maxAcceptableError)){
                setPower(posPID.calculate(currentKineticState));
            }else{
                setPower(0);
            }
        }

        /**
         * Uses a PID controller to accelerate the motor to the desired RPM.<br>
         * Must be called every loop to function properly.
         * @param rpm The RPM to accelerate the motor to. Can be any number
         */
        public void setRPM(double rpm){
            target = rpm;
            buildPIDs();
            double degreesPerSecond = target*6;
            KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
            velPID.setGoal(new KineticState(0, degreesPerSecond));
            setPower(velPID.calculate(currentKineticState));
        }

        /**
         * @param units The units to use for the returned motor current
         * @return The current draw of the motor
         */
        public double getCurrent(CurrentUnit units){
            return motorObject.getCurrent(units);
        }
    }
    public static class ServoClass {
        private Servo servo;

        public void init(@NonNull OpMode opmode, String servoName){
            servo = opmode.hardwareMap.get(Servo.class, servoName);
        }

        /**
         * @param angle The angle to set the servo to. Must be a value between 0 and 1,
         *              representing the endpoints of it's movement.
         */
        public void setAngle(double angle){
            servo.setPosition(angle);
        }

        /**
         * @param direction The direction to set the servo to.
         */
        public void setDirection(Servo.Direction direction){
            servo.setDirection(direction);
        }

        /**
         * @return A value between 0 and 1 that the servo has been set to.
         */
        public double getAngle(){
            return servo.getPosition();
        }
    }
    public static class REVColorSensorV3Class {
        private NormalizedColorSensor sensor;

        public void init(@NonNull OpMode opmode, String sensorName){
            sensor = opmode.hardwareMap.get(NormalizedColorSensor.class, sensorName);
        }
        /**
         * @return The current detected color of the sensor
         */
        public NormalizedRGBA getNormalizedColors(){
            return sensor.getNormalizedColors();
        }
        /**
         * @return The current gain of the sensor
         */
        public double getGain(){
            return sensor.getGain();
        }
        /**
         * Sets the current gain of the sensor
         */
        public void setGain(double gain){
            sensor.setGain((float) gain);
        }

        /**
         * @param units The units to be used for the returned distance
         * @return The current distance detected by the sensor
         */
        public double getDistance(DistanceUnit units){
            return ((DistanceSensor) sensor).getDistance(units);
        }
    }
    public static class DualProximitySensorClass {
        private final REVColorSensorV3Class sensor1 = new REVColorSensorV3Class();
        private final REVColorSensorV3Class sensor2 = new REVColorSensorV3Class();

        private boolean sensor1WasLastRead = false;
        private boolean sensor1Triggered = false;
        private boolean sensor2Triggered = false;

        public double threshold = 2;
        public DistanceUnit units = DistanceUnit.CM;

        public void init(@NonNull OpMode opmode, String sensor1Name, String sensor2Name){
            sensor1.init(opmode, sensor1Name);
            sensor2.init(opmode, sensor2Name);
        }

        /**
         * Sets the gain of the sensors
         */
        public void setGain(double gain){
            sensor1.setGain(gain);
            sensor2.setGain(gain);
        }

        /**
         * @return <code>true</code> if either sensor detects and object closer than the <br>
         * set threshold, otherwise <code>false</code>.
         */
        public boolean objectDetected(){
            if (!sensor1WasLastRead){
                sensor1WasLastRead = true;
                sensor1Triggered = (sensor1.getDistance(units) < threshold);
            } else {
                sensor1WasLastRead = false;
                sensor2Triggered = (sensor2.getDistance(units) < threshold);
            }
            return sensor1Triggered || sensor2Triggered;
        }

        /**
         * @return A list containing the distance values for each sensor
         */
        public double[] getDistances(){
            return new double[]{sensor1.getDistance(units), sensor2.getDistance(units)};
        }
    }
    public static class REVHallEffectSensorClass {
        private DigitalChannel sensor;

        public void init(@NonNull OpMode opMode, String sensorName){
            sensor = opMode.hardwareMap.get(DigitalChannel.class, sensorName);
            sensor.setMode(DigitalChannel.Mode.INPUT);
        }

        /**
         * @return <code>true</code> if the sensor detects a magnet, otherwise <code>false</code>.
         */
        public Boolean getTriggered(){
            return !sensor.getState();
        }
    }
    public static class GoBildaPrismBarClass {
        // Maximum length of 4 daisy chained strips is 36 (12 + 12 + 6 + 6)
        // Scrimmage length of 2 daisy chained strips is 24 (12 + 12)

        // Artboards current status:
        GoBildaPrismDriver.Artboard solidRED = GoBildaPrismDriver.Artboard.ARTBOARD_0;
        GoBildaPrismDriver.Artboard solidBLUE = GoBildaPrismDriver.Artboard.ARTBOARD_1;
        GoBildaPrismDriver.Artboard blinkingRED = GoBildaPrismDriver.Artboard.ARTBOARD_2;
        GoBildaPrismDriver.Artboard blinkingBLUE = GoBildaPrismDriver.Artboard.ARTBOARD_3;
        GoBildaPrismDriver.Artboard rainbow = GoBildaPrismDriver.Artboard.ARTBOARD_4;

        GoBildaPrismDriver prism;
        int stripBrightness = 25;
        public void init(@NonNull OpMode opmode, int stripLength){
            prism = opmode.hardwareMap.get(GoBildaPrismDriver.class, "prism");
            prism.setStripLength(stripLength);
            prism.clearAllAnimations();
        }

        public void setStripBrightness(int brightness){
            stripBrightness = brightness;
        }

        public void setDisplayedArtboard(GoBildaPrismDriver.Artboard board){
            prism.loadAnimationsFromArtboard(board);
        }

        public void setStripRainbow(){
            setDisplayedArtboard(rainbow);
        }

        public void setSolidAllianceDisplay(LoadHardwareClass.Alliance alliance){
            if (alliance == LoadHardwareClass.Alliance.RED){
                setDisplayedArtboard(solidRED);
            }else if (alliance == LoadHardwareClass.Alliance.BLUE){
                setDisplayedArtboard(solidBLUE);
            }
        }
        public void setBlinkingAllianceDisplay(LoadHardwareClass.Alliance alliance){
            if (alliance == LoadHardwareClass.Alliance.RED){
                setDisplayedArtboard(blinkingRED);
            }else if (alliance == LoadHardwareClass.Alliance.BLUE){
                setDisplayedArtboard(blinkingBLUE);
            }
        }
    }
    public static class Limelight3AClass {
        public Limelight3A device;
        public LLResult result = null;
        public boolean initialized = false;

        public void init(@NonNull OpMode opMode){
            device = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            initialized = true;
            device.start();
        }

        /**
         * Updates the current result of the limelight camera. <br>
         * Must be called every loop.
         * @param robotHeading the current heading of the robot, used for more accurate 3D tracking
         */
        public void updateResult(double robotHeading){
            device.updateRobotOrientation(robotHeading);
            result = device.getLatestResult();
        }

        /**
         * Sets the current pipeline of the limelight camera.
         */
        public void setPipeline(int pipelineIndex){
            device.pipelineSwitch(pipelineIndex);
        }

        /**
         * @return The currently selected pipeline of the limelight camera.
         */
        public int getPipeline(){
            return device.getStatus().getPipelineIndex();
        }
    }
}

