/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND NEAR ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_.Teleop_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.transferState;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret.flywheelState;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret.gatestate;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@Configurable
@TeleOp(name="Teleop_Main_", group="TeleOp")
public class Teleop_Main_ extends LinearOpMode {
    private final JoinedTelemetry debug = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    private final double stickDeadZones = 0.2;

    private int shootingState = 0;
    private final TimerEx stateTimer = new TimerEx(1);
    private double hoodOffset = 0;

    // Create a new instance of our Robot class
    private final LoadHardwareClass Robot = new LoadHardwareClass(this);
    // Create a new Paths instance
    private final Pedro_Paths Paths = new Pedro_Paths();

    // Controls mapping variables

    private double drivetrain_drive;
    private double drivetrain_strafe;
    private double drivetrain_rotate;

    private double drivetrain_speed_medium;
    private double drivetrain_speed_slow;

    private boolean drivetrain_reset_pose;

    private float intake;

    private boolean toggle_flywheel;
    private boolean start_shooting;

    private boolean hood_tweak_up;
    private boolean hood_tweak_down;



    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Initialize all hardware of the robot and build paths for auto movement
        Robot.init(MecanumDrivetrainClass.robotPose);
        Paths.buildPaths(selectedAlliance, Robot.drivetrain.follower);
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Run_Controls();
            updateInputs();

            debug.addLine("DRIVETRAIN DATA");
            debug.addData("Speed Multiplier", Robot.drivetrain.speedMultiplier);
            debug.addData("X Position", Robot.drivetrain.follower.getPose().getX());
            debug.addData("Y Position", Robot.drivetrain.follower.getPose().getY());
            debug.addData("Heading", Math.toDegrees(Robot.drivetrain.follower.getPose().getHeading()));
            debug.addData("Distance From Goal", Robot.drivetrain.distanceFromGoal());
            debug.addLine("TURRET ROTATION DATA");
            debug.addData("Target Angle", Robot.turret.rotation.target);
            debug.addData("Actual Angle", Robot.turret.rotation.getAngleAbsolute());
            debug.addLine("HOOD DATA");
            debug.addData("Servo Angle", Robot.turret.getHood());
            debug.addData("Current Offset", hoodOffset);
            debug.addLine("FLYWHEEL DATA");
            debug.addData("Speed Percent", Robot.turret.getFlywheelPercent()+"%");
            debug.addData("Target Speed", Robot.turret.flywheel.target);
            debug.addData("Actual Speed", Robot.turret.getFlywheelRPM());
            debug.addData("Motor Set Power", Robot.turret.flywheel.getPower());
            debug.addLine("INTAKE DATA");
            debug.addData("Current Mode", Robot.intake.getMode());
            debug.addData("Upper Sensor", Robot.intake.getTopSensorState());
            debug.addData("Lower Sensor", Robot.intake.getBottomSensorState());
            debug.update();
        }
    }
    
    public void updateInputs(){
        drivetrain_drive =          gamepad1.left_stick_y;
        drivetrain_strafe =         gamepad1.left_stick_x;
        drivetrain_rotate =         gamepad1.right_stick_x;
        drivetrain_speed_medium =   gamepad1.left_trigger;
        drivetrain_speed_slow =     gamepad1.right_trigger;
        drivetrain_reset_pose =     gamepad1.guide;
        intake =                    gamepad1.right_stick_y;
        toggle_flywheel =           gamepad1.yWasPressed();
        start_shooting =            gamepad1.bWasPressed();
        hood_tweak_up =             gamepad1.dpadUpWasPressed();
        hood_tweak_down =           gamepad1.dpadDownWasPressed();
    }

    /**
     * <h1>Gamepad 1 Controls</h1>
     * <ul>
     *     <li><b>Analog Inputs</b><ul>
     *         <li>Left Stick:<ul>
     *             <li>X: <code>Rotate Left/Right</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Right Stick:<ul>
     *             <li>X: <code>Strafe Left/Right</code></li>
     *             <li>Y: <code>Drive Forwards/Backwards</code></li>
     *         </ul></li>
     *         <li>Left Trigger: <code>Slow Mod</code></li>
     *         <li>Right Trigger: <code>Quick Mod</code></li>
     *     </ul></li>
     *
     *     <li><b>Button Inputs</b></li><ul>
     *         <li>Letter Buttons:<ul>
     *             <li>A: <code>N/A</code></li>
     *             <li>B: <code>N/A</code></li>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Letter Buttons:<ul>
     *             <li>DpadUp: <code>N/A</code></li>
     *             <li>DpadDown: <code>N/A</code></li>
     *             <li>DpadLeft: <code>N/A</code></li>
     *             <li>DpadRight: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Bumpers:<ul>
     *             <li>Left Bumper: <code>N/A</code></li>
     *             <li>Right Bumper: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Stick Buttons:<ul>
     *             <li>Left Stick Button: <code>N/A</code></li>
     *             <li>Right Stick Button: <code>N/A</code></li>
     *         </ul></li>
     *     </ul>
     * </ul>
     */
    private void Run_Controls() {

        // Update Turret Aimbot
        Robot.turret.updateAimbot(true, true, hoodOffset);
        // Update Flywheel Controls
        Robot.turret.updateFlywheel();

        // Run the manual hood controls constantly, so that it's angle
        // can be tweaked even while the state machine is running.
        manualHoodControls();
        drivetrainControls();
        robotPoseResetting();

        // Activate the shooting state machine
        if (startShootingStateMachine()) {
            shootingState++;
        }
        switch (shootingState) {
            case 0:
                // These controls are only on when
                manualFlywheelControls();
                manualIntakeControls();
                debug.addData("Shooting State", "OFF");
                return;
            case 1:
                shootingStateMachine1();
                debug.addData("Shooting State", "FIRING BEGAN");
                return;
            case 2:
                shootingStateMachine2();
                debug.addData("Shooting State", "FIRING FINISHED");
                return;
            case 3:
                shootingStateMachine3();
                debug.addData("Shooting State", "RESETTING");
        }
    }

    // These functions contain the manual controls for the robot subsystems.
    /**
     * This function contains the manual intake controls, for use when the shooting state machine is not in effect.
     */
    private void manualIntakeControls (){
        // Intake Controls
        if (Math.abs(intake) >= stickDeadZones){
            Robot.intake.setMode(intakeMode.INTAKING);
        }else{
            Robot.intake.setMode(intakeMode.IDLE);
        }
    }
    /**
     * This function runs all the controls for the flywheel, both the manual toggling,
     * and automatic activation based on the number of artifacts in the robot.
     */
    private void manualFlywheelControls (){
        // Flywheel Controls
        if (toggle_flywheel) {
            if (Robot.turret.flywheelMode == flywheelState.OFF) {
                Robot.turret.setFlywheelState(flywheelState.ON);
            } else {
                Robot.turret.setFlywheelState(flywheelState.OFF);
            }
        }
        // This turns on the flywheel if it detects two balls within the robot
        if (Robot.intake.getTopSensorState() && Robot.intake.getBottomSensorState()){
            Robot.turret.setFlywheelState(flywheelState.ON);
        }
    }
    /**
     * This function contains the controls to manually
     * tweak the hood angle to compensate for any hardware errors.
     */
    private void manualHoodControls (){
        // Hood Controls
        if (hood_tweak_up){
            hoodOffset += 10;
        }else if (hood_tweak_down){
            hoodOffset -= 10;
        }
    }
    /**
     * This function contains the controls for the robot's drivetrain, including
     * the speed modifier controls for more precision when driving.
     */
    private void drivetrainControls(){
        // Adjusts the speed of the drivetrain based on inputs
        if (drivetrain_speed_slow >= stickDeadZones) {
            Robot.drivetrain.speedMultiplier -= 0.33;
        }else if (drivetrain_speed_medium >= stickDeadZones) {
            Robot.drivetrain.speedMultiplier -= 0.66;
        }else{
            Robot.drivetrain.speedMultiplier = 1;
        }

        // Applies a conditional multiplier to the turning to allow driving while turning
        double turn = drivetrain_rotate;
        if (!(drivetrain_drive < stickDeadZones && drivetrain_strafe < stickDeadZones)){
            turn = turn/2;
        }

        // Sends the inputs to the
        Robot.drivetrain.pedroMecanumDrive(
                drivetrain_drive,
                drivetrain_strafe,
                turn,
                true
        );
    }
    /**
     * Handles the resetting of the robot's pose on the field in case of odometry drift.
     */
    private void robotPoseResetting(){
        if (drivetrain_reset_pose){
            if (selectedAlliance == LoadHardwareClass.Alliance.RED){
                Robot.drivetrain.follower.setPose(new Pose(8, 8, Math.toRadians(90)));
            }else if (selectedAlliance == LoadHardwareClass.Alliance.BLUE){
                Robot.drivetrain.follower.setPose(new Pose(136, 8, Math.toRadians(90)));
            }
        }
    }
    // These functions contain the logic for each state of the shooting state machine
    private void shootingStateMachine1 (){
        if (Robot.intake.getMode() == intakeMode.OFF){
            stateTimer.restart();
            stateTimer.start();
        }
        Robot.intake.setMode(intakeMode.INTAKING);
        Robot.turret.setGateState(gatestate.OPEN);
        if (stateTimer.isDone() && Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState()){
            shootingState++;
        }
    }
    private void shootingStateMachine2 (){
        if (Robot.intake.getMode() == intakeMode.INTAKING){
            stateTimer.restart();
            stateTimer.start();
        }
        Robot.intake.setMode(Intake.intakeMode.SHOOTING);
        Robot.intake.setTransfer(transferState.UP);

        if (stateTimer.isDone()) {
            shootingState++;
        }
    }
    private void shootingStateMachine3 (){
        Robot.turret.setFlywheelState(flywheelState.OFF);
        Robot.turret.setGateState(gatestate.CLOSED);
        Robot.intake.setMode(intakeMode.OFF);
        Robot.intake.setTransfer(transferState.DOWN);
        shootingState = 0;
    }
    /**
     * Returns whether or not to begin the shooting state machine,
     * based on gamepad inputs and the current flywheel speed.
     */
    private boolean startShootingStateMachine(){
        return (
                start_shooting &&
                shootingState == 1 &&
                Robot.turret.isFlywheelReadyToShoot()
        );
    }
}