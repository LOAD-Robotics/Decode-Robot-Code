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

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.OFF;
import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.ON;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drawing;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name="Teleop_Testing_", group="TeleOp")
public class Teleop_Testing_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private final TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private final Telemetry ftcTelemetry = super.telemetry;
    private final JoinedTelemetry telemetry = new JoinedTelemetry(ftcTelemetry, panelsTelemetry);
    public LoadHardwareClass Robot = new LoadHardwareClass(this);
    public Pedro_Paths paths = new Pedro_Paths();

    public int shootingState = 0;
    public TimerEx stateTimer = new TimerEx();

    int hood = 0;

    @Override
    public void runOpMode() {

        Robot.init(paths.farStart);
        LoadHardwareClass.selectedAlliance = LoadHardwareClass.Alliance.RED;
        Drawing.init();

        Turret.zeroed = false;
        Turret.zeroingState = 0;

        while (!isStopRequested() && Robot.turret.zeroTurret()){
            Robot.sleep(0);
            telemetry.addData("Current", Robot.turret.rotation.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y/3,
                    gamepad1.left_stick_x/3,
                    gamepad1.right_stick_x / 4,
                    true
            );

            int change = 10;

            if (gamepad1.dpadUpWasPressed()){
                hood += change;
            }
            if (gamepad1.dpadDownWasPressed()){
                hood -= change;
            }

            telemetry.addData("Hood Angle", hood);
            telemetry.addData("Distance", Robot.drivetrain.distanceFromGoal());
            telemetry.addData("Speed", Robot.turret.getFlywheelCurrentMaxSpeed());

            if (gamepad1.left_trigger > 0.5){
                Robot.intake.setMode(ON, ON);
            }else{
                Robot.intake.setMode(OFF, OFF);
            }

            if (gamepad1.yWasPressed()){
                Robot.turret.setFlywheelState(Turret.flywheelState.ON);
            }else if (gamepad1.aWasPressed()){
                Robot.turret.setFlywheelState(Turret.flywheelState.OFF);
            }
            Robot.turret.updateFlywheel(0);

            if (gamepad1.bWasPressed() && shootingState < 1 && Robot.turret.getFlywheelRPM() > Robot.turret.getFlywheelCurrentMaxSpeed()-100) {
                shootingState++;
            }
            if (gamepad1.xWasPressed()){
                shootingState = 4;
            }
            boolean forceGateOpen = false;
            switch (shootingState) {
                case 0:
                    telemetry.addData("Shooting State", "OFF");
                    break;
                case 1:
                    Robot.turret.setFlywheelState(Turret.flywheelState.ON);
                    if (Robot.turret.getGate() == Turret.gatestate.CLOSED){
                        stateTimer.restart();
                    }
                    if (!forceGateOpen){
                        Robot.turret.setGateState(Turret.gatestate.OPEN);
                    }
                    telemetry.addData("Shooting State", "GATE OPENING");
                    if (stateTimer.getElapsed() > 0.2){
                        shootingState = 2;
                        stateTimer.restart();
                    }
                    break;
                case 2:
                    Robot.intake.setMode(ON, ON);
                    telemetry.addData("Shooting State", "INTAKE_NOINTAKE FIRST TWO");
                    if (stateTimer.getElapsed() > 0.7 && Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState()){
                        shootingState = 3;
                        stateTimer.restart();
                    }
                    break;
                case 3:
                    Robot.intake.setMode(OFF, ON);
                    Robot.intake.setTransfer(Intake.transferState.UP);
                    telemetry.addData("Shooting State", "INTAKE_NOINTAKE FINAL");
                    if (stateTimer.getElapsed() > 0.5) {
                        shootingState = 4;
                    }
                    break;
                case 4:
                    if (!forceGateOpen){
                        Robot.turret.setGateState(Turret.gatestate.CLOSED);
                    }
                    Robot.intake.setMode(OFF, OFF);
                    Robot.intake.setTransfer(Intake.transferState.DOWN);
                    telemetry.addData("Shooting State", "RESET");
                    shootingState = 0;
            }

            Robot.turret.rotation.setAngle(90);
            Robot.turret.setHood(Math.max(0, Math.min(hood, Turret.upperHoodLimit)));
            Robot.turret.updatePIDs();

            telemetry.addData("Loop Time (ms)", loopTimer.time(TimeUnit.MILLISECONDS));
            telemetry.update();
            loopTimer.reset();
        }
    }
}
