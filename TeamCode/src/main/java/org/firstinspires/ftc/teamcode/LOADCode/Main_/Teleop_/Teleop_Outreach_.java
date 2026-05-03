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

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@TeleOp(name="Teleop_Outreach_", group="TeleOp")
public class Teleop_Outreach_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private double targetRPM = 0;
    public int shootingState = 0;
    public TimerEx stateTimerFifthSec = new TimerEx(0.2);
    public TimerEx stateTimerFullSec = new TimerEx(1);
    public TimerEx stateTimerHalfSec = new TimerEx(0.5);

    LoadHardwareClass Robot = new LoadHardwareClass(this);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize all hardware of the robot
        Robot.init(new Pose(72, 72, 90));

        if (!Turret.zeroed){
            while (!isStopRequested() && Robot.turret.zeroTurret()){
                Robot.sleep(0);
            }
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            // Pass the joystick positions to our mecanum drive controller
            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y/3,
                    gamepad1.left_stick_x/2.5,
                    gamepad1.right_stick_x/3,
                    true
            );

            if (Math.abs(gamepad1.right_stick_y) > 0.1){
                Robot.intake.setMode(Intake.intakeMode.ON, Intake.intakeMode.ON);
            }else{
                Robot.intake.setMode(Intake.intakeMode.OFF, Intake.intakeMode.OFF);
            }

            if (gamepad1.yWasPressed()){
                if (targetRPM == 0){
                    targetRPM = 1000;
                }else{
                    targetRPM = 0;
                }
            }

            //Shoot (B Button Press)
            // Increment the shooting state
            if (gamepad1.bWasPressed() && shootingState < 1 && Robot.turret.getFlywheelRPM() > 900) {
                shootingState++;
            }
            if (gamepad1.xWasPressed()){
                shootingState = 4;
            }
            switch (shootingState) {
                case 0:
                    telemetry.addData("Shooting State", "OFF");
                    break;
                case 1:
                    Robot.turret.setFlywheelState(Turret.flywheelState.ON);
                    if (Robot.turret.getGate() == Turret.gatestate.CLOSED){
                        stateTimerFifthSec.restart();
                        stateTimerFifthSec.start();
                    }
                    Robot.turret.setGateState(Turret.gatestate.OPEN);
                    telemetry.addData("Shooting State", "GATE OPENING");
                    if (stateTimerFifthSec.isDone()){
                        shootingState = 2;
                        stateTimerHalfSec.restart();
                        stateTimerHalfSec.start();
                    }
                    break;
                case 2:
                    Robot.intake.setMode(Intake.intakeMode.ON, Intake.intakeMode.ON);
                    telemetry.addData("Shooting State", "INTAKE_NOINTAKE FIRST TWO");
                    if (stateTimerHalfSec.isDone() && Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState()){
                        shootingState = 3;
                        stateTimerHalfSec.restart();
                        stateTimerHalfSec.start();
                    }
                    break;
                case 3:
                    Robot.intake.setMode(Intake.intakeMode.OFF, Intake.intakeMode.ON);
                    Robot.intake.setTransfer(Intake.transferState.UP);
                    telemetry.addData("Shooting State", "INTAKE_NOINTAKE FINAL");
                    if (stateTimerHalfSec.isDone()) {
                        shootingState = 4;
                    }
                    break;
                case 4:
                    Robot.turret.setGateState(Turret.gatestate.CLOSED);
                    Robot.intake.setMode(Intake.intakeMode.OFF, Intake.intakeMode.OFF);
                    Robot.intake.setTransfer(Intake.transferState.DOWN);
                    telemetry.addData("Shooting State", "RESET");
                    shootingState = 0;
            }

            Robot.turret.flywheel.setRPM(targetRPM);
            Robot.turret.rotation.setAngle(90);
            Robot.turret.setHood(0);

            telemetry.addData("Flywheel RPM", Robot.turret.getFlywheelRPM());

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
    }
}
