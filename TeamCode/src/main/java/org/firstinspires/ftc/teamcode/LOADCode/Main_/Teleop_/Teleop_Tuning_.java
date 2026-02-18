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

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@Configurable
@TeleOp(name="Teleop_Tuning_", group="TeleOp")
public class Teleop_Tuning_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double hoodOffset = 0;

    double initial = 0;

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(88.5,7.8, Math.toRadians(90));

    @Override
    public void runOpMode() {

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);
        Pedro_Paths paths = new Pedro_Paths();
        // Initialize all hardware of the robot
        Robot.init(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Robot.turret.setGateState(Turret.gatestate.CLOSED);

        if (!Turret.zeroed){
            while (!isStopRequested() && Robot.turret.zeroTurret()){
                sleep(0);
            }
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );
            if (gamepad2.yWasPressed()) {
                if (Robot.turret.flywheelMode == Turret.flywheelState.OFF) {
                    Robot.turret.setFlywheelState(Turret.flywheelState.ON);
                } else {
                    Robot.turret.setFlywheelState(Turret.flywheelState.OFF);
                }
            }
            Robot.turret.updateFlywheel();

            if (Math.abs(gamepad2.left_stick_y) < 0.1){
                Robot.intake.setMode(Intake.intakeMode.INTAKING);
            }else{
                Robot.intake.setMode(Intake.intakeMode.OFF);
            }
            if (gamepad2.dpadUpWasPressed()) hoodOffset += 10;
            if (gamepad2.dpadDownWasPressed()) hoodOffset -= 10;
            Robot.turret.updateAimbot(false, true, hoodOffset);

            telemetry.addData("Aimbot Hood Angle", Robot.turret.getHood());
            telemetry.addData("Hood Offset", hoodOffset);
            telemetry.addData("Distance From Goal", Robot.drivetrain.distanceFromGoal());


            // System-related Telemetry
            telemetry.addLine();
            telemetry.addLine("SYSTEM DATA");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Loop Time", loopTimer.toString());
            telemetry.addData("Version: ", "12/26/25");
            telemetry.update();
            panelsTelemetry.update();
            loopTimer.reset();
            Robot.turret.updatePIDs();
        }
    }
}
