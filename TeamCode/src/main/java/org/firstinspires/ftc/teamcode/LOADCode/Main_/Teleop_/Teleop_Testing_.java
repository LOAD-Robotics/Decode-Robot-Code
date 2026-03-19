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
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Devices;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

import java.util.concurrent.TimeUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
@TeleOp(name="Teleop_Testing_", group="TeleOp")
public class Teleop_Testing_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private final TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private final Telemetry ftcTelemetry = super.telemetry;
    private final JoinedTelemetry telemetry = new JoinedTelemetry(ftcTelemetry, panelsTelemetry);

    public Devices.Limelight3AClass limelight = new Devices.Limelight3AClass();
    public LoadHardwareClass Robot = new LoadHardwareClass(this);
    public Pedro_Paths paths = new Pedro_Paths();

    public static PIDCoefficients cameraCoefficients = new PIDCoefficients(0.05, 0.00000000001, 0.0000001);
    private static PIDCoefficients oldCameraCoefficients = new PIDCoefficients(0, 0, 0);
    private final PIDCoefficients turretCoefficients = new PIDCoefficients(0.022, 0.0000000002, 0.0015); // 223RPM Motor

    public ControlSystem pid = ControlSystem.builder().posPid(cameraCoefficients).build();

    @Override
    public void runOpMode() {

        limelight.init(this);
        Robot.init(paths.farStart);
        paths.buildPaths(Robot.drivetrain.follower);
        selectedAlliance = LoadHardwareClass.Alliance.BLUE;

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (oldCameraCoefficients != cameraCoefficients){
                oldCameraCoefficients = cameraCoefficients;
                pid = ControlSystem.builder().posPid(cameraCoefficients).build();
            }

            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x/2,
                    true
            );

            limelight.updateResult();
            pid.setGoal(new KineticState(0, -Math.toDegrees(Robot.drivetrain.follower.getAngularVelocity())));
            double maxPower = 1;
            double minPower = -1;
            double power = 0;
            if (Robot.turret.rotation.getAngleAbsolute() > 360){
                maxPower = 0;
            }
            if (Robot.turret.rotation.getAngleAbsolute() < 0){
                minPower = 0;
            }

            if (limelight.result != null && limelight.result.isValid()){
                power = pid.calculate(
                        new KineticState(
                                limelight.result.getTx(),
                                Robot.turret.rotation.getDegreesPerSecond()
                        )
                );
                Robot.turret.rotation.setPower(Math.min(Math.max(power, minPower), maxPower));
            }else{
                Robot.turret.updateAimbot(true, false, 0);
            }

            telemetry.addData("Result Valid", limelight.result.isValid());
            telemetry.addData("Turret Error", limelight.result.getTx());

            if (gamepad1.bWasPressed()){
                limelight.setPipeline(0);
                selectedAlliance = LoadHardwareClass.Alliance.BLUE;
            }else if (gamepad1.xWasPressed()){
                limelight.setPipeline(1);
                selectedAlliance = LoadHardwareClass.Alliance.RED;
            }

            if (limelight.getPipeline() == 0){
                telemetry.addData("Current Tag Target", "RED");
            }else if (limelight.getPipeline() == 1){
                telemetry.addData("Current Tag Target", "BLUE");
            }

            telemetry.addData("Loop Time (ms)", loopTimer.time(TimeUnit.MILLISECONDS));
            telemetry.update();
            loopTimer.reset();
        }
    }
}
