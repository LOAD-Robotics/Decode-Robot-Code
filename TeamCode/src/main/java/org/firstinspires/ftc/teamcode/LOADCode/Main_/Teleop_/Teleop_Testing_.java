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
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    public static final double llOffset = 6.496063; // INCHES
    public static final double turretOffset = -1.094488;

    int turretTarget = 90;

    @Override
    public void runOpMode() {

        Robot.init(new Pose(72, 24, Math.toRadians(90)));
        //Robot.init(paths.farStart);
        Drawing.init();

        Turret.zeroed = false;

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

            Robot.turret.limelight.setPipeline(2);

            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x / 2,
                    true
            );
            Robot.turret.updatePIDs();
            Robot.turret.rotation.setAngle(turretTarget);
            int change = 10;
            if (gamepad1.dpadLeftWasPressed()){
                turretTarget += change;
            }else if (gamepad1.dpadRightWasPressed()){
                turretTarget -= change;
            }

            Robot.turret.limelight.updateResult(Robot.drivetrain.follower.getHeading()
                +  Math.toRadians(Robot.turret.rotation.getAngle() - 90));
            LLResult result = Robot.turret.limelight.result;

            Pose2D botPose = null;
            if (result != null && result.isValid()){
                Pose3D pose = result.getBotpose_MT2();
                botPose = new Pose2D(DistanceUnit.METER, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES, pose.getOrientation().getYaw());
            }
            Pose pedroPose = null;
            if (botPose != null) {
                pedroPose = PoseConverter.pose2DToPose(botPose, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                double turretOffsetX = llOffset * Math.cos(pedroPose.getHeading());
                double turretOffsetY = llOffset * Math.sin(pedroPose.getHeading());

                pedroPose = new Pose(pedroPose.getX() - turretOffsetX,
                        pedroPose.getY()- turretOffsetY,
                        Robot.drivetrain.follower.getHeading());

                double robotCenterOffsetX = turretOffset * Math.cos(pedroPose.getHeading());
                double robotCenterOffsetY = turretOffset * Math.sin(pedroPose.getHeading());

                pedroPose = new Pose(pedroPose.getX() - robotCenterOffsetX,
                        pedroPose.getY()- robotCenterOffsetY,
                        pedroPose.getHeading());

                Drawing.drawRobot(pedroPose);


                telemetry.addData("MT2 PosX", botPose.getX(DistanceUnit.INCH));
                telemetry.addData("MT2 PosY", botPose.getY(DistanceUnit.INCH));
                telemetry.addData("MT2 Heading", botPose.getHeading(AngleUnit.DEGREES));

                telemetry.addData("Pedro MT2 PosX", pedroPose.getX());
                telemetry.addData("Pedro MT2 PosY", pedroPose.getY());
                telemetry.addData("Pedro MT2 Heading", Math.toDegrees(pedroPose.getHeading()));
            }

            Drawing.drawRobot(Robot.drivetrain.follower.getPose(), Drawing.turretLook);
            Drawing.sendPacket();

            telemetry.addData("Loop Time (ms)", loopTimer.time(TimeUnit.MILLISECONDS));
            telemetry.update();
            loopTimer.reset();
        }
    }
}
