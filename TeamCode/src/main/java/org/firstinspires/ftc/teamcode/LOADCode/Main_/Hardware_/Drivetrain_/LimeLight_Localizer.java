package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drawing;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class LimeLight_Localizer {
    private LoadHardwareClass Robot = null;

    public static final double llOffset = 6.496063; // INCHES
    public static final double turretOffset = -1.094488;
    public static double varianceMult = 16;

    public void init(LoadHardwareClass robot){
        this.Robot = robot;
    }

    public Pose updateLLPose(){
        if (Robot.turret.limelight.getPipeline() != 2){
            Robot.turret.limelight.setPipeline(2);
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

            long timestampNanos = System.nanoTime() - result.getStaleness() * 1_000_000L;

            double[] measurementStdDevs = result.getStddevMt2();

            double stdX_in = measurementStdDevs[0] * 39.3701;
            double stdY_in = measurementStdDevs[1] * 39.3701;
            double stdYaw_rad = Math.toRadians(measurementStdDevs[5]);

            Pose measurementVariance = new Pose(
                    stdX_in * stdX_in * varianceMult,
                    stdY_in * stdY_in * varianceMult,
                    stdYaw_rad * stdYaw_rad * varianceMult
            );


            Constants.getFusionLocalizer().addMeasurement(
                    pedroPose,
                    timestampNanos,
                    measurementVariance
            );
            return pedroPose;
        }
        return new Pose(0,0,0);
    }
}
