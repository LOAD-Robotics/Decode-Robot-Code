package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

public class Pedro_Paths {
    // The variable to store PedroPathing's follower object for path building
    private Follower follower;
    private LoadHardwareClass.Alliance alliance = LoadHardwareClass.selectedAlliance;

    /**
     * Define primary poses to be used in paths
      */
    // Start Poses
    public Pose nearStart = new Pose(118, 132, Math.toRadians(306));
    public Pose farStart = new Pose(88, 7.4, Math.toRadians(90));
    // Preload Poses
    public Pose nearPreload = new Pose(124.000, 83.500, Math.toRadians(0));
    public Pose midPreload = new Pose(130.000, 59.500, Math.toRadians(0));
    public Pose farPreload = new Pose(130.000, 35.500, Math.toRadians(0));
    public Pose hpPreload = new Pose(135, 8, -90);
    public Pose rampIntake = new Pose(135, 44, 80);
    public Pose hpIntake = null;
    // Shooting Poses
    public Pose nearShoot = new Pose(115, 120, Math.toRadians(-35));
    public Pose midShoot = new Pose(85, 85, Math.toRadians(-15));
    public Pose farShoot = new Pose(85, 15, Math.toRadians(60));
    public Pose noTurretMidShoot = new Pose(85, 85, Math.toRadians(45));
    public Pose noTurretFarShoot = new Pose(85, 15, Math.toRadians(67.3));
    // Leave Poses
    public Pose nearLeave = new Pose(90,120, Math.toRadians(90));
    public Pose midLeave = new Pose(95,55, Math.toRadians(90));
    public Pose farLeave = new Pose(105,20, Math.toRadians(90));
    // Open Gate Pose
    public Pose openGateBasic = new Pose(127.5, 72, Math.toRadians(90));
    public Pose openGateBasicReversed = new Pose(127.5, 72, Math.toRadians(-90));
    public Pose openGateIntakeGate = new Pose(131, 61.5, Math.toRadians(20));
    public Pose openGateIntakeRamp = new Pose(133, 55, Math.toRadians(40));

    /**
     * <h4>Define all path variables.</h4></br>
     * Comments indicate the start pose of the paths
     */
    /**
     * <hr></br>
     * <h4>Near Start Pose</h4>
     */
    public PathChain nearStart_to_midShoot, nearStart_to_nearShoot;
    /**
     * <hr></br>
     * <h4>Far Start Pose</h4>
     */
    public PathChain farStart_to_midShoot, farStart_to_farShoot;
    /**
     * <hr></br>
     * <h4>Near Preload Pose</h4>
     */
    public PathChain nearPreload_to_nearShoot, nearPreload_to_midShoot, nearPreload_to_farShoot;
    public PathChain nearPreload_to_openGateBasic;
    /**
     * <hr></br>
     * <h4>Mid Preload Pose</h4>
     */
    public PathChain midPreload_to_nearShoot, midPreload_to_midShoot, midPreload_to_farShoot;
    public PathChain midPreload_to_openGateBasic;
    /**
     * <hr></br>
     * <h4>Far Preload Pose</h4>
     */
    public PathChain farPreload_to_nearShoot, farPreload_to_midShoot, farPreload_to_farShoot;
    /**
     * <hr></br>
     * <h4>Human Player Preload Pose</h4>
     */
    public PathChain hpPreload_to_nearShoot, hpPreload_to_midShoot, hpPreload_to_farShoot;
    /**
     * <hr></br>
     * <h4>Ramp Intake Pose</h4>
     */
    public PathChain rampIntake_to_nearShoot, rampIntake_to_midShoot, rampIntake_to_farShoot;
    /**
     * <hr></br>
     * <h4>Human Player Random Intake Pose</h4>
     */
    /**
     * <hr></br>
     * <h4>Near Shooting Pose</h4>
     */
    public PathChain nearShoot_to_nearPreload, nearShoot_to_midPreload, nearShoot_to_farPreload;
    public PathChain nearShoot_to_hpPreload, nearShoot_to_hpIntake;
    public PathChain nearShoot_to_nearLeave, nearShoot_to_midLeave;
    public PathChain nearShoot_to_openGateIntake, nearShoot_to_openGateBasic;
    /**
     * <hr></br>
     * <h4>Mid Shooting Pose</h4>
     */
    public PathChain midShoot_to_nearPreload, midShoot_to_midPreload, midShoot_to_farPreload;
    public PathChain midShoot_to_hpPreload, midShoot_to_hpIntake;
    public PathChain midShoot_to_nearLeave, midShoot_to_midLeave;
    public PathChain midShoot_to_openGateIntake, midShoot_to_openGateBasic;
    public PathChain midShoot_to_rampIntake;
    /**
     * <hr></br>
     * <h4>Far Shooting Pose</h4>
     */
    public PathChain farShoot_to_nearPreload, farShoot_to_midPreload, farShoot_to_farPreload;
    public PathChain farShoot_to_hpPreload, farShoot_to_hpIntake;
    public PathChain farShoot_to_midLeave, farShoot_to_farLeave;
    public PathChain farShoot_to_openGateIntake, farShoot_to_openGateBasic;
    public PathChain farShoot_to_rampIntake;
    /**
     * <hr></br>
     * <h4>Open Gate Basic Pose (No Intake)</h4>
     */
    public PathChain openGateBasic_to_nearShoot, openGateBasic_to_midShoot, openGateBasic_to_farShoot;
    /**
     * <hr></br>
     * <h4>Open Gate Basic Pose (Intake)</h4>
     */
    public PathChain openGateIntake_to_nearShoot, openGateIntake_to_midShoot, openGateIntake_to_farShoot;





    public Pose autoMirror(Pose pose){
        if (alliance == LoadHardwareClass.Alliance.BLUE){
            return new Pose(
                    144 - pose.getX(),
                    pose.getY(),
                    mirrorHeading(pose.getHeading(), alliance)
            );
        }else{
            return pose;
        }
    }
    private double mirrorHeading(double heading, LoadHardwareClass.Alliance alliance){
        if (alliance == LoadHardwareClass.Alliance.BLUE){
            final double v = Math.toDegrees(Math.atan2(Math.sin(heading), Math.cos(heading)));
            if (Math.cos(heading) >= 0){
                return Math.toRadians((180 - v));
            }else{
                return Math.toRadians((360 - v)%360);
            }
        }else{
            return heading;
        }
    }

    public void buildStart1ToShootings(){
        nearStart_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearStart,
                        nearShoot
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), nearShoot.getHeading())
                .build();
        nearStart_to_midShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearStart,
                        midShoot
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), midShoot.getHeading())
                .build();
    }
    public void buildStart2ToShootings(){
        farStart_to_midShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        farStart,
                        midShoot
                ))
                .setLinearHeadingInterpolation(farStart.getHeading(), midShoot.getHeading())
                .build();
        farStart_to_farShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        farStart,
                        farShoot
                ))
                .setLinearHeadingInterpolation(farStart.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload1ToShootings(){
        nearPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearPreload,
                        nearShoot
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), nearShoot.getHeading())
                .build();
        nearPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearPreload,
                        midShoot
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), midShoot.getHeading())
                .build();
        nearPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearPreload,
                        autoMirror(new Pose(80.000, 83.500)),
                        farShoot
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload2ToShootings(){
        midPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        autoMirror(new Pose(65,59.5)),
                        nearShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        midPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        autoMirror(new Pose(65,59.5)),
                        midShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), midShoot.getHeading())
                .build();
        midPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        autoMirror(new Pose(90.000, 59.500)),
                        farShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload3ToShootings(){
        farPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farPreload,
                        autoMirror(new Pose(75,30)),
                        autoMirror(new Pose(80,100)),
                        nearShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        farPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farPreload,
                        autoMirror(new Pose(90,41)),
                        midShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), midShoot.getHeading())
                .build();
        farPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        farPreload,
                        farShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildShooting1ToPreloads(){
        nearShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearShoot,
                        autoMirror(new Pose(60, 80)),
                        nearPreload
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), nearPreload.getHeading())
                .build();
        nearShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearShoot,
                        autoMirror(new Pose(60, 55)),
                        midPreload
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), midPreload.getHeading())
                .build();
        nearShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearShoot,
                        autoMirror(new Pose(60, 27)),
                        farPreload
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting2ToPreloads(){
        midShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        midShoot,
                        nearPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), nearPreload.getHeading())
                .build();
        midShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        autoMirror(new Pose(75, 56)),
                        midPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), midPreload.getHeading())
                .build();
        midShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        autoMirror(new Pose(68, 30)),
                        farPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting3ToPreloads(){
        farShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(73, 88)),
                        nearPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), nearPreload.getHeading())
                .build();
        farShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(78, 62)),
                        midPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), midPreload.getHeading())
                .build();
        farShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(82.5, 35)),
                        farPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting1ToLeaves(){
        nearShoot_to_nearLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearShoot,
                        nearLeave
                ))
                .setConstantHeadingInterpolation(nearShoot.getHeading())
                .build();
        nearShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        nearShoot,
                        midLeave
                ))
                .setConstantHeadingInterpolation(nearShoot.getHeading())
                .build();
    }
    public void buildShooting2ToLeaves(){
        midShoot_to_nearLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        midShoot,
                        nearLeave
                ))
                .setConstantHeadingInterpolation(midShoot.getHeading())
                .build();
        midShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        midShoot,
                        midLeave
                ))
                .setConstantHeadingInterpolation(midShoot.getHeading())
                .build();
    }
    public void buildShooting3ToLeaves(){
        farShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        farShoot,
                        midLeave
                ))
                .setConstantHeadingInterpolation(farShoot.getHeading())
                .build();
        farShoot_to_farLeave = follower.pathBuilder()
                .addPath(new BezierLine(
                        farShoot,
                        farLeave
                ))
                .setConstantHeadingInterpolation(farShoot.getHeading())
                .build();
    }
    public void buildMidShootToOpenGates(){
        midShoot_to_openGateBasic = follower.pathBuilder()
                .addPath(new BezierCurve(
                    midShoot,
                    autoMirror(new Pose(94, 71)),
                    openGateBasic
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), openGateBasic.getHeading())
                .build();
        midShoot_to_openGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        autoMirror(new Pose(96, 66.5)),
                        openGateIntakeGate
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), openGateIntakeRamp.getHeading())
                .addPath(new BezierLine(
                        openGateIntakeGate,
                        openGateIntakeRamp
                ))
                .setLinearHeadingInterpolation(openGateIntakeGate.getHeading(), openGateIntakeRamp.getHeading())
                .build();
    }
    public void buildFarShootToOpenGates(){
        farShoot_to_openGateBasic = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(94, 71)),
                        openGateBasic
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), openGateBasic.getHeading())
                .build();
        farShoot_to_openGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(96, 66.5)),
                        openGateIntakeGate
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), openGateIntakeRamp.getHeading())
                .addPath(new BezierLine(
                        openGateIntakeGate,
                        openGateIntakeRamp
                ))
                .setLinearHeadingInterpolation(openGateIntakeGate.getHeading(), openGateIntakeRamp.getHeading())
                .build();
    }
    public void buildOpenGateIntakeToShootings(){
        openGateIntake_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        openGateIntakeRamp,
                        autoMirror(new Pose(94, 71)),
                        farShoot
                ))
                .setLinearHeadingInterpolation(openGateIntakeRamp.getHeading(), farShoot.getHeading())
                .build();
        openGateIntake_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        openGateIntakeRamp,
                        autoMirror(new Pose(97, 60)),
                        midShoot
                ))
                .setLinearHeadingInterpolation(openGateIntakeRamp.getHeading(), midShoot.getHeading())
                .build();
    }
    public void buildOpenGateBasicToShootings(){
        openGateBasic_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        openGateIntakeRamp,
                        autoMirror(new Pose(94, 71)),
                        farShoot
                ))
                .setLinearHeadingInterpolation(openGateBasic.getHeading(), farShoot.getHeading())
                .setVelocityConstraint(10)
                .build();
        openGateBasic_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        openGateBasic,
                        autoMirror(new Pose(97, 67)),
                        midShoot
                ))
                .setLinearHeadingInterpolation(openGateBasic.getHeading(), midShoot.getHeading())
                .build();
    }
    public void buildPreloadsToOpenGateBasic(){
        nearPreload_to_openGateBasic = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearPreload,
                        autoMirror(new Pose(115, 75)),
                        openGateBasic
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), openGateBasic.getHeading())
                .build();
        midPreload_to_openGateBasic = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        autoMirror(new Pose(110, 59)),
                        openGateBasicReversed
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), openGateBasicReversed.getHeading())
                .build();
    }
    public void buildShootingsToHPPreload(){
        farShoot_to_hpPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                    farShoot,
                    autoMirror(new Pose(136, 30)),
                    hpPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), hpPreload.getHeading())
                .build();
        midShoot_to_hpPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        autoMirror(new Pose(87, 28)),
                        autoMirror(new Pose(138, 42)),
                        hpPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), hpPreload.getHeading())
                .build();
    }
    public void buildShootingsToRampIntake(){
        midShoot_to_rampIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        autoMirror(new Pose(85, 40)),
                        autoMirror(new Pose(137, 2)),
                        autoMirror(new Pose(135, 30, rampIntake.getHeading()))
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), rampIntake.getHeading())
                .addPath(new BezierLine(
                        autoMirror(new Pose(135, 30, rampIntake.getHeading())),
                        rampIntake
                ))
                .setConstantHeadingInterpolation(rampIntake.getHeading())
                .build();
        farShoot_to_rampIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        autoMirror(new Pose(137, 2)),
                        autoMirror(new Pose(135, 30, rampIntake.getHeading()))
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), rampIntake.getHeading())
                .addPath(new BezierLine(
                        autoMirror(new Pose(135, 30, rampIntake.getHeading())),
                        rampIntake
                ))
                .setConstantHeadingInterpolation(rampIntake.getHeading())
                .build();
    }
    public void buildHPPreloadToShootings(){
        hpPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        hpPreload,
                        farShoot
                ))
                .setLinearHeadingInterpolation(hpPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildRampIntakeToShootings(){
        rampIntake_to_farShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        rampIntake,
                        farShoot
                ))
                .setLinearHeadingInterpolation(rampIntake.getHeading(), farShoot.getHeading())
                .build();
    }


    /**
     * Builds all the paths, mirroring them to the other side of the field if necessary
     */
    public void buildPaths(Follower follow){
        follower = follow;
        alliance = LoadHardwareClass.selectedAlliance;

        nearStart = autoMirror(nearStart);
        farStart = autoMirror(farStart);

        nearPreload = autoMirror(nearPreload);
        midPreload = autoMirror(midPreload);
        farPreload = autoMirror(farPreload);
        hpPreload = autoMirror(hpPreload);

        nearShoot = autoMirror(nearShoot);
        midShoot = autoMirror(midShoot);
        farShoot = autoMirror(farShoot);
        noTurretMidShoot = autoMirror(noTurretMidShoot);
        noTurretFarShoot = autoMirror(noTurretFarShoot);

        nearLeave = autoMirror(nearLeave);
        midLeave = autoMirror(midLeave);
        farLeave = autoMirror(farLeave);

        openGateBasic = autoMirror(openGateBasic);
        openGateBasicReversed = autoMirror(openGateBasicReversed);
        openGateIntakeGate = autoMirror(openGateIntakeGate);
        openGateIntakeRamp = autoMirror(openGateIntakeRamp);

        /// All paths are for the RED side of the field. they will be mirrored if necessary.
        // Paths going from each start position to each of the shooting positions.
        buildStart1ToShootings();
        buildStart2ToShootings();
        // Paths going from each preload to each shooting position
        buildPreload1ToShootings();
        buildPreload2ToShootings();
        buildPreload3ToShootings();
        // Paths going from each shooting position to each preload
        buildShooting1ToPreloads();
        buildShooting2ToPreloads();
        buildShooting3ToPreloads();
        // Paths going from each shooting position to the leave positions.
        buildShooting1ToLeaves();
        buildShooting2ToLeaves();
        buildShooting3ToLeaves();
        // Paths going from shooting positions to open gate positions
        buildFarShootToOpenGates();
        buildMidShootToOpenGates();
        buildOpenGateIntakeToShootings();
        buildOpenGateBasicToShootings();
        buildPreloadsToOpenGateBasic();
        buildShootingsToHPPreload();
        buildShootingsToRampIntake();
        buildRampIntakeToShootings();
        buildHPPreloadToShootings();
    }
}
