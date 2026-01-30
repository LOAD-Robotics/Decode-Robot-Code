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
    public Pose hpPreload = new Pose(136, 8, -90);
    public Pose rampIntake = new Pose(133, 55, 65);
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
    public Pose openGate = new Pose(128, 68, 90);

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
    /**
     * <hr></br>
     * <h4>Mid Preload Pose</h4>
     */
    public PathChain midPreload_to_nearShoot, midPreload_to_midShoot, midPreload_to_farShoot;
    /**
     * <hr></br>
     * <h4>Far Preload Pose</h4>
     */
    public PathChain farPreload_to_nearShoot, farPreload_to_midShoot, farPreload_to_farShoot;
    /**
     * <hr></br>
     * <h4>Human Player Preload Pose</h4>
     */
    /**
     * <hr></br>
     * <h4>Ramp Intake Pose</h4>
     */
    /**
     * <hr></br>
     * <h4>Human Player Random Intake Pose</h4>
     */
    /**
     * <hr></br>
     * <h4>Near Shooting Pose</h4>
     */
    public PathChain nearShoot_to_nearPreload, nearShoot_to_midPreload, nearShoot_to_farPreload;
    public PathChain nearShoot_to_hpPreload, nearShoot_to_rampIntake, nearShoot_to_hpIntake;
    public PathChain nearShoot_to_nearLeave, nearShoot_to_midLeave;
    public PathChain nearShoot_to_openGate;
    /**
     * <hr></br>
     * <h4>Mid Shooting Pose</h4>
     */
    public PathChain midShoot_to_nearPreload, midShoot_to_midPreload, midShoot_to_farPreload;
    public PathChain midShoot_to_hpPreload, midShoot_to_rampIntake, midShoot_to_hpIntake;
    public PathChain midShoot_to_nearLeave, midShoot_to_midLeave;
    public PathChain midShoot_to_openGate;
    /**
     * <hr></br>
     * <h4>Far Shooting Pose</h4>
     */
    public PathChain farShoot_to_nearPreload, farShoot_to_midPreload, farShoot_to_farPreload;
    public PathChain farShoot_to_hpPreload, farShoot_to_rampIntake, farShoot_to_hpIntake;
    public PathChain farShoot_to_midLeave, farShoot_to_farLeave;
    public PathChain farShoot_to_openGate;
    /**
     * <hr></br>
     * <h4>Open Gate Pose</h4>
     */
    public PathChain openGate_to_nearShoot, openGate_to_midShoot, openGate_to_farShoot;
    public PathChain openGate_to_rampIntake;




    public Pose autoMirror(Pose pose, LoadHardwareClass.Alliance alliance){
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
                        new Pose(80.000, 83.500),
                        farShoot
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload2ToShootings(){
        midPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        new Pose(65,59.5),
                        nearShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        midPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        new Pose(65,59.5),
                        midShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), midShoot.getHeading())
                .build();
        midPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPreload,
                        new Pose(90.000, 59.500),
                        farShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload3ToShootings(){
        farPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farPreload,
                        new Pose(75,30),
                        new Pose(80,100),
                        nearShoot
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        farPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farPreload,
                        new Pose(85,32),
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
                        new Pose(60, 80),
                        nearPreload
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), nearPreload.getHeading())
                .build();
        nearShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearShoot,
                        new Pose(60, 55),
                        midPreload
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), midPreload.getHeading())
                .build();
        nearShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        nearShoot,
                        new Pose(60, 27),
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
                        new Pose(75, 56),
                        midPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), midPreload.getHeading())
                .build();
        midShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midShoot,
                        new Pose(68, 30),
                        farPreload
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting3ToPreloads(){
        farShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        new Pose(73, 88),
                        nearPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), nearPreload.getHeading())
                .build();
        farShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        new Pose(78, 62),
                        midPreload
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), midPreload.getHeading())
                .build();
        farShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShoot,
                        new Pose(82.5, 35),
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


    /**
     * Builds all the paths, mirroring them to the other side of the field if necessary
     */
    public void buildPaths(LoadHardwareClass.Alliance Alliance, Follower follow){
        follower = follow;

        nearStart = autoMirror(nearStart, Alliance);
        farStart = autoMirror(farStart, Alliance);
        nearPreload = autoMirror(nearPreload, Alliance);
        midPreload = autoMirror(midPreload, Alliance);
        farPreload = autoMirror(farPreload, Alliance);
        nearShoot = autoMirror(nearShoot, Alliance);
        midShoot = autoMirror(midShoot, Alliance);
        farShoot = autoMirror(farShoot, Alliance);
        noTurretMidShoot = autoMirror(noTurretMidShoot, Alliance);
        noTurretFarShoot = autoMirror(noTurretFarShoot, Alliance);
        nearLeave = autoMirror(nearLeave, Alliance);
        midLeave = autoMirror(midLeave, Alliance);
        farLeave = autoMirror(farLeave, Alliance);


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
    }
}
