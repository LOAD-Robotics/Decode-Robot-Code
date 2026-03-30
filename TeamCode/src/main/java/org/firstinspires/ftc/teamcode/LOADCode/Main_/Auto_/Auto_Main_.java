package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.OFF;
import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.ON;
import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.isLiftAttached;
import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drawing;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants_NoLift;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends NextFTCOpMode {

    TimerEx timer25Sec = new TimerEx(25);
    // Variable to store the selected auto program
    Auto selectedAuto = null;
    // Create the prompter object for selecting Alliance and Auto
    Prompter prompter = null;
    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);
    // Create a Paths object for accessing modular auto paths
    Pedro_Paths paths = new Pedro_Paths();
    // Create a Commands object for auto creation
    Commands Commands = new Commands(Robot);

    // Auto parameter variables
    private boolean turretOn = true;

    @SuppressWarnings("unused")
    public Auto_Main_() {
        addComponents(
                new PedroComponent(Constants_NoLift::createFollower)
        );
    }

    @Override
    public void onInit() {
        prompter = new Prompter(this);
        prompter.prompt("lift", new OptionPrompt<>("Is Lift Attached?",
                LoadHardwareClass.IsLiftAttached.NO,
                LoadHardwareClass.IsLiftAttached.YES
        ));
        prompter.prompt("alliance",
                new OptionPrompt<>("Select Alliance",
                        LoadHardwareClass.Alliance.RED,
                        LoadHardwareClass.Alliance.BLUE
                ));
        prompter.prompt("auto",
                new OptionPrompt<>("Select Auto",
                        new testAuto(),
                        new MOE_365_FAR(),
                        new Near_15Ball(),
                        new Near_15Ball2(),
                        new Near_12Ball(),
                        new Near_9Ball(),
                        new Far_12Ball(),
                        new Far_9Ball()
                ));
        prompter.onComplete(() -> {
            isLiftAttached = prompter.get("lift");
            selectedAlliance = prompter.get("alliance");
            selectedAuto = prompter.get("auto");
            telemetry.update();
            // Build paths
            paths.buildPaths(follower());
            // Initialize all hardware of the robot
            Robot.init(selectedAuto.getStartPose(), follower());
            if (!Turret.zeroed){
                while (!isStopRequested() && Robot.turret.zeroTurret()){
                    Robot.sleep(0);
                }
            }
            telemetry.addData("Selection", "Complete");
            telemetry.addData("Alliance", selectedAlliance.toString());
            telemetry.addData("Auto", selectedAuto);
            telemetry.update();
        });
    }

    @Override
    public void onWaitForStart() {
        prompter.run();
    }

    @Override
    public void onStartButtonPressed() {
        Robot.lights.setSolidAllianceDisplay(selectedAlliance);
        // Schedule the proper auto
        selectedAuto.runAuto();
        turretOn = selectedAuto.getTurretEnabled();
        timer25Sec.restart();

        // Indicate that initialization is done
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Running Auto", selectedAuto.toString());
        telemetry.addData("Alliance", selectedAlliance);
        Robot.turret.updateAimbot(turretOn, true, selectedAuto.getHoodOffset());
        Robot.turret.updateFlywheel(0);
        MecanumDrivetrainClass.robotPose = Robot.drivetrain.follower.getPose();
        telemetry.update();
        Drawing.drawRobot(Robot.drivetrain.follower.getPose());
        Drawing.drawPoseHistory(Robot.drivetrain.follower.getPoseHistory());
        Drawing.sendPacket();
    }

    @Override
    public void onStop(){
        Robot.lights.setStripRainbow();
        Robot.drivetrain.follower.holdPoint(Robot.drivetrain.follower.getPose());
        MecanumDrivetrainClass.robotPose = Robot.drivetrain.follower.getPose();
    }

    /**
     * This class serves as a template for all auto programs. </br>
     * The methods runAuto() and ToString() must be overridden for each auto.
     */
    abstract static class Auto{
        /**
         * @return The start pose of the robot for this auto.
         */
        abstract Pose getStartPose();

        /**
         * @return The offset of the hood for this auto
         */
        abstract double getHoodOffset();

        /**
         * @return A boolean indicating whether the turret is enabled.
         */
        abstract boolean getTurretEnabled();

        /** Override this to schedule the auto command*/
        abstract void runAuto();
        /** Override this to rename the auto*/
        @NonNull
        @Override
        public abstract String toString();
    }

    private class Far_9Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.farStart;
        }

        @Override
        double getHoodOffset() {
            return 10;
        }

        @Override
        public boolean getTurretEnabled(){
            return true;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.farShoot_to_hpPreload, true, 1),
                    Commands.runPath(paths.hpPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Far 9 Ball";}
    }
    private class Far_12Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.farStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }
        @Override
        double getHoodOffset() {
            return 10;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.farShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.farShoot_to_hpPreload, true, 1),
                    Commands.runPath(paths.hpPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Far 12 Ball";}
    }

    private class Near_9Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.nearStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near 9 Ball";}
    }
    private class Near_12Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.nearStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near 12 Ball";}
    }
    private class Near_15Ball extends Auto{
        @Override
        Pose getStartPose() {
            return paths.nearStart;
        }
        @Override
        boolean getTurretEnabled() {
            return true;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        void runAuto() {
            new SequentialGroup(
                    new ParallelRaceGroup(
                            new Delay(29),
                            new SequentialGroup(
                                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                                    Commands.setIntakeMode(OFF, ON),
                                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.midShoot_to_openGateIntake, true, 1),
                                    Commands.waitForArtifacts(),
                                    Commands.runPath(paths.openGateIntake_to_midShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.midShoot_to_farPreload, true, 1),
                                    Commands.setIntakeMode(OFF, ON),
                                    Commands.runPath(paths.farPreload_to_midShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                                    Commands.setIntakeMode(OFF, ON),
                                    Commands.runPath(paths.nearPreload_to_midShoot, false, 1),
                                    Commands.shootBalls(),
                                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
                            )
                    ),
                    Commands.runPath(
                            Robot.drivetrain.follower.pathBuilder().addPath(
                                    new BezierLine(
                                            Robot.drivetrain.follower.getPose(),
                                            paths.nearLeave
                                    )
                            ).setLinearHeadingInterpolation(
                                    Robot.drivetrain.follower.getPose().getHeading(),
                                    paths.farLeave.getHeading()
                            ).build(), true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString() {
            return "Near 15 Ball (With Far Spike Mark)";
        }
    }
    private class Near_15Ball2 extends Auto{
        @Override
        Pose getStartPose() {
            return paths.nearStart;
        }
        @Override
        boolean getTurretEnabled() {
            return true;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        void runAuto() {
            new SequentialGroup(
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_openGateIntake, true, 1),
                    Commands.waitForArtifacts(),
                    Commands.runPath(paths.openGateIntake_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_openGateIntake, true, 1),
                    Commands.waitForArtifacts(),
                    Commands.runPath(paths.openGateIntake_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(ON, ON),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, false, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString() {
            return "Near 15 Ball (No Far Spike Mark)";
        }
    }
    private class MOE_365_FAR extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.farStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    new ParallelRaceGroup(
                            new Delay(29),
                            new SequentialGroup(
                                    Commands.setFlywheelState(Turret.flywheelState.ON),
                                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.farShoot_to_rampIntake, true, 1),
                                    Commands.runPath(paths.rampIntake_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.farShoot_to_hpPreload, true, 1),
                                    Commands.runPath(paths.hpPreload_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(ON, ON),
                                    Commands.runPath(paths.farShoot_to_hpPreload, true, 1),
                                    Commands.runPath(paths.hpPreload_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
                            )
                    ),
                    Commands.runPath(
                            Robot.drivetrain.follower.pathBuilder().addPath(
                                    new BezierLine(
                                            Robot.drivetrain.follower.getPose(),
                                            paths.farLeave
                                    )
                            ).setLinearHeadingInterpolation(
                                    Robot.drivetrain.follower.getPose().getHeading(),
                                    paths.farLeave.getHeading()
                            ).build(), true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "MOE 365 Far";}
    }

    private class testAuto extends Auto{
        @Override
        Pose getStartPose() {
            return paths.farStart;
        }
        @Override
        boolean getTurretEnabled() {
            return false;
        }
        @Override
        double getHoodOffset() {
            return -25;
        }

        @Override
        void runAuto() {
            new SequentialGroup(
                    Commands.setIntakeMode(ON, ON),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    new WaitUntil(() -> gamepad1.bWasPressed()),
                    Commands.shootBalls()
            ).schedule();
        }

        @NonNull
        @Override
        public String toString() {
            return "Test Auto";
        }
    }
}
