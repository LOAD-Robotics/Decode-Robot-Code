package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends NextFTCOpMode {

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
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        prompter = new Prompter(this);
        prompter.prompt("alliance",
                new OptionPrompt<>("Select Alliance",
                        LoadHardwareClass.Alliance.RED,
                        LoadHardwareClass.Alliance.BLUE
                ));
        prompter.prompt("auto",
                new OptionPrompt<>("Select Auto",
                        new MOE_365_FAR(),
                        new Near_12Ball(),
                        new Near_9Ball(),
                        new Far_9Ball(),
                        new Far_6Ball()
                ));
        prompter.onComplete(() -> {
                    selectedAlliance = prompter.get("alliance");
                    selectedAuto = prompter.get("auto");
                    telemetry.update();
                    // Build paths
                    paths.buildPaths(follower());
                    // Initialize all hardware of the robot
                    Robot.init(selectedAuto.getStartPose(), follower());
                    while (opModeInInit() && Robot.turret.zeroTurret()){
                        telemetry.addLine("TURRET ZEROING");
                        telemetry.addData("Selection", "Complete");
                        telemetry.addData("Alliance", selectedAlliance.toString());
                        telemetry.addData("Auto", selectedAuto);
                        telemetry.update();
                        sleep(0);
                    }
            telemetry.addLine("TURRET READY");
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
        // Schedule the proper auto
        selectedAuto.runAuto();
        turretOn = selectedAuto.getTurretEnabled();

        // Indicate that initialization is done
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Running Auto", selectedAuto.toString());
        telemetry.addData("Alliance", selectedAlliance);
        Robot.turret.updateAimbot(turretOn, true, 0);
        Robot.turret.updateFlywheel();

        MecanumDrivetrainClass.robotPose = Robot.drivetrain.follower.getPose();

        telemetry.addLine();
        telemetry.addData("Mirror -90", Math.toDegrees(paths.autoMirror(new Pose(0,0,-90)).getHeading()));
        telemetry.update();
    }

    @Override
    public void onStop(){
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

    private class Far_6Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.farStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Far Zone 6 Ball";}
    }

    private class Far_9Ball extends Auto{
        @Override
        public Pose getStartPose(){
            return paths.farStart;
        }
        @Override
        public boolean getTurretEnabled(){
            return true;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_farShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Far Zone 9 Ball";}
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
        public void runAuto(){
            new SequentialGroup(
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near Zone 9 Ball";}
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
        public void runAuto(){
            new SequentialGroup(
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near Zone 12 Ball";}
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
        public void runAuto(){
            new SequentialGroup(
                    new ParallelRaceGroup(
                            new Delay(29),
                            new SequentialGroup(
                                    Commands.setFlywheelState(Turret.flywheelState.ON),
                                    Commands.runPath(paths.farStart_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                                    Commands.runPath(paths.farShoot_to_farPreload, true, 1),
                                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                                    Commands.runPath(paths.farShoot_to_rampIntake, true, 1),
                                    Commands.runPath(paths.rampIntake_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                                    Commands.runPath(paths.farShoot_to_hpPreload, true, 1),
                                    Commands.runPath(paths.hpPreload_to_farShoot, true, 1),
                                    Commands.shootBalls(),
                                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
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
        public String toString(){return "MOE 365 Far Zone Auto";}
    }
}
