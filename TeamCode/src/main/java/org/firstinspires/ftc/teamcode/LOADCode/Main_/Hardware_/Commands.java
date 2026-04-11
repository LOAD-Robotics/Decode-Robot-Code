package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.OFF;
import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode.ON;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    // Robot Object for command access
    public LoadHardwareClass Robot;
    public Pedro_Paths paths = new Pedro_Paths();
    public Commands(@NonNull LoadHardwareClass robot){
        Robot = robot;
    }

    public static int shootingState = 0;
    public static boolean isDoneShooting = false;

    // Delay timer for shooting sequence
    private static final TimerEx shootingTimerFifthSec = new TimerEx(0.2);
    private static final TimerEx shootingTimerHalfSec = new TimerEx(0.5);
    private static final TimerEx shootingTimer1sec = new TimerEx(1);
    private static final TimerEx shootingTimer2sec = new TimerEx(2);
    private static final TimerEx shootingTimer5sec = new TimerEx(5);
    private static Command resetShootingTimerFifthsec() {
        return new LambdaCommand("resetShootingTimer0.2sec").setStart(shootingTimerFifthSec::restart);
    }
    private static Command resetShootingTimerHalfsec() {
        return new LambdaCommand("resetShootingTimer0.5sec").setStart(shootingTimerHalfSec::restart);
    }
    private static Command resetShootingTimer1sec() {
        return new LambdaCommand("resetShootingTimer1sec").setStart(shootingTimer1sec::restart);
    }
    private static Command resetShootingTimer2sec() {
        return new LambdaCommand("resetShootingTimer2sec").setStart(shootingTimer2sec::restart);
    }
    private static Command resetShootingTimer5sec() {
        return new LambdaCommand("resetShootingTimer5sec").setStart(shootingTimer2sec::restart);
    }

    public Command runPath(PathChain path){
        return runPath(path, true, 1);
    }

    public Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new ParallelRaceGroup(
                new FollowPath(path, holdEnd, maxPower),
                new SequentialGroup(
                        new Delay(1),
                        new WaitUntil(() -> Robot.drivetrain.follower.getVelocity().getMagnitude() < 0.2)
                )
        );
    }

    public Command setFlywheelState(Turret.flywheelState state) {
        return new LambdaCommand("setFlywheelState()")
                .setInterruptible(false)
                .setStart(() -> Robot.turret.setFlywheelState(state))
                .setIsDone(() -> {
                    if (state == Turret.flywheelState.ON){
                        return Robot.turret.getFlywheelRPM() > Robot.turret.getFlywheelCurrentMaxSpeed() - 100;
                    }else{
                        return true;
                    }
                })
        ;
    }

    private Command setGateState(Turret.gatestate state){
        return new InstantCommand(new LambdaCommand("setGateState")
                .setStart(() -> Robot.turret.setGateState(state))
        );
    }

    public Command setIntakeMode(Intake.intakeMode intake, Intake.intakeMode belt) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setMode(intake, belt))
        );
    }

    public Command setTransferState(Intake.transferState state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setTransfer(state))
        );
    }

    public Command waitForTurret(double posErrorThreshold, double velErrorThreshold){
        return new ParallelGroup(
                new WaitUntil(() -> Robot.turret.rotation.getAngleAbsolute() < Robot.turret.rotation.target + posErrorThreshold || Robot.turret.rotation.getAngleAbsolute() > Robot.turret.rotation.target - posErrorThreshold),
                new WaitUntil(() -> Robot.turret.rotation.getDegreesPerSecond() < velErrorThreshold)
        );
    }

    /**
     * Waits until both proximity sensors are activated at the same time or until 0.7 seconds have passed
     */
    public Command waitForArtifacts(){
        return new ParallelRaceGroup(
                new Delay(0.7),
                new WaitUntil(() -> (Robot.intake.getTopSensorState()))
        );
    }

    public Command leaveAtEnd(Command auto, Pose leavePose){
        return new SequentialGroup(
                new ParallelRaceGroup(
                        new Delay(29),
                        auto
                ),
                runPath(
                        Robot.drivetrain.follower.pathBuilder().addPath(
                                new BezierLine(
                                        Robot.drivetrain.follower.getPose(),
                                        leavePose
                                )
                        ).setTangentHeadingInterpolation().build(), true, 1)
        );
    }

    public Command leaveLineSafetyCommand(Command mainAuto, double duration){
        return new SequentialGroup(
                new ParallelRaceGroup(
                        mainAuto,
                        new Delay(duration)
                ),
                runPath(
                        Robot.drivetrain.follower.pathBuilder().addPath(
                                new BezierLine(
                                        Robot.drivetrain.follower.getPose(),
                                        paths.farLeave
                                )
                        ).setLinearHeadingInterpolation(
                                Robot.drivetrain.follower.getPose().getHeading(),
                                paths.farLeave.getHeading()
                        ).build(), true, 1)
        );
    }

    public Command shootBalls(){
        return new SequentialGroup(
                setFlywheelState(Turret.flywheelState.ON),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                // Shoot the first two balls
                                waitForTurret(2, 4),
                                setGateState(Turret.gatestate.OPEN),
                                new Delay(0.25),
                                setIntakeMode(ON, ON),
                                new ParallelGroup(
                                        new Delay(0.5),
                                        new WaitUntil(() -> (Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState()))
                                ),

                                // Shoot the last ball
                                setIntakeMode(OFF, ON),
                                setTransferState(Intake.transferState.UP),
                                new Delay(0.3)
                        ),
                        new Delay(3)
                ),
                new ParallelGroup(
                        setIntakeMode(OFF, OFF),
                        setGateState(Turret.gatestate.CLOSED),
                        setTransferState(Intake.transferState.DOWN)
                )
        );
    }
}
