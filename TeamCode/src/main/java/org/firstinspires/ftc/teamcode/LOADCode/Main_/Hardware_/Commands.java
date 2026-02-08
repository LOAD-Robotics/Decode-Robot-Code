package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import androidx.annotation.NonNull;

import com.pedropathing.paths.PathChain;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    // Robot Object for command access
    public LoadHardwareClass Robot;
    public Commands(@NonNull LoadHardwareClass robot){
        Robot = robot;
    }

    // Delay timer for shooting sequence
    private static final TimerEx shootingTimerFifthSec = new TimerEx(0.2);
    private static final TimerEx shootingTimerHalfSec = new TimerEx(0.5);
    private static final TimerEx shootingTimer2sec = new TimerEx(2);
    private static final TimerEx shootingTimer5sec = new TimerEx(5);
    private static Command resetShootingTimerFifthsec() {
        return new LambdaCommand("resetShootingTimer0.2sec").setStart(shootingTimerFifthSec::restart);
    }
    private static Command resetShootingTimerHalfsec() {
        return new LambdaCommand("resetShootingTimer0.5sec").setStart(shootingTimerHalfSec::restart);
    }
    private static Command resetShootingTimer2sec() {
        return new LambdaCommand("resetShootingTimer2sec").setStart(shootingTimer2sec::restart);
    }
    private static Command resetShootingTimer5sec() {
        return new LambdaCommand("resetShootingTimer5sec").setStart(shootingTimer2sec::restart);
    }

    public Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPath(path, holdEnd, maxPower);
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

    public Command setIntakeMode(Intake.intakeMode state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setMode(state))
                .setIsDone(() -> true)
        );
    }

    public Command setTransferState(Intake.transferState state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setTransfer(state))
                .setIsDone(() -> true)
        );
    }

    public Command waitForArtifacts(){
        return new SequentialGroup(
                setIntakeMode(Intake.intakeMode.INTAKING),
                resetShootingTimer5sec(),
                new WaitUntil(() -> Robot.intake.getTopSensorState() || Robot.intake.getBottomSensorState()),
                new ParallelRaceGroup(
                        new WaitUntil(() -> Robot.intake.getTopSensorState() && Robot.intake.getBottomSensorState()),
                        new WaitUntil(shootingTimer5sec::isDone)
                )
        );
    }

    public Command shootBalls(){
        return new SequentialGroup(
                // Ensure the flywheel is up to speed, if not, spin up first
                setFlywheelState(Turret.flywheelState.ON),

                // Shoot the first two balls
                setGateState(Turret.gatestate.OPEN),
                resetShootingTimerFifthsec(),
                new WaitUntil(shootingTimerFifthSec::isDone),
                setIntakeMode(Intake.intakeMode.INTAKING),
                resetShootingTimer2sec(),
                new WaitUntil(() -> (Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState() && shootingTimer2sec.isDone())),

                // Shoot the last ball
                setIntakeMode(Intake.intakeMode.SHOOTING),
                setTransferState(Intake.transferState.UP),
                resetShootingTimerHalfsec(),
                new WaitUntil(shootingTimerHalfSec::isDone),

                // Reset the systems
                setIntakeMode(Intake.intakeMode.OFF),
                setGateState(Turret.gatestate.CLOSED),
                setTransferState(Intake.transferState.DOWN)
        );
    }

}
