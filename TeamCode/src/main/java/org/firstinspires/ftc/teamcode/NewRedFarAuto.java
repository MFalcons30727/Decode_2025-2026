package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

// panels dashboard: 192.168.43.1:8001

@Autonomous
public class NewRedFarAuto extends OpMode {
    public enum AutoState {
        MOVE_FROM_START_POS_TO_SHOOTING_LINE,
        SHOOT_PRELOAD,
        MOVE_FROM_SHOOTING_LINE_TO_RED_GATE,
        DONE
    }

    private Timer autoStateTimer, opModeTimer;
    private AutoState currentAutoState;
    private DriverDanny driver;
    private ShooterMcGavin shooter;

    public void autoStateUpdate(){
        switch (currentAutoState){
            case MOVE_FROM_START_POS_TO_SHOOTING_LINE:
                driver.moveToPose(DriverDanny.Poses.RED_NEAR_SHOOTING_POSE, true);
                setAutoState(AutoState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!driver.isBusy()) {
                    //shooter.startShootingAtVelocity(1100);
                    shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
                    setAutoState(AutoState.MOVE_FROM_SHOOTING_LINE_TO_RED_GATE);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_GATE:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.PARK_RED_GATE_POSE, true);
                    setAutoState(AutoState.DONE);
                }
                break;
            case DONE:
                // do nothing here from right now.  maybe stop motors or something in the future?
                break;
            default:
                telemetry.addLine("AutoState unknown");
                break;
        }
    }

    public void setAutoState(AutoState newState) {
        currentAutoState = newState;
        autoStateTimer.resetTimer();
    }

    @Override
    public void init() { // this runs once when you hit "init" on the driver hub
        currentAutoState = AutoState.MOVE_FROM_START_POS_TO_SHOOTING_LINE;
        autoStateTimer = new Timer();
        opModeTimer = new Timer();
        driver = new DriverDanny(hardwareMap,
                                    telemetry,
                                    DriverDanny.Alliance.RED,
                                    DriverDanny.Poses.RED_FAR_START_POSE);
        shooter = new ShooterMcGavin(hardwareMap, telemetry);
    }

    @Override
    public void start() { // this runs once when you hit "play" on the driver hub
        opModeTimer.resetTimer();
        setAutoState(currentAutoState);
    }

    @Override
    public void loop(){
        driver.update();
        shooter.update();
        autoStateUpdate();

        telemetry.addData("AutoState", currentAutoState.toString());
        telemetry.addData("StateTime", autoStateTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}