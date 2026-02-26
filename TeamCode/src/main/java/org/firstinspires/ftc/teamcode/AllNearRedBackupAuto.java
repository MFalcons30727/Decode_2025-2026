package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

// panels dashboard: 192.168.43.1:8001

@Autonomous
public class AllNearRedBackupAuto extends OpMode {

    public enum AutoState {
        MOVE_FROM_START_POS_TO_SHOOTING_LINE,
        SHOOT_PRELOAD,
        MOVE_FROM_SHOOTING_LINE_TO_RED_TOP_ARTIFACTS,
        EAT_TOP_RED_ARTIFACTS,
        MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_TOP_RED_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_RED_MIDDLE_ARTIFACTS,
        EAT_MIDDLE_RED_ARTIFACTS,
        AVOID_GATE_AFTER_EATING_MIDDLE_RED_ARTIFACTS,
        MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_MIDDLE_RED_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_RED_BOTTOM_ARTIFACTS,
        EAT_BOTTOM_RED_ARTIFACTS,
        MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_FINAL_SHOOTING_LINE,
        SHOOT_BOTTOM_RED_ARTIFACTS,
        DONE
    }

    private Timer autoStateTimer, opModeTimer;
    private AutoState currentAutoState;
    private DriverDanny driver;
    private ShooterMcGavin shooter;

    public void autoStateUpdate(){
        switch (currentAutoState){
            case MOVE_FROM_START_POS_TO_SHOOTING_LINE:
                driver.moveToPose(DriverDanny.Poses.ANRBA_NEAR_SHOOTING_POSE, true);
                setAutoState(AutoState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!driver.isBusy()) {
                    shooter.startShootingAtVelocityAndHoodPosition(980, 0.3);
                    setAutoState(AutoState.MOVE_FROM_SHOOTING_LINE_TO_RED_TOP_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_TOP_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_TOP_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.EAT_TOP_RED_ARTIFACTS);
                }
                break;
            case EAT_TOP_RED_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_EAT_TOP_ARTIFACTS_POSE, true);
                    shooter.turnOnIntake();
                    setAutoState(AutoState.MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_NEAR_SHOOTING_POSE, true);
                    setAutoState(AutoState.SHOOT_TOP_RED_ARTIFACTS);
                }
                break;
            case SHOOT_TOP_RED_ARTIFACTS:
                if (!driver.isBusy()) {
                    shooter.startShootingAtVelocityAndHoodPosition(980, 0.3);
                    setAutoState(AutoState.MOVE_FROM_SHOOTING_LINE_TO_RED_MIDDLE_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_MIDDLE_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_MIDDLE_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.EAT_MIDDLE_RED_ARTIFACTS);
                }
                break;
            case EAT_MIDDLE_RED_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()) {
                    driver.moveToPose(DriverDanny.Poses.ANRBA_EAT_MIDDLE_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.AVOID_GATE_AFTER_EATING_MIDDLE_RED_ARTIFACTS);
                }
                break;
            case AVOID_GATE_AFTER_EATING_MIDDLE_RED_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()) {
                    driver.moveToPose(DriverDanny.Poses.ANRBA_MIDDLE_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE:
                if (!driver.isBusy() && !shooter.isShooting()) {
                    driver.moveToPose(DriverDanny.Poses.ANRBA_NEAR_SHOOTING_POSE, true);
                    setAutoState(AutoState.SHOOT_MIDDLE_RED_ARTIFACTS);
                }
                break;
            case SHOOT_MIDDLE_RED_ARTIFACTS:
                if (!driver.isBusy()) {
                    shooter.startShootingAtVelocityAndHoodPosition(980, 0.3);
                    setAutoState(AutoState.MOVE_FROM_SHOOTING_LINE_TO_RED_BOTTOM_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_BOTTOM_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_BOTTOM_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.EAT_BOTTOM_RED_ARTIFACTS);
                }
                break;
            case EAT_BOTTOM_RED_ARTIFACTS:
                if (!driver.isBusy() && !shooter.isShooting()){
                    driver.moveToPose(DriverDanny.Poses.ANRBA_EAT_BOTTOM_ARTIFACTS_POSE, true);
                    setAutoState(AutoState.MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_FINAL_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_FINAL_SHOOTING_LINE:
                if (!driver.isBusy() && !shooter.isShooting()) {
                    driver.moveToPose(DriverDanny.Poses.ANRBA_FINAL_SHOOTING_POSE, true);
                    setAutoState(AutoState.SHOOT_BOTTOM_RED_ARTIFACTS);
                }
                break;
            case SHOOT_BOTTOM_RED_ARTIFACTS:
                if (!driver.isBusy()) {
                    shooter.startShootingAtVelocityAndHoodPosition(980, 0.3);
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
                DriverDanny.Poses.ANRBA_START_POSE);
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