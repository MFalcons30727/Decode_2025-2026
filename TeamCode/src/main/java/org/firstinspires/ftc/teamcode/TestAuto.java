package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

// panels dashboard: 192.168.43.1:8001

@Autonomous
public class TestAuto extends OpMode {

    public enum AutoState {
        POSE1,
        POSE2,
        POSE3,
        POSE4,
        DONE
    }

    private Timer autoStateTimer, opModeTimer;
    private AutoState currentAutoState;
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    public void autoStateUpdate(){
        switch (currentAutoState){
            case POSE1:
                driver.moveToPose(DriverDanny.Poses.TEST_POSE2, true);
                setAutoState(AutoState.POSE2);
                break;
            case POSE2:
                if (!driver.isBusy()) {
                    driver.moveToPose(DriverDanny.Poses.TEST_POSE3, true);
                    setAutoState(AutoState.POSE3);
                }
                break;
            case POSE3:
                if (!driver.isBusy()) {
                    driver.moveToPose(DriverDanny.Poses.TEST_POSE4, true);
                    setAutoState(AutoState.POSE4);
                }
                break;
            case POSE4:
                if (!driver.isBusy()) {
                    driver.moveToPose(DriverDanny.Poses.TEST_POSE1, true);
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
        currentAutoState = AutoState.POSE1;
        autoStateTimer = new Timer();
        opModeTimer = new Timer();
        driver = new DriverDanny(hardwareMap,
                telemetry,
                DriverDanny.Alliance.BLUE,
                DriverDanny.Poses.TEST_POSE1);
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
        shooter.updateWithLUT(driver.getCurrentDistanceFromGoal());
        autoStateUpdate();

        telemetry.addData("AutoState", currentAutoState.toString());
        telemetry.addData("StateTime", autoStateTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}