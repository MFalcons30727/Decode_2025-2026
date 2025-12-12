package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterMcGavin {

    // when declaring enums and class-level variables, always use private if it's only used or
    // needed within this class.  otherwise you might get confused and try to use them in other
    // classes when they aren't designed for that
    private enum ShootingState {
        OFF, // shooter is powered down
        START_SPIN_UP, // start spinning up the shooter to the SHOOTER_TARGET_VELOCITY
        WAIT_FOR_TARGET_VELOCITY, // wait until we're at the SHOOTER_TARGET_VELOCITY before feeding
        START_FEEDING, // start indexers to feed one artifact
        STOP_FEEDING, // stop indexers so the shooter has time to get back to the SHOOTER_TARGET_VELOCITY
        FLYWHEEL_RECOVERY
    }

    private double SHOOTER_TARGET_VELOCITY = 1100; // the velocity we want our shooter to be set to
    private static final double SHOOTER_ACCEPTABLE_VELOCITY_ERROR = 50; // in case the shooter motor isn't able to reach that exact velocity, allow it to still shoot when being this close to the target velocity
    private static final double TIME_TO_FEED_IN_MILLISECONDS = 5000; // this is how long it takes our indexers to feed one artifact through (the time between starting and stopping the indexers)
    //    private static final double FLYWHEEL_RECOVERY_TIME_IN_MILLISECONDS = 300;
    private static final double FEEDER_POWER = 1; // the power we send to the indexer servos to feed
    private static final double STEP_TIMEOUT_IN_MILLISECONDS = 2000;
    private static final double REVERSE_TIME_IN_MILLISECONDS = 300;
    private final DcMotorEx shootMotor;
    private final DcMotor indexer;
    private ElapsedTime shootStateTimer; // tried to use the Pedro Pathing timer first but it didn't allow for milliseconds, only seconds
    private ShootingState currentShootingState; // keeping track of the current step we're on in our shooting state machine
    private int shotsFired; // keeps track of how many artifacts we've attempted to shoot (between 0 and 3)
    private Telemetry telemetry = null;

    public ShooterMcGavin(HardwareMap hardwareMap, Telemetry telemetry) { // this is our constructor that gets called like this from our autos:  shooter = new Shooter(hardwareMap);
        // think of this like our "init" but for the Shooter specifically
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        shootStateTimer = new ElapsedTime();
        currentShootingState = ShootingState.OFF;
        shotsFired = 0;
        this.telemetry = telemetry;

        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(181,0,0,12);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    private void setShootingState(ShootingState newState) {
        currentShootingState = newState;
        shootStateTimer.reset();
    }

    public boolean isShooting() { // this can be called by our main auto routine and let it know if we're still in the process of trying to shoot the 3 artifacts
        if (currentShootingState == ShootingState.OFF) {
            return false;
        }
        else {
            return true;
        }
    }

    public void update() { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        switch (currentShootingState) {
            case START_SPIN_UP: // start the shooter
                shotsFired = 0;
                shootMotor.setVelocity(SHOOTER_TARGET_VELOCITY);
                setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY);
                break;
            case WAIT_FOR_TARGET_VELOCITY: // wait until we're close to the target velocity for the shooter
                if (Math.abs(shootMotor.getVelocity() - SHOOTER_TARGET_VELOCITY) < SHOOTER_ACCEPTABLE_VELOCITY_ERROR
                        && shootStateTimer.milliseconds() > STEP_TIMEOUT_IN_MILLISECONDS) { // this makes sure the auto doesn't fail completely if it's not able to ever reach target velocity
                    setShootingState(ShootingState.START_FEEDING);
                }
                break;
            case START_FEEDING: // turn on the indexers to feed the next artifact
                indexer.setPower(FEEDER_POWER);

                if (shootStateTimer.milliseconds() > TIME_TO_FEED_IN_MILLISECONDS) { // wait for the indexer to finish feeding the artifact
                    shotsFired++;
                    setShootingState(ShootingState.STOP_FEEDING);
                }
                break;
            case STOP_FEEDING:
                indexer.setPower(0);

                if (shotsFired == 1) { // after 3 shot attempts, power off shooter
                    shotsFired = 0;
                    setShootingState(ShootingState.OFF);
                }
                else {
                    setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY); // if more shots to fire, wait for the shooter to reach target velocity again
                }
                break;
//            case REVERSE_FEED:
//                shootMotor.setVelocity(-100);
//                indexer.setPower(-FEEDER_POWER);
//                if (shootStateTimer.milliseconds() > REVERSE_TIME_IN_MILLISECONDS) {
//                    setShootingState(ShootingState.OFF);
//                }
//                break;
//            case FLYWHEEL_RECOVERY:
//
//                if (shootStateTimer.milliseconds() > FLYWHEEL_RECOVERY_TIME_IN_MILLISECONDS){
//                    setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY);
//                }
//                break;
            case OFF:
                shootMotor.setVelocity(0);
                indexer.setPower(0);
                break;

        }

        telemetry.addData("shooter velocity", shootMotor.getVelocity());
    }

    public void startShooting(double targetVelocity) {
        // this is what will be called by our main auto code to shoot 3 artifacts automatically
        SHOOTER_TARGET_VELOCITY = targetVelocity;
        setShootingState(ShootingState.START_SPIN_UP);
    }

//    public void reverseFeed() {
//        setShootingState(ShootingState.REVERSE_FEED);
//    }
}
