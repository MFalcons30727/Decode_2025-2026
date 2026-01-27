package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
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
        TEST_MODE_ONLY
    }

    private static double SHOOTER_ACCEPTABLE_VELOCITY_ERROR = 20; // in case the shooter motor isn't able to reach that exact velocity, allow it to still shoot when being this close to the target velocity
    private static double SHOOTER_VELOCITY_DROP_AFTER_SHOT = 80; // if velocity drops this amount in the START_FEEDING step, we know an artifact has been shot
    private static double HOOD_SERVO_ACCEPTABLE_ERROR = 0.05; // if servo doesn't make it to exact position, this is to say "good enough"
    private static double FEEDER_POWER = 1; // the power we send to the indexer motor to feed
    private static double STEP_TIMEOUT_IN_MILLISECONDS = 4000; // this helps to make sure our "waiting for" steps never run longer than a certain time
    private double shooterTargetVelocity = 1100; // the velocity we want our shooter to be set to
    private double hoodServoPosition = 0; // the servo position of the adjustable hood
    private DcMotorEx shootMotor;
    private DcMotor indexer, intake;
    private Servo hoodServo;
    private Telemetry telemetry;
    private ElapsedTime shootStateTimer; // tried to use the Pedro Pathing timer first but it didn't allow for milliseconds, only seconds
    private ShootingState currentShootingState; // keeping track of the current step we're on in our shooting state machine
    private int shotsFired; // keeps track of how many artifacts we've attempted to shoot (between 0 and 3)
    private InterpLUT velocityLUT, hoodServoPositionLUT; // these are look-up-tables that will help us find the right velocity and hood angle at any distance
    public boolean testMode = false;
    //TelemetryManager.TelemetryWrapper pTelemetry;

    public ShooterMcGavin(HardwareMap hardwareMap, Telemetry telemetryFromOpMode) { // this is our constructor that gets called like this from our autos:  shooter = new Shooter(hardwareMap);
        // think of this like our "init" but for the Shooter specifically
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        telemetry = telemetryFromOpMode;
        shootStateTimer = new ElapsedTime();
        currentShootingState = testMode ? ShootingState.TEST_MODE_ONLY : ShootingState.OFF;
        shotsFired = 0;
        velocityLUT = new InterpLUT();
        hoodServoPositionLUT = new InterpLUT();
        buildLUTS();

        //pTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        //pTelemetry.addLine("Init complete");
        //pTelemetry.update();

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400,0,0,15);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    private void buildLUTS() {
        // TODO: need to measure certain distances and determine what velocity and hood servo positions are best
        // the first parameter of both velocityLUT and hoodServoPositionLUT below is the measured distance from goal (use tape measure from goal to front of robot)
        // on velocityLUT, the second parameter is what velocity we need to set the flywheel to for it to make it
        // on hoodServoPositionLUT, the second parameter is what position we need to set the hood servo to for the best angle
        velocityLUT.add(60, 900);
        velocityLUT.add(72, 920);
        velocityLUT.add(84, 1040);
        velocityLUT.add(96, 1100);
        velocityLUT.add(108, 1160);
        velocityLUT.add(120, 1300);
        velocityLUT.add(132, 1360);
        velocityLUT.add(144, 1500);
        velocityLUT.add(156, 1540);
        velocityLUT.createLUT();

        hoodServoPositionLUT.add(60, 0);
        hoodServoPositionLUT.add(72, 0);
        hoodServoPositionLUT.add(84, 0);
        hoodServoPositionLUT.add(96, 0);
        hoodServoPositionLUT.add(108, 0);
        hoodServoPositionLUT.add(120, 0);
        hoodServoPositionLUT.add(132, 0);
        hoodServoPositionLUT.add(144, 0);
        hoodServoPositionLUT.add(156, 0);
        hoodServoPositionLUT.createLUT();
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
                shootMotor.setVelocity(shooterTargetVelocity);
                //hoodServo.setPosition(hoodServoPosition);
                setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY);
                break;
            case WAIT_FOR_TARGET_VELOCITY: // wait until we're close to the target velocity for the shooter
                if (
                    //(
                        Math.abs(shootMotor.getVelocity() - shooterTargetVelocity) < SHOOTER_ACCEPTABLE_VELOCITY_ERROR
                                //&& Math.abs(hoodServo.getPosition() - hoodServoPosition) < HOOD_SERVO_ACCEPTABLE_ERROR)
                                || shootStateTimer.milliseconds() > STEP_TIMEOUT_IN_MILLISECONDS) { // this makes sure the auto doesn't fail completely if it's not able to ever reach target velocity
                    setShootingState(ShootingState.START_FEEDING);
                }
                break;
            case START_FEEDING: // turn on the indexers to feed the next artifact
                intake.setPower(FEEDER_POWER);
                indexer.setPower(FEEDER_POWER);

                // instead of waiting a certain delay, check for velocity drop to tell if an artifact was shot
                if (shootMotor.getVelocity() < (shooterTargetVelocity - SHOOTER_VELOCITY_DROP_AFTER_SHOT)
                        || shootStateTimer.milliseconds() > STEP_TIMEOUT_IN_MILLISECONDS) {
                    shotsFired++;

                    if (shotsFired >= 3) { // after 3 shot attempts, power off shooter
                        shotsFired = 0;
                        setShootingState(ShootingState.OFF);
                    }
                    else {
                        indexer.setPower(0); // stop feeding until we reach target velocity again
                        setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY); // if more shots to fire, wait for the shooter to reach target velocity again
                    }
                }
                break;
            case OFF:
                shootMotor.setVelocity(0);
                hoodServo.setPosition(0);
                indexer.setPower(0);
                shooterTargetVelocity = 1100;
                hoodServoPosition = 0;
                break;
        }

        telemetry.addData("ShooterState", currentShootingState.toString());
        telemetry.addData("FlywheelVelocity", shootMotor.getVelocity());
        telemetry.addData("TargetVelocity", shooterTargetVelocity);
        telemetry.addData("HoodServoPosition", hoodServo.getPosition());
        telemetry.addData("HoodTargetPosition", hoodServoPosition);
        telemetry.addData("ShotsFired", shotsFired);

//        pTelemetry.addData("ShooterState", currentShootingState.toString());
//        pTelemetry.addData("FlywheelVelocity", shootMotor.getVelocity());
//        pTelemetry.addData("TargetVelocity", shooterTargetVelocity);
//        pTelemetry.addData("HoodServoPosition", hoodServo.getPosition());
//        pTelemetry.addData("HoodTargetPosition", hoodServoPosition);
//        pTelemetry.addData("ShotsFired", shotsFired);
//        pTelemetry.update();
    }

    public void update2(double distanceFromGoalInInches) { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        if (currentShootingState != ShootingState.OFF) {
            shooterTargetVelocity = velocityLUT.get(distanceFromGoalInInches);
            shootMotor.setVelocity(shooterTargetVelocity);
        }

        switch (currentShootingState) {
            case START_SPIN_UP: // start the shooter
                shotsFired = 0;
                shootMotor.setVelocity(shooterTargetVelocity);
                //hoodServo.setPosition(hoodServoPosition);
                setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY);
                break;
            case WAIT_FOR_TARGET_VELOCITY: // wait until we're close to the target velocity for the shooter
                if (
                    //(
                        Math.abs(shootMotor.getVelocity() - shooterTargetVelocity) < SHOOTER_ACCEPTABLE_VELOCITY_ERROR
                                //&& Math.abs(hoodServo.getPosition() - hoodServoPosition) < HOOD_SERVO_ACCEPTABLE_ERROR)
                                || shootStateTimer.milliseconds() > STEP_TIMEOUT_IN_MILLISECONDS) { // this makes sure the auto doesn't fail completely if it's not able to ever reach target velocity
                    setShootingState(ShootingState.START_FEEDING);
                }
                break;
            case START_FEEDING: // turn on the indexers to feed the next artifact
                intake.setPower(FEEDER_POWER);
                indexer.setPower(FEEDER_POWER);

                // instead of waiting a certain delay, check for velocity drop to tell if an artifact was shot
                if (shootMotor.getVelocity() < (shooterTargetVelocity - SHOOTER_VELOCITY_DROP_AFTER_SHOT)
                        || shootStateTimer.milliseconds() > STEP_TIMEOUT_IN_MILLISECONDS) {
                    shotsFired++;

                    if (shotsFired >= 3) { // after 3 shot attempts, power off shooter
                        shotsFired = 0;
                        setShootingState(ShootingState.OFF);
                    }
                    else {
                        indexer.setPower(0); // stop feeding until we reach target velocity again
                        setShootingState(ShootingState.WAIT_FOR_TARGET_VELOCITY); // if more shots to fire, wait for the shooter to reach target velocity again
                    }
                }
                break;
            case OFF:
                shootMotor.setVelocity(0);
                hoodServo.setPosition(0);
                indexer.setPower(0);
                shooterTargetVelocity = 1100;
                hoodServoPosition = 0;
                break;
        }

        telemetry.addData("ShooterState", currentShootingState.toString());
        telemetry.addData("FlywheelVelocity", shootMotor.getVelocity());
        telemetry.addData("TargetVelocity", shooterTargetVelocity);
        telemetry.addData("HoodServoPosition", hoodServo.getPosition());
        telemetry.addData("HoodTargetPosition", hoodServoPosition);
        telemetry.addData("ShotsFired", shotsFired);

//        pTelemetry.addData("ShooterState", currentShootingState.toString());
//        pTelemetry.addData("FlywheelVelocity", shootMotor.getVelocity());
//        pTelemetry.addData("TargetVelocity", shooterTargetVelocity);
//        pTelemetry.addData("HoodServoPosition", hoodServo.getPosition());
//        pTelemetry.addData("HoodTargetPosition", hoodServoPosition);
//        pTelemetry.addData("ShotsFired", shotsFired);
//        pTelemetry.update();
    }

    public void startShootingFromDistance(double distanceFromGoalInInches) { // if this version of startShooting is called with no arguments, use the LUTs to determine velocity and hood servo position
        // maybe with LEDs, if it's a distance we can shoot from, turn LED green.  otherwise turn it red.
        // may need to add some error handling here since InterpLUT will throw exceptions if we ask for a distance that is outside of the min/max values we measured
        shooterTargetVelocity = velocityLUT.get(distanceFromGoalInInches);
        //hoodServoPosition = hoodServoPositionLUT.get(distanceFromGoalInInches);
        setShootingState(ShootingState.START_SPIN_UP);
    }
    public void startShootingAtVelocity(double targetVelocity) {
        // this lets us continue to shoot the old way if we need to for testing
        shooterTargetVelocity = targetVelocity;
        //hoodServoPosition = 0; // retract hood all the way
        setShootingState(ShootingState.START_SPIN_UP);
    }

    public void startShootingAtVelocityAndHoodPosition(double targetVelocity,
                                                       double targetHoodServoPosition) {
        // can use this one for testing of our LUT measurements
        shooterTargetVelocity = targetVelocity;
        //hoodServoPosition = targetHoodServoPosition;
        setShootingState(ShootingState.START_SPIN_UP);
    }

    public void startShooting() {
        setShootingState(ShootingState.START_SPIN_UP);
    }

    /* THESE ARE TESTING FUNCTIONS FOR THE NEW BOT */
    public void turnOnIntake() { intake.setPower(1); }
    public void turnOffIntake() { intake.setPower(0); }
    public void setHoodUp() { hoodServo.setPosition(1); }
    public void setHoodDown() { hoodServo.setPosition(0); }
    public void turnOnIndexer() { indexer.setPower(1); }
    public void turnOffIndexer() { indexer.setPower(0); }
    public void turnOnFlywheel() { shootMotor.setVelocity(1400); }
    public void turnOffFlywheel() { shootMotor.setVelocity(0); }
}
