package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name="Blue far auto", group="Autonomous")
public class BasicAuto extends LinearOpMode {

    // Constants
    private static final double TELEMETRY_TRANSMISSION_INTERVAL_IN_MILLISECONDS = 11;
    private static final double TICKS_PER_INCH = 50; // I am not sure the ticks per inch, go over calculations, this is placeholder
    private static final double MILLISECONDS_PER_TURN_DEGREE = 16.666; // Right now this is calculated based on 45 degrees taking 750ms to turn
    
    private static final double DRIVE_POWER = 0.8; // how fast we want the robot for now
    private static final double TURN_POWER = 0.4; // how fast it will turn
    private static final double SHOOTER_TARGET_POWER = 0.53; // target power of shooter
    private static final double INDEXER_FEED_POWER = 0.6; // power of indexer feed

    // Members
    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorShooter;
    private CRServo indexer1, indexer2;
    private Limelight3A limeLight;

    @Override
    public void runOpMode() {
        mapHardware();
        initializeMotors();
        initializeLimeLight();

        telemetry.setMsTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL_IN_MILLISECONDS);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            refreshLimeLight()

            // Move, shoot, and retreat!
            move(Direction.FORWARD, 70, DRIVE_POWER)
            turn(Direction.LEFT, 45);
            shootArtifacts(3);
            turn(Direction.RIGHT, 45);
            move(Direction.BACKWARD, 20, DRIVE_POWER);
        }
    }

    // --- Helper Functions ---
    private void mapHardware() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftRear");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightRear");
        motorShooter = hardwareMap.get(DcMotor.class, "shooter");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        limeLight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    private void initializeMotors() {
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorShooter.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Don't these 4 lines get overridden by the next 4 lines?
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeLimeLight() {
        limeLight.pipelineSwitch(0);
        limeLight.start(); // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
    }

    private void refreshLimeLight() {
        LLStatus limeLightStatus = limeLight.getStatus();
        LimeLightResultType desiredResultType = LimeLightResultType.FIDUCIAL;

        telemetry.addData("Name", "%s",
                limeLightStatus.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                limeLightStatus.getTemp(), limeLightStatus.getCpu(), (int)limeLightStatus.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                limeLightStatus.getPipelineIndex(), limeLightStatus.getPipelineType());

        LLResult limeLightResult = limeLight.getLatestResult();

        if (limeLightResult.isValid()) {
            Pose3D botpose = limeLightResult.getBotpose();
            double captureLatency = limeLightResult.getCaptureLatency();
            double targetingLatency = limeLightResult.getTargetingLatency();
            double parseLatency = limeLightResult.getParseLatency();

            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(limeLightResult.getPythonOutput()));
            telemetry.addData("tx", limeLightResult.getTx());
            telemetry.addData("txnc", limeLightResult.getTxNC());
            telemetry.addData("ty", limeLightResult.getTy());
            telemetry.addData("tync", limeLightResult.getTyNC());
            telemetry.addData("Botpose", botpose.toString());

            switch (desiredResultType) {
                case LimeLightResultType.FIDUCIAL:
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                    break;
                case LimeLightResultType.BARCODE:
                    List<LLResultTypes.BarcodeResult> barcodeResults = limeLightResult.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }
                    break;
                case LimeLightResultType.CLASSIFIER:
                    List<LLResultTypes.ClassifierResult> classifierResults = limeLightResult.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }
                    break;
                case LimeLightResultType.DETECTOR:
                    List<LLResultTypes.DetectorResult> detectorResults = limeLightResult.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }             
                    break;
                case LimeLightResultType.COLOR:
                    List<LLResultTypes.ColorResult> colorResults = limeLightResult.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                    break;
                default:
                    break;
            }
        }
        else { 
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
        limeLight.stop();
    }

    // these parameters are saying we need to move forward a certain amount of inches, and also how much power we give the motors which is defined at the top.
    private void move(Direction direction, double distanceInInches, double drivePower) { 
        int ticks = (int)(distanceInInches * TICKS_PER_INCH); // this is saying basically how many inches we want to move multiplied by the ticks per inch. this calculates how many ticks in total we would have to move.

        if (direction == Direction.BACKWARD) { ticks = ticks * -1; } // If moving backwards, ticks should be reversed.

        // lines 66-69 is saying "start from current pos and add the amount of ticks you want to move".
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks); 
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);

        // just go to the amount of ticks you want
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // setting power to the motors.
        motorFrontLeft.setPower(drivePower); 
        motorBackLeft.setPower(drivePower);
        motorFrontRight.setPower(drivePower);
        motorBackRight.setPower(drivePower);

        // this chunk is basically telling the robot to keep going until it reaches its target.
        while (opModeIsActive() && 
                (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy())
              ) {
            // Optional: telemetry here
            telemetry.addData("Shoot Speed", motorShooter.getPower());
            telemetry.update();
        }

        stopMotors();
    }

    // this turning function is very basic right now, but you get the idea. :)
    // this is currently time based which is not ideal, but it works for now
    // also remember that turn_power is defined at the top as 0.5 so it can be easily reused, and it is constant
    private void turn(Direction direction, double degreesToTurn) {
        if (direction == Direction.LEFT) {
            motorFrontLeft.setPower(-TURN_POWER);
            motorBackLeft.setPower(-TURN_POWER);
            motorFrontRight.setPower(TURN_POWER);
            motorBackRight.setPower(TURN_POWER);
        }
        else {
            motorFrontLeft.setPower(TURN_POWER);
            motorBackLeft.setPower(TURN_POWER);
            motorFrontRight.setPower(-TURN_POWER);
            motorBackRight.setPower(-TURN_POWER);
        }

        sleep(degreesToTurn * MILLISECONDS_PER_TURN_DEGREE);
        stopMotors();
    }

    // this function is self explanatory, just stops it
    private void stopMotors() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void shootArtifacts(int numberOfArtifactsToShoot) {
        double delayBetweenShotsInMilliseconds = 2500;

        motorShooter.setPower(SHOOTER_TARGET_POWER);

        telemetry.addData("spinning Up Shooter", 100);
        telemetry.update();

        while (motorShooter.getPower() < SHOOTER_TARGET_POWER) { sleep(10); } // Wait until motor is spun up.

        telemetry.addData("shooter Ready", 100);
        telemetry.update();

        for (int i = 0; i < numberOfArtifactsToShoot; i++)
        {
            feedArtifactFromIndexers();
            telemetry.addData("waiting for next shot", 100);
            telemetry.update();
            sleep(delayBetweenShotsInMilliseconds);
        }
    }

    private void feedArtifactFromIndexers() {
        double timeToFeedInMilliseconds = 1000;

        indexer1.setPower(INDEXER_FEED_POWER);
        indexer2.setPower(-INDEXER_FEED_POWER);
        sleep(timeToFeedInMilliseconds);
        telemetry.addData("feed complete", 100);
        telemetry.update();
        indexer1.setPower(0);
        indexer2.setPower(0);
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public enum LimeLightResultType {
        FIDUCIAL,
        BARCODE,
        CLASSIFIER,
        DETECTOR,
        COLOR
    }
}