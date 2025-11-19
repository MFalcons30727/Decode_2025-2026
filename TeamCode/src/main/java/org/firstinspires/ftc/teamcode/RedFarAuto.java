package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name="Red far auto", group="Autonomous")
public class RedFarAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constants
    private static final double TICKS_PER_INCH = 50; // I am not sure the ticks per inch, go over calculations, this is placeholder
    private static final double FORWARD_DISTANCE_INCHES = 70; // change at practice, this is just a random placeholder for now
    private static final double DRIVE_POWER = 0.8; // how fast we want the robot for now
    private static final double TURN_POWER = 0.4; // how fast it will turn
    private DcMotorEx shooter = null;
    private CRServo indexer1 = null;
    private CRServo indexer2 = null;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        setMotorDirections();

        // Reset encoders
        resetEncoders();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

//                // Access barcode results
//                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//                for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                    telemetry.addData("Barcode", "Data: %s", br.getData());
//                }
//
//                 //Access classifier results
//                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//                for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//                }
//
//                 //Access detector results
//                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//                for (LLResultTypes.DetectorResult dr : detectorResults) {
//                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//                }

                //Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                }
//
//                 //Access color results
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                for (LLResultTypes.ColorResult cr : colorResults) {
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
            limelight.stop();


            // steps being called

            moveForward(FORWARD_DISTANCE_INCHES, DRIVE_POWER);
            turnRight45();
            shoot();
            turnLeft45();
            moveForward(-20, DRIVE_POWER);
        }
    }

    // --- Helper Functions ---

    private void setMotorDirections() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveForward(double inches, double power) { // these parameters are saying we need to move forward a certain amount of inches, and also how much power we give the motors which is defined at the top.
        int ticks = (int)(inches * TICKS_PER_INCH); // this is saying basically how many inches we want to move multiplied by the ticks per inch. this calculates how many ticks in total we would have to move.

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks); // lines 66-69 is saying "start from current pos and add the amount of ticks you want to move".
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); // just go to the amount of ticks you want
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power); // setting power of 0.5 to the motors.
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && // this chunk is basically telling the robot to keep going until it reaches its target.
                frontLeft.isBusy() &&
                backLeft.isBusy() &&
                frontRight.isBusy() &&
                backRight.isBusy()) {
            // Optional: telemetry here

            telemetry.addData("Shoot Speed", shooter.getPower());
            telemetry.update();

        }

        stopMotors();
    }
    // this turning function is very basic right now, but you get the idea. :)
    // also remember that turn_power is defined at the top as 0.5 so it can be easily reused, and it is constant
    private void turnRight45() {
        // Simple time-based turn
        frontLeft.setPower(TURN_POWER);
        backLeft.setPower(TURN_POWER);
        frontRight.setPower(-TURN_POWER);
        backRight.setPower(-TURN_POWER);
        //this is currently time based which is not ideal, but it works for now
        sleep(750); // adjust for ~45° turn

        stopMotors();
    }

    private void turnLeft45() {
        // Simple time-based turn
        frontLeft.setPower(-TURN_POWER);
        backLeft.setPower(-TURN_POWER);
        frontRight.setPower(TURN_POWER);
        backRight.setPower(TURN_POWER);
        //this is currently time based which is not ideal, but it works for now
        sleep(750); // adjust for ~45° turn

        stopMotors();
    }



    private void shoot() {

        shooter.setMotorEnable();
        shooter.setVelocity(0.53);
        if (shooter.getVelocity() == 0.53){
            sleep(3000);
            telemetry.addData("spinning Up", 100);
            telemetry.update();


            indexer1.setPower(0.6);
            indexer2.setPower(-0.6);
            sleep(1000);
            telemetry.addData("turning off power after shot", 100);
            telemetry.update();
            indexer1.setPower(0);
            indexer2.setPower(0);
            telemetry.addData("Waiting", 100);
            telemetry.update();
            sleep(2500);

            indexer1.setPower(0.6);
            indexer2.setPower(-0.6);
            sleep(100);
            telemetry.addData("turning off power after shot", 100);
            telemetry.update();
            indexer1.setPower(0);
            indexer2.setPower(0);
            telemetry.addData("Waiting", 100);
            telemetry.update();
            sleep(2500);

            indexer1.setPower(0.6);
            indexer2.setPower(-0.6);
            sleep(100);
            telemetry.addData("turning off power after shot", 100);
            telemetry.update();
            indexer1.setPower(0);
            indexer2.setPower(0);
            telemetry.addData("Waiting", 100);
            telemetry.update();

            sleep(500);





//
//            indexer1.setPower(0.6);
//            indexer2.setPower(-0.6);
//            sleep(500);
//            indexer1.setPower(0);
//            indexer2.setPower(0);
//            sleep(10000);
//
//
//            indexer1.setPower(0.6);
//            indexer2.setPower(-0.6);
//            sleep(500);
//            indexer1.setPower(0);
//            indexer2.setPower(0);




        }

    }
    // this function is self explanatory, just stops it
    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}