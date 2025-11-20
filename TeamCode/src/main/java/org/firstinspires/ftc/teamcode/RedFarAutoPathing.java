package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name="RedFarAutoPathing", group="Autonomous")
public class RedFarAutoPathing extends LinearOpMode {
    // Constants - adjust these values and distances based on testing
    // Came up with these initial values from looking at page 59 of the game manual at https://ftc-resources.firstinspires.org/ftc/game/manual
    private static final double TICKS_PER_INCH = 50; // I am not sure the ticks per inch, go over calculations, this is placeholder
    private static final double DRIVE_POWER = 0.8; // how fast we want the robot for now
    private static final double TURN_POWER = 0.4; // how fast it will turn
    private static final double INCHES_FROM_LAUNCH_LINE_TO_SHOOTING_LINE = 70;
    private static final double INCHES_FROM_SHOOTING_LINE_TO_FIRST_ARTIFACTS = 12;
    private static final double INCHES_FROM_SHOOTING_LINE_TO_RELEASE_LEVER = 36;
    private static final double INCHES_BETWEEN_ARTIFACT_GROUPS = 24;
    private static final double INCHES_FORWARD_TO_GRAB_ARTIFACTS = 12;

    // Variables
    private DcMotor frontLeft, frontRight, backLeft, backRight, shoot;
    private CRServo indexer1, indexer2;
    private Limelight3A limelight;
    private int obeliskTagID = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(11);

        initializeMotors();
        setMotorDirections();
        resetEncoders();
        initializeLimelight();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            // Attempt to read the AprilTag ID on the obelisk using the Limelight
            getLatestLimelightInfo();

            // Move forward by INCHES_FROM_LAUNCH_LINE_TO_SHOOTING_LINE
            moveForward(INCHES_FROM_LAUNCH_LINE_TO_SHOOTING_LINE, DRIVE_POWER);

            // If we're still unsure of the obelisk pattern, try to get it again now that we're closer
            if (obeliskTagID == 0)
            {
                getLatestLimelightInfo();
            }

            // Turn 45 degrees right (towards the red goal - would be better to do this with April Tag)
            turnRight45();

            // Shoot 3 times
            shoot();

            // Turn 45 degrees right (to face first set of artifacts) - Again, better to orient from red goal April Tag if possible
            turnRight45();

            // Check vs pattern to determine how far to move right from the shooting line
            // If we don't know the obelisk pattern, just proceed to the closest set of artifacts
            double distanceToMoveFromShootingLine = 0;

            switch (obeliskTagID) {
                case 21: // GPP
                    distanceToMoveFromShootingLine = INCHES_FROM_SHOOTING_LINE_TO_FIRST_ARTIFACTS + (2 * INCHES_BETWEEN_ARTIFACT_GROUPS);
                    break;
                case 22: // PGP
                    distanceToMoveFromShootingLine = INCHES_FROM_SHOOTING_LINE_TO_FIRST_ARTIFACTS + (1 * INCHES_BETWEEN_ARTIFACT_GROUPS);
                    break;
                case 23: // PPG
                    distanceToMoveFromShootingLine = INCHES_FROM_SHOOTING_LINE_TO_FIRST_ARTIFACTS + (0 * INCHES_BETWEEN_ARTIFACT_GROUPS);
                    break;
                default:
                    distanceToMoveFromShootingLine = INCHES_FROM_SHOOTING_LINE_TO_FIRST_ARTIFACTS + (0 * INCHES_BETWEEN_ARTIFACT_GROUPS);
                    break;
            }
            moveRight(distanceToMoveFromShootingLine, DRIVE_POWER);

            // Move forward to pick up artifacts
            moveForward(INCHES_FORWARD_TO_GRAB_ARTIFACTS, DRIVE_POWER);

            // Back up same distance (call moveForward but with negative inches)
            moveForward(-INCHES_FORWARD_TO_GRAB_ARTIFACTS, DRIVE_POWER);

            // Move left (call moveRight but with negative inches) back to the shooting line the same distance we moved away from it earlier
            moveRight(-distanceToMoveFromShootingLine, DRIVE_POWER);

            // Turn 45 degrees left (towards the red goal - would be better to do this with April Tag)
            turnLeft45();

            // Shoot 3 times
            shoot();

            // Turn 45 degrees right (to face the release level) - Again, better to orient from red goal April Tag if possible
            turnRight45();

            // Move right to be facing forward in the direction of the release lever in preparation for TeleOp
            moveRight(INCHES_FROM_SHOOTING_LINE_TO_RELEASE_LEVER, DRIVE_POWER);

            // Other thoughts...
            // Need to see if teammate has an auto and what path they are taking so we don't run into them
            // Before ending auto, maybe line up with the next row of available artifacts for a quick start?  Or in line with the release lever to clear the classifier

        }

        limelight.stop();
    }

    // --- Helper Functions ---
    private void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");
        shoot = hardwareMap.get(DcMotor.class, "shooter");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    private void setMotorDirections() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shoot.setDirection(DcMotor.Direction.REVERSE);

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

    private void initializeLimelight() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

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

    private void getLatestLimelightInfo() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int foundTagId = fr.getFiducialId();
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", foundTagId, fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                if (foundTagId == 21 || foundTagId == 22 || foundTagId == 23)
                {
                    obeliskTagID = foundTagId;
                }

                telemetry.addData("Obelisk Tag ID:", obeliskTagID);
            }
        }
        else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
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

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && // this chunk is basically telling the robot to keep going until it reaches its target.
                frontLeft.isBusy() &&
                backLeft.isBusy() &&
                frontRight.isBusy() &&
                backRight.isBusy()) {
            // Optional: telemetry here
            // telemetry.addData("Shoot Speed", shoot.getPower());
            // telemetry.update();
        }

        stopMotors();
    }

    private void moveRight(double inches, double power) {
        // Need to figure out how to move right
        int ticks = (int)(inches * TICKS_PER_INCH); // this is saying basically how many inches we want to move multiplied by the ticks per inch. this calculates how many ticks in total we would have to move.

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks); // lines 66-69 is saying "start from current pos and add the amount of ticks you want to move".
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); // just go to the amount of ticks you want
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && // this chunk is basically telling the robot to keep going until it reaches its target.
                frontLeft.isBusy() &&
                backLeft.isBusy() &&
                frontRight.isBusy() &&
                backRight.isBusy()) {
            // Optional: telemetry here
            // telemetry.addData("Shoot Speed", shoot.getPower());
            // telemetry.update();
        }

        stopMotors();
    }

    // this turning function is very basic right now, but you get the idea. :)
    // also remember that TURN_POWER is defined at the top as a constant so it can be easily reused
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

        shoot.setPower(0.53);
        if (shoot.getPower() == 0.53){
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
}