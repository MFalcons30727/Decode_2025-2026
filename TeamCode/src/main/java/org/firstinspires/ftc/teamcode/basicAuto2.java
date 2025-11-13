package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red far auto", group="Autonomous")
public class basicAuto2 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constants
    private static final double TICKS_PER_INCH = 50; // I am not sure the ticks per inch, go over calculations, this is placeholder
    private static final double FORWARD_DISTANCE_INCHES = 70; // change at practice, this is just a random placeholder for now
    private static final double DRIVE_POWER = 0.8; // how fast we want the robot for now
    private static final double TURN_POWER = 0.4; // how fast it will turn
    private DcMotor shoot = null;
    private CRServo indexer1 = null;
    private CRServo indexer2 = null;

    @Override
    public void runOpMode() {

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");
        shoot = hardwareMap.get(DcMotor.class, "shooter");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");

        setMotorDirections();

        // Reset encoders
        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            // steps being called
            moveForward(FORWARD_DISTANCE_INCHES, DRIVE_POWER);
            turnRight45();
            shoot();
        }
    }

    // --- Helper Functions ---

    private void setMotorDirections() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shoot.setDirection(DcMotor.Direction.REVERSE);
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

        }

        stopMotors();
    }
    // this turning function is very basic right now, but you get the idea. :)
    // also remember that turn_power is defined at the top as 0.5 so it can be easily reused, and it is constant
    private void turnRight45() {
        // Simple time-based turn
        frontLeft.setPower(-TURN_POWER);
        backLeft.setPower(-TURN_POWER);
        frontRight.setPower(TURN_POWER);
        backRight.setPower(TURN_POWER);
        //this is currently time based which is not ideal, but it works for now
        sleep(4000); // adjust for ~45° turn

        stopMotors();
    }


    private void shoot() {

        shoot.setPower(0.55);
        if (shoot.getPower() == 0.55){
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