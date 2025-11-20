package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name="TeleOp", group = "TeleOp")
public class WorkingTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
   // private DcMotor intake = null;
    private CRServo indexer1 = null;
    private CRServo indexer2 = null;
    private DcMotorEx shooter = null;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        indexer1.setDirection(CRServo.Direction.FORWARD);
        indexer2.setDirection(CRServo.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //wait for start (then player presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; //remember that Y stick value is reversed!!
            double x = gamepad1.right_stick_x * 1.1; //counteract thingy
            double rx = gamepad1.left_stick_x;

            double shooterPower = gamepad2.right_trigger;

            //denominator is max power, for right now we changed it from 1 to 0.5, or absolute value

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            shooterPower = Math.min(shooterPower, 0.58);

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backRightPower);
            backRightDrive.setPower(backLeftPower);

//            if(gamepad1.right_bumper){
//                intake.setPower(1);
//            }
//
//            else {
//                intake.setPower(0);
//
//            }
//
//            if(gamepad1.left_bumper){
//                intake.setPower(-1);
//            }
//
//            else {
//                intake.setPower(0);
//            }
//
            if(gamepad2.left_bumper) {
                indexer1.setPower(1);
                indexer2.setPower(1);
            }
            else {
                indexer1.setPower(0);
                indexer2.setPower(0);
            }

            if(gamepad2.right_trigger > 0) {
                shooter.setMotorEnable();
                shooter.setVelocity(1500);
            }
            else {
                shooter.setVelocity(0);
                shooter.setMotorDisable();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Shoot Speed", shooter.getPower());
            telemetry.update();
        }
    }

}

