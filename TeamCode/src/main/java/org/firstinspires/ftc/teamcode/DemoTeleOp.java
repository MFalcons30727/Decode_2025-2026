package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DemoTeleOp", group = "TeleOp")
public class DemoTeleOp extends OpMode {
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx shootMotor;
    private DcMotor indexer, intake;
    private Servo hoodServo;
    private boolean slowMode = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(191,0,0,15);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
            shootMotor.setVelocity(1000);
            hoodServo.setPosition(0.5);

            if (Math.abs(shootMotor.getVelocity() - 1000) < 20) {
                indexer.setPower(1);
            }
        } else {
            shootMotor.setVelocity(0);
            indexer.setPower(0);
            hoodServo.setPosition(0);
        }

        if (gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25
            || gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        robotCentricDrive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    private void robotCentricDrive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        if (slowMode) { maxSpeed = 0.5; }

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}