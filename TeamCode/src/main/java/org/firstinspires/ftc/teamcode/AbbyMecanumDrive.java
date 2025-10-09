package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Abby Mecanum Drive", group = "linear OpMode"  )

public class AbbyMecanumDrive extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor wheelSpin = null;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        wheelSpin = hardwareMap.get(DcMotor.class, "lift");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //wait for start (then player presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; //remember that Y stick value is reversed!!
            double x = gamepad1.left_stick_x * 1.1; //counteract thingy
            double rx = gamepad1.right_stick_x;

            //denominator is max power aka 1 or absolute value

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backRightPower);
            backRightDrive.setPower(backLeftPower);

            if(gamepad1.right_bumper) {
                wheelSpin.setPower(1);
            }

            else {
                wheelSpin.setPower(0);
            }

//            if (gamepad1.right_bumperWasPressed) {
//                wheelSpin.setPower(1);
//            }
//
//            else {
//                wheelSpin.setPower(0);
//            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();

        }
}

    }
