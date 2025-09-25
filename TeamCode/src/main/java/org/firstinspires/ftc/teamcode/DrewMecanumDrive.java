package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DrewMecanumDrive", group="Linear OpMode")
public class DrewMecanumDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;

    private DcMotor rearLeft = null;

    private DcMotor frontRight = null;

    private DcMotor rearRight = null;

    private DcMotor topMotor = null;

    @Override
    public void runOpMode()  {

        frontLeft = hardwareMap.get(DcMotor.class,  "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        topMotor = hardwareMap.get(DcMotor.class,  "topMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double rearLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double rearRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);


            if(gamepad1.right_bumper) {
                topMotor.setPower(1);
            }

            else {
                topMotor.setPower(0);
            }

            telemetry.addData("Status", "Runtime: " + runtime.toString());
            telemetry.addData("Front left/right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Front left/right", "%4.2f, %4.2f", rearLeftPower, rearRightPower);
            telemetry.update();


        }
    }
}
