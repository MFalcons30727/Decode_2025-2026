package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MecanumBasicStuff", group = "TeleOp")
public class MecanumBasicStuff extends OpMode {
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

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

    }

    @Override
    public void loop() {
        // -1 to 1

        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(-1);
        backLeftDrive.setPower(-1);
        backRightDrive.setPower(1);

        //sleep(1000);


//        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
//        double joyX = gamepad1.left_stick_x;
//
//        double frontLeftPower = joyY + joyX;
//        double backLeftPower = joyY - joyX;
//        double frontRightPower = joyY - joyX;
//        double backRightPower = joyY + joyX;
//
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;
//
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//        //frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
//        //backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
//        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
//        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
//    }
    }
}

