package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// made for blue goal auto aim.  will need to think about red goal
@TeleOp (name="WorkingTeleOp", group = "TeleOp")
public class WorkingTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private ShooterMcGavin shooter;

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

        shooter = new ShooterMcGavin(hardwareMap, telemetry);

        // wait for start (then player presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; //remember that Y stick value is reversed!!
        double x = gamepad1.right_stick_x * 1.1; //counteract thingy
        double rx = gamepad1.left_stick_x;

        //denominator is max power, for right now we changed it from 1 to 0.5, or absolute value
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backRightPower);
        backRightDrive.setPower(backLeftPower);

        if(gamepad2.right_trigger > 0 && !shooter.IsShooting) { // Shooting from zone closest to goal.  uses default 1200 velocity
            shooter.startShooting();
        }

        if (gamepad2.left_trigger > 0 && !shooter.IsShooting) { // experiment - shooting from far zone if left trigger pressed???
            shooter.startShooting(1400);
        }

        if (gamepad2.bWasPressed && !shooter.IsShooting) { // if artifact fed in incorrect chute, reverse feed to pull the jammed artifact in the opposite direction
            shooter.reverseFeed();
        }

        shooter.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }
}
