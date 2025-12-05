package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="MotorTest", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    private DcMotor shoot = null;
    private DcMotor intake = null;

    @Override
    public void runOpMode() {

        shoot = hardwareMap.get(DcMotor.class, "shoot");
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.left_bumper) {
                shoot.setPower(1);
            } else {
                shoot.setPower(0);
            }

            if (gamepad2.right_bumper){
                intake.setPower(1);
            }

            else {

                intake.setPower(0);
            }

        }

    }

}