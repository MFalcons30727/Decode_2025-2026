package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Motor Test", group = "linearOpMode" )
@Disabled
public class Intake extends LinearOpMode {

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                motor1.setPower(1);
            }
            if (gamepad1.left_bumper) {
                motor2.setPower(1);
            }
        }
    }
}
