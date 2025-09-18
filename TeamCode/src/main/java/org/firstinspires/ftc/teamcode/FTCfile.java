package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor;

public class FTCfile extends LinearOpMode {
    DcMotor right_front;
    DcMotor left_front;
    DcMotor left_rear;
    DcMotor right_rear;
    double turn;
    double forward;
    double strafe;
    public void initialsetup(){
        right_front.setDirection.(DcMotor.Direction.REVERSE);
        right_rear.setDirection.(DcMotor.Direction.REVERSE);
        left_front.setDirection.(DcMotor.Direction.FORWARD);
        left_rear.setDirection.(DcMotor.Direction.FORWARD);
    }
    public void gamepadDrive(){
        while(opModeIsActive()){
            turn = gamepad1.right_stick_x;
            forward = gamepad1.right_stick_y;
            strafe = gamepad1.left_stick_x;

            //TODO processDriveinputs();
        }

    }
    @Override
    public void runOpMode{} throws InterruptedException {

    }
}
