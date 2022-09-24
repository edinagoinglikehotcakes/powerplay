package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class AutoA extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        //run("motorBackLeft");
        //run("motorBackRight");
        run("motorFrontLeft");
        //run("motorFrontRight");
    }
    void run(String deviceName){
        DcMotor motorTest = hardwareMap.get(DcMotor.class, deviceName);
        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest.setTargetPosition(10000);
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setPower(0.5);
        while(motorTest.isBusy()) {
            telemetry.addData(deviceName, motorTest.getCurrentPosition());
            telemetry.update();
        }
        motorTest.setPower(0);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































