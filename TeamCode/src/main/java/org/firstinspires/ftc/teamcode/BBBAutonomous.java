package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareBIGBRAINBOTS;

@Autonomous (name = "BBBAutonomous")
public class BBBAutonomous extends LinearOpMode {
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();
        robot.drive(1,1000);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Over", "0");
        telemetry.update();


    }
}
