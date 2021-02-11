/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "FL_DCmotor"
 * Motor channel:  Right drive motor:        "FR_DCmotor"
 * Motor channel:  Manipulator drive motor:  "RL_DCmotor"
 * Motor channel:  Manipulator drive motor:  "RR_DCmotor"
 * Motor channel:  Manipulator drive motor:  "IntakeMotor"
 * Motor channel:  Manipulator drive motor:  "TransferMotor"
 * Servo channel:  Servo to open left claw:  "WobbleGoalArmDrive"
 * Servo channel:  Servo to open right claw: "WobbleGoalClaw"
 */
public class HardwareBIGBRAINBOTS
{
    /* Public OpMode members. */
    public DcMotor  FrontLeftDrive   = null;
    public DcMotor  FrontRightDrive  = null;
    public DcMotor  RearLeftDrive     = null;
    public DcMotor RearRightDrive  =  null;
    public DcMotor IntakeTransferDrive  =  null;
    public DcMotor ShooterFlywheel = null;
    public DcMotor    WobbleGoalArmDrive    = null;
    public Servo    WobbleGoalClaw   = null;
    public Servo   ShooterPush = null;


    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBIGBRAINBOTS(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
       // Define and Initialize Motors
        FrontLeftDrive  = hwMap.get(DcMotor.class, "FL_DCmotor");
        FrontRightDrive = hwMap.get(DcMotor.class, "FR_DCmotor");
        RearLeftDrive = hwMap.get(DcMotor.class, "RL_DCmotor");
        RearRightDrive = hwMap.get(DcMotor.class, "RR_DCmotor");
        IntakeTransferDrive = hwMap.get(DcMotor.class, "IntakeTransferMotor");
        ShooterFlywheel = hwMap.get(DcMotor.class, "ShooterDrive");
        WobbleGoalArmDrive  = hwMap.get(DcMotor.class, "arm_motor");


        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        IntakeTransferDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);
        IntakeTransferDrive.setPower(0);
        WobbleGoalArmDrive.setPower(0);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WobbleGoalArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGoalArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        WobbleGoalClaw = hwMap.get(Servo.class, "arm_claw");
        ShooterPush = hwMap.get(Servo.class, "Shooter_Arm");
    }
    public void drive (double power, int EncoderCounts){
        FrontLeftDrive.setTargetPosition(EncoderCounts);
        FrontRightDrive.setTargetPosition(EncoderCounts);
        RearLeftDrive.setTargetPosition(EncoderCounts);
        RearRightDrive.setTargetPosition(EncoderCounts);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        RearLeftDrive.setPower(power);
        RearRightDrive.setPower(power);
    }
    public void strafe (double power, int EncoderCounts){
        FrontLeftDrive.setTargetPosition(-1 * EncoderCounts);
        FrontRightDrive.setTargetPosition(EncoderCounts);
        RearLeftDrive.setTargetPosition(EncoderCounts);
        RearRightDrive.setTargetPosition(-1 * EncoderCounts);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setPower(-1 * power);
        FrontRightDrive.setPower(power);
        RearLeftDrive.setPower(power);
        RearRightDrive.setPower(-1 * power);
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", 0,power);
    }
    public void turn (double power, int EncoderCounts){
        FrontLeftDrive.setTargetPosition(-1 * EncoderCounts);
        FrontRightDrive.setTargetPosition(EncoderCounts);
        RearLeftDrive.setTargetPosition(-1 * EncoderCounts);
        RearRightDrive.setTargetPosition(EncoderCounts);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setPower(-1 * power);
        FrontRightDrive.setPower(power);
        RearLeftDrive.setPower(-1 * power);
        RearRightDrive.setPower(power);
    }
    public void resetEncoders (){
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGoalArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}