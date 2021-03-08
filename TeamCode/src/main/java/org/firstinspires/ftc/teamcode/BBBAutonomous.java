package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.annotation.Target;
import java.util.List;

import static java.lang.Math.abs;

@Autonomous (name = "BBBAutonomous")
public class BBBAutonomous extends LinearOpMode {
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware
    //tensorflow stuff
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AbhyEVr/////AAABmZM8yCLGH0QTjtItBa5rMYRy02Ki/kP8EoxIHn4Ppn0WctnbSjg1lXOvzubEKaIMeNZa6Nm888zj2QpeyEnhHmrMhDQZdMc8Bm2edRuJHZQElwdFrJNIPSw79HQlZ9ZcNDrWCJRqKrnqVwhxiASqbE6/tbTMTZbL7p/ImK9VdkhTajlntfNZYe1NZy75vq1CrtIHfn66TXK61TBTxITKcuT/m/zAWFLcxfv3f7SvdmaQEtvVBqXwyiDp/z2I+9gCcah7h9VFSrjVsbhsXKTRXef/PTnWTvy5RLYC5eM1u77PC+9ugrD6a/6lHAs+r9rOnTEkSNsNmDcXReKQqIpOkFwZLZpdhPD3LPzAYjnIWuwS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //tensorflow stuff end

    //imu stuff
    private BNO055IMU imu;
    static final double TURN_SPEED = 0.75;
    static final double P_TURN_COEFF_1 = 0.025;
    static final double P_TURN_COEEF_2 = 0.0035;
    static final double HEADING_THRESHOLD = 0.5;
    //imu stuff end

    static final double COUNTS_PER_INCH = 43.5; //handy for using inches

    @Override
    public void runOpMode() throws InterruptedException {
        String TargetZone = "nothing";
        String getTargetZone;
        initVuforia(); //for tensorflow
        initTfod(); //for tensorflow
        //gets all the hardware from HARDWAREBIGBRAINBOTS
        robot.init(this.hardwareMap);

        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
  /*      telemetry.addData("Third angle", "%.1f", orientation.thirdAngle);
        telemetry.update(); */
        //initialize IMU end

        telemetry.addData("Mode", "waiting");
        telemetry.update();
        if (tfod != null) { //tensorflow stuff
            tfod.activate();
            tfod.setZoom(2.0, 1.50);
        }
        waitForStart();
//        robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION + 0.2);
        //      sleep(500);
        //      robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION - 0.2); //open the claw
        //      sleep(30000);
        robot.drive(-0.75, -(int) (17.5 * COUNTS_PER_INCH));
        telemetry.addData("drive", "finished");
        telemetry.update();
        gyroTurn(TURN_SPEED, -40, P_TURN_COEFF_1);
        telemetry.addData("turn", "finished");
        telemetry.update();
        getTargetZone = objectDetection(TargetZone);
        gyroTurn(TURN_SPEED, 0, P_TURN_COEFF_1);
        robot.drive(-0.75, -439); //1200 encoder counts (original) - 5 inches
        robot.strafe(-0.75, -300);
        robot.ShooterFlywheel.setPower(1);
        sleep(500);
        //robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        // sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);//shoot
        sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        robot.strafe(-0.75, 300);
        sleep(200); //wait for the shooter arm back to the position
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1); //shoot
        sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        robot.strafe(-0.75, 300);
        sleep(200);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.05); //shoot
        sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        //     sleep(750);

        robot.strafe(-1, -300);
        telemetry.addData("Target Zone", getTargetZone);
        telemetry.update();
        if (getTargetZone.equals("A")) {
            robot.drive(-0.75, -3000);
            gyroTurn(TURN_SPEED, 90, P_TURN_COEEF_2);
            robot.init(this.hardwareMap);
            robot.drive(0.75, (int) (38 * COUNTS_PER_INCH));
        } else if (getTargetZone.equals("B")) {
            robot.drive(-0.75, -3000);
            robot.strafe(0.75, (int) (38 * COUNTS_PER_INCH));
        } else {
            robot.drive(-0.75, -3000);
            gyroTurn(TURN_SPEED, -90, P_TURN_COEEF_2);
            robot.init(this.hardwareMap);
            robot.drive(-0.75, -(int) (40 * COUNTS_PER_INCH));
        }
        robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION + 0.1);//
        sleep(500);
        robot.WobbleGoalArmDrive.setTargetPosition(-300);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(-0.05);
        while (abs(robot.WobbleGoalArmDrive.getCurrentPosition() - (-300)) > 80) {

        }
        robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION - 0.4); //open the claw
        sleep(2000);
        robot.WobbleGoalArmDrive.setPower(0);
        robot.WobbleGoalArmDrive.setTargetPosition(0);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(0.50);
        while(robot.WobbleGoalArmDrive.isBusy()) {

        }
        if (getTargetZone.equals("A")) {
            robot.drive(-1, -(int)(24 * COUNTS_PER_INCH));
            robot.strafe(-1,(int)(24 * COUNTS_PER_INCH));
        } else if (getTargetZone.equals("B")) {
            robot.drive(1, (int)(24 * COUNTS_PER_INCH));
        } else {
            robot.strafe(1,(int)(30 * COUNTS_PER_INCH));
        }
        telemetry.addData("Over", "0");
        telemetry.update();
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void gyroTurn(double speed, double angle, double coeff) {
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        boolean onTarget = false;
        error = getError(angle);
        while (Math.abs(error) > HEADING_THRESHOLD) {
            steer = Range.clip(coeff * error, -speed, speed);
            rightSpeed = steer;
            leftSpeed = -rightSpeed;
            robot.FrontLeftDrive.setPower(leftSpeed);
            robot.RearLeftDrive.setPower(leftSpeed);
            robot.FrontRightDrive.setPower(rightSpeed);
            robot.RearRightDrive.setPower(rightSpeed);

 /*           telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.4f:%5.4f", leftSpeed, rightSpeed); */
            error = getError(angle);
        }
        robot.FrontLeftDrive.setPower(0);
        robot.FrontRightDrive.setPower(0);
        robot.RearLeftDrive.setPower(0);
        robot.RearRightDrive.setPower(0);
  /*      telemetry.addData("turn","stopped");
        telemetry.update();   */
    }

    public double getError(double targetAngle) {
        double angleError;
        Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angleError = targetAngle - orientation.thirdAngle;

        if (angleError > 100) {
            angleError = angleError - 360;
        }
        if (angleError <= -180) {
            angleError = angleError + 360;
        }
        return angleError;
    }
    /*
    //this is my stuff

    private void waitForMotor() throws InterruptedException{
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
    }

     */

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId",
                    "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private String objectDetection (String TargetZone) {
        if (opModeIsActive()) {
            int count=0;
            while (opModeIsActive() && count<=10) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if(updatedRecognitions.size()==0){
                            TargetZone = "A";
                            telemetry.addData("TFOD", "No items detected");
                            telemetry.addData("Target Zone","A");
                        }else{
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                if(recognition.getLabel().equals("Single")){
                                    TargetZone = "B";
                                    telemetry.addData("Target Zone","B");
                                }else if(recognition.getLabel().equals("Quad")){
                                    TargetZone = "C";
                                    telemetry.addData("Target Zone","C");
                                }else{
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                }
                            }
                        }
                        telemetry.addData("count= ", "%d", count);
                        count++;
                        telemetry.update();

                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    return TargetZone;
    }

}
