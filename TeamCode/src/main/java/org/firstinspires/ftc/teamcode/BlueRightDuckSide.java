/*
/*
https://github.com/Rambotics/FTC-2016-2017-v2.4-pc/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
https://github.com/pmtischler/ftc_app/tree/master/SharedCode/src/main/java/com/github/pmtischler
Code Folding Expand/Collapse ALL => Control || Shift || +/-
Last Edit Location => Control + Shift + BackSpace
Add Bookmark => Control + F11 + number
Find Bookmark => Control + number
Show Bookmarks => Shift + F11
Jump to Declaration => Control + B
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//===========================================
// ***** Section 1          *****
// ***** Change OpMode Name *****
//===========================================

@Autonomous(name = "BlueRightDuckSide", group = "11697")

public class BlueRightDuckSide extends LinearOpMode {

    Hardware robot = new Hardware();

    String duckpos = "";

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    private static final String VUFORIA_KEY =
            "Ab26zCH/////AAABmQY7P/AwHEvwuxSSSU8Qvolut+JL87PhjCoN41zFYiW0440dcuirvdo21YYOP2c+JLr1KsEU/jnNlbfTLD7aYFajUrEUJVrMFXFgjMTwQwZthdeC6kKe2LIUP+I7z/PmaEaSxqkkFbZ03CEpyEIQhcRBRLVOwWY4U2cKaZd6eBmxouJoQARBrhbfuF/dQ/CMfkxQyQCKshm8dc+KDKho9RToHR3naPCzjdXxEweJU2phrUM8p2bc2DQJg9paMmJ7Hhg5o0Xr97FoZd2bLvlonteZPcqLeHMlj+zmApsU8x5iABLHrS11OUPkKRdJP1RGw7qkpLyB+t+pZAsejgiYsws9oiBLkKFLpCcT+p7zp8p7";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck"

    };


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        scan(tfod);
        sleep(500);

        scan(tfod);
        telemetry.addData("case:", duckpos);
        telemetry.addData("DONEDONEDONEDONE", "DONEDONEDONEIT'SDONEE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            scan(tfod);

            robot.FlipperJohn.setPosition(0.673);

            if (duckpos.equals("left")) {
                driveByEncoder(0.5, -15, -15);
                sleep(50);
                driveByEncoder(1, 5, 5);
                driveByEncoder(0.5, 3, 3);
                sleep(50);
                driveByEncoder(0.75, -5.45, 5.45);
                driveByEncoder(.75, -5.2, -5.2);
                place();

                driveByEncoder(0.75, -2.6, 2.6);
                driveByEncoder(1, 10.15, 10.15);
                driveByEncoder(0.5, 6.5, 6.5);
                driveByEncoder(0.25, 2.5, 2.5);
                //turn duck carousel
                turnDuck("blue");

            } else if (duckpos.equals("mid")) {
                driveByEncoder(0.5, -15, -15);
                sleep(50);
                driveByEncoder(1, 5, 5);
                driveByEncoder(0.5, 3, 3);
                sleep(50);
                driveByEncoder(0.75, -5.25, 5.25);
                driveByEncoder(0.75, -5.5, -5.5);
                place();

                driveByEncoder(0.75, -2.6, 2.6);
                driveByEncoder(1, 9.75, 9.75);
                driveByEncoder(0.5, 7.65, 7.65);
                driveByEncoder(0.25, 2.3, 2.3);
                //turn duck carousel
                turnDuck("blue");
            } else {
                driveByEncoder(1, -2, -2);
                driveByEncoder(0.75, -4.5, 4.5);
                driveByEncoder(1, -8, -8);

                //don't need switch since place() changes depending on duckpos
                place();

                driveByEncoder(0.75, -4, 4);
                driveByEncoder(0.75, 15.2, 15.2);
                driveByEncoder(0.4, 5.35, 5.35);
                //turn duck carousel
                turnDuck("blue");

            }

            driveByEncoder(1,-4,-4);
            driveByEncoder(1,-3.55,3.55);
            driveByEncoder(1, -2, -2, "1");
            driveByEncoder(0.75,0,7.25, "2");

//            elaborate parking, doesn't work for unknown reasons(robot just stops randomly)
            driveByEncoder(0.75, 2, 0, "3");
            driveByEncoder(0.5,-5,-5, "4");
            driveByEncoder(0.75,0,5, "5");

            driveByEncoder(0.75,2,0, "6");
            driveByEncoder(1,2.2,2.2, "7");
            driveByEncoder(0.75,0,2, "8");
            sleep(100);

//            elaborate parking end

            driveByEncoder(0.6,14.8,0);
            driveByEncoder(0.75,9,9);

            break;

        }

    }

    private void scan(TFObjectDetector tfod) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                if (updatedRecognitions.size() == 0) {
                    duckpos = "left";
                }
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (updatedRecognitions.size() == 1) {
                        if (recognition.getRight() < 300) {
                            duckpos = "mid";
                        } else {
                            duckpos = "right";
                        }
                        i++;
                    }
                    telemetry.addData("case:", duckpos);
                    telemetry.addData("DONEDONEDONEDONE", "DONEDONEDONEIT'SDONEE");
                    telemetry.update();

                }
            }
        }
    }

    public void turnDuck(String color) {
        double pow = 0;
        if (color.equals("blue")) {
            pow = 0.7;
        } else if (color.equals("red")) {
            pow = -0.7;
        }
        robot.Sheesh.setPower(pow);
        sleep(3000);
        robot.Sheesh.setPower(0);
    }


    public void place() {
        int vatorpos = 1610;

        if (duckpos.equals("right")) {
            vatorpos = 1650;
        } else if (duckpos.equals("mid")) {
            vatorpos = 705;
        } else if (duckpos.equals("left")) {
            robot.Output.setPosition(0.512);
            sleep(2000);
            robot.Output.setPosition(0.331);
            sleep(15);
            return;
        }

        int startPosition = robot.VatorBoi.getCurrentPosition();

        if (duckpos.equals("right")) {
            //go up
            robot.VatorBoi.setTargetPosition(vatorpos);
            while (robot.VatorBoi.getCurrentPosition() <= startPosition + 435) {
                robot.VatorBoi.setPower(0.6);
            }
            robot.Output.setPosition(0.525);
            while (robot.VatorBoi.getCurrentPosition() <= robot.VatorBoi.getTargetPosition()) robot.VatorBoi.setPower(0.7);
            robot.VatorBoi.setPower(0);

            sleep(1500);
            //go down
            robot.Output.setPosition(0.331);

            robot.VatorBoi.setTargetPosition(startPosition);
            while (robot.VatorBoi.getCurrentPosition() >= robot.VatorBoi.getTargetPosition()) robot.VatorBoi.setPower(0.45);

            robot.VatorBoi.setPower(0);
        }

        if (duckpos.equals("mid")) {
            robot.Output.setPosition(0.525);
            robot.VatorBoi.setTargetPosition(vatorpos);
            while (robot.VatorBoi.getCurrentPosition() <= robot.VatorBoi.getTargetPosition()) {
                robot.VatorBoi.setPower(0.6);
            }
            robot.VatorBoi.setPower(0);

            sleep(1500);

            //lower
            robot.Output.setPosition(0.331);

            robot.VatorBoi.setTargetPosition(startPosition);
            while (robot.VatorBoi.getCurrentPosition() >= robot.VatorBoi.getTargetPosition()) {
                robot.VatorBoi.setPower(0.45);
            }
            robot.VatorBoi.setPower(0);
        }

    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        //ORIGINAL CONFIDENCE 0.8
        tfodParameters.minResultConfidence = 0.45f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    private void driveByEncoder(double speed, double leftInches, double rightInches) throws InterruptedException {
        /**********************************************************
         driveByEncoder(0.3, 10.0, 10.0);            // Forward
         driveByEncoder(0.3, -10.0, -10.0);          // Backward
         driveByEncoder(0.3, 0, 10.0);               // Shift Right
         driveByEncoder(0.3, 10.0, 0);               // Shift Left
         driveByEncoder(0.3, 22.0, -22.0);           // Turn Right
         driveByEncoder(0.3, -22.0, 22.0);           // Turn Left
         ***********************************************************/
        speed /=2;
        leftInches /=2;
        rightInches /=2;
        String robotAction = "";
        int newLeftTarget;
        int newRightTarget;

        if (leftInches < 0 && rightInches < 0) {
            robotAction = "BACKWARD";
        } else if (leftInches > 0 && rightInches > 0) {
            robotAction = "FORWARD";
        } else if (leftInches > 0 && rightInches == 0) {
            robotAction = "SHIFT_LEFT";
        } else if (leftInches == 0 && rightInches > 0) {
            robotAction = "SHIFT_RIGHT";
        } else if (leftInches < 0 && rightInches > 0) {
            robotAction = "TURN_LEFT";
        } else if (leftInches > 0 && rightInches < 0) {
            robotAction = "TURN_RIGHT";
        } else {
            return;
        }

        // Remember current motors direction, will reset in the end
        DcMotor.Direction dirFL = robot.frontLeftMotor.getDirection();
        DcMotor.Direction dirFR = robot.frontRightMotor.getDirection();
        DcMotor.Direction dirRL = robot.rearLeftMotor.getDirection();
        DcMotor.Direction dirRR = robot.rearRightMotor.getDirection();
        DcMotor.RunMode runModeFL = robot.frontLeftMotor.getMode();
        DcMotor.RunMode runModeFR = robot.frontRightMotor.getMode();
        DcMotor.RunMode runModeRL = robot.rearLeftMotor.getMode();
        DcMotor.RunMode runModeRR = robot.rearRightMotor.getMode();

        DcMotor[] motors = {robot.frontLeftMotor, robot.frontRightMotor, robot.rearLeftMotor, robot.rearRightMotor};
        for (DcMotor m: motors) {
            m.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        }
        for (DcMotor m: motors) {
            // power is removed from the motor, set the current encoder position to zero
            m.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }

        // All motors will move forward
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
        //logMessage("curFL,curFR",  robot.frontLeftMotor.getCurrentPosition() +", "+ robot.frontRightMotor.getCurrentPosition());

        switch (robotAction) {

            case "FORWARD":
                //logMessage("Moving Robot", "FORWARD");
                break;

            case "BACKWARD": // motor direction aame as FORWAED, because encoder will be "-"
                //logMessage("Moving Robot", "BACKWARD");
                break;

            case "SHIFT_LEFT":
                //logMessage("Moving Robot", "SHIFT_LEFT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //+
                robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //+
                robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //+
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //+
                //robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                newRightTarget = newLeftTarget;
                break;

            case "SHIFT_RIGHT":
                //logMessage("Moving Robot", "SHIFT_RIGHT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //-
                robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //-
                robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //-
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //-
                //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                newLeftTarget = newRightTarget;
                break;

            case "TURN_LEFT":
                //logMessage("Moving Robot", "TURN_LEFT");
                break;

            case "TURN_RIGHT":
                //logMessage("Moving Robot", "TURN_RIGHT");
                break;

        }

        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
        robot.frontRightMotor.setTargetPosition(newRightTarget);
        robot.rearLeftMotor.setTargetPosition(newLeftTarget);
        robot.rearRightMotor.setTargetPosition(newRightTarget);
        //logMessage("newLeftTarget,newRightTarget",  newLeftTarget +", "+ newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the motor speed and start motion
        robot.frontLeftMotor.setPower(Math.abs(speed));
        robot.frontRightMotor.setPower(Math.abs(speed));
        robot.rearLeftMotor.setPower(Math.abs(speed));
        robot.rearRightMotor.setPower(Math.abs(speed));


        //Ramp up motor speed to match target
//        while(power <= speed) {
//            power += RAMP_INCREMENT;
//        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {

            /*
            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
            logMessage("Path2",
                    robot.frontLeftMotor.getCurrentPosition() + ", " +
                    robot.frontRightMotor.getCurrentPosition() + ", " +
                            robot.rearLeftMotor.getCurrentPosition() + ", " +
                            robot.rearRightMotor.getCurrentPosition());
            */
        }


        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset back motors direction
        robot.frontLeftMotor.setDirection(dirFL);
        robot.frontRightMotor.setDirection(dirFR);
        robot.rearLeftMotor.setDirection(dirRL);
        robot.rearRightMotor.setDirection(dirRR);
        robot.frontLeftMotor.setMode(runModeFL);
        robot.frontRightMotor.setMode(runModeFR);
        robot.rearLeftMotor.setMode(runModeRL);
        robot.rearRightMotor.setMode(runModeRR);

    }

    private void driveByEncoder(double speed, double leftInches, double rightInches, String debug) throws InterruptedException {
        telemetry.addData("Drive Began", debug);
        telemetry.update();
        driveByEncoder(speed, leftInches, rightInches);
        telemetry.addData("Drive Completed", debug);
        telemetry.update();
    }

}
