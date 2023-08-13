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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.A;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

//===========================================
// ***** Section 1          *****
// ***** Change OpMode Name *****
//===========================================

@TeleOp(name = "DriveOfficial", group = "11697")
// @Disabled

public class DriveOfficial extends LinearOpMode {
    //===========================================
    // ***** Section 2                *****
    // ***** Declare global variables *****
    //===========================================

    Hardware robot = new Hardware();

    // Motors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    float driveSpeed = 1.0f;       // Driving speed rate
    int testprint = 1;
    int frontside = 1;
    int mode = 1;
    boolean wascapping = false;

    /* Declare Global Variables. */
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");

    DigitalChannel cantTouchThis;  // Hardware Device Object

    /* Insert more stuff as needed
     *  Look at https://tinyurl.com/yb2xco82 for examples */

    /***** Main Code *****/
    private boolean usingOutput = false;
    private boolean usingOutput2 = false;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("OMEGALUL", testprint);
        telemetry.update();
        robot.init(hardwareMap);

        robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

//        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        driveSpeed = 1.0f;

        logMessage("Status", "Initialized v4.0a - " + myDateFormat.format(today));

        //init

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        String PREVIOUS_MSG = "";        // MesLog
        float lxValue;
        float lyValue;
        float rxValue;
        float ryValue;

        ArrayList<Thread> runningThreads = new ArrayList();

        while (opModeIsActive()) {
            try {
                // Drive the robot
                lxValue = getJoystickValue(gamepad1.left_stick_x);
                lyValue = getJoystickValue(gamepad1.left_stick_y);
                rxValue = getJoystickValue(gamepad1.right_stick_x);
                ryValue = getJoystickValue(gamepad1.right_stick_y);
                driveByJoystick(-lxValue * frontside, -lyValue * frontside, -rxValue);

                if (gamepad1.dpad_left) frontside = 1;
                else if (gamepad1.dpad_right) frontside = -1;

                // Toggle Intake
                if (gamepad1.b) {
                    robot.FlipperJohn.setPosition(0.46);
                    robot.Intake.setPower(1);
                } else if (gamepad1.a) {
                    robot.FlipperJohn.setPosition(0.673);
                    frontside = -1;
                    robot.Intake.setPower(0);
                } else if (gamepad1.y) {
                    robot.Intake.setPower(-1);
                }

                // Elevator High
                if (gamepad1.x && !this.usingOutput) {
                    this.usingOutput = true;
                    this.driveSpeed = 0.75f;
                    Thread threadx = new Thread(() -> {
                        int startPosition= robot.VatorBoi.getCurrentPosition();
                            robot.VatorBoi.setTargetPosition(startPosition + 1610); //400, 1175, 1575
                            while (robot.VatorBoi.getCurrentPosition() <= startPosition + 435) robot.VatorBoi.setPower(0.7);
                            robot.Output.setPosition(0.512);
                            while (robot.VatorBoi.getCurrentPosition() <= robot.VatorBoi.getTargetPosition()) robot.VatorBoi.setPower(0.7);

                            frontside = 1;

                            robot.VatorBoi.setPower(0);

                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            //lower
                            robot.Output.setPosition(0.331);

                            robot.VatorBoi.setTargetPosition(startPosition);
                            while (robot.VatorBoi.getCurrentPosition() >= robot.VatorBoi.getTargetPosition()) robot.VatorBoi.setPower(0.45);

                            robot.VatorBoi.setPower(0);

                        // Reset effects / modifiers
                        this.driveSpeed = 1.0f;
                        this.usingOutput = false;
                    });
                    runningThreads.add(threadx);
                    threadx.start();
                }

                //need to change depending on red or blue side, blue side: -1, red side: 1
                if (gamepad1.right_bumper && !this.usingOutput) {
                    this.usingOutput = true;
                    Thread threadrbumper = new Thread(() -> {

                        robot.Output.setPosition(0.512);
                        robot.VatorBoi.setTargetPosition(robot.VatorBoi.getCurrentPosition() + 270);
                        while (robot.VatorBoi.getCurrentPosition() <= robot.VatorBoi.getTargetPosition()) {
                              robot.VatorBoi.setPower(0.7);
                        }
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        robot.Output.setPosition(0.331);
                        try {
                            Thread.sleep(600);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        robot.VatorBoi.setTargetPosition(robot.VatorBoi.getCurrentPosition() - 270);
                        while (robot.VatorBoi.getCurrentPosition() >= robot.VatorBoi.getTargetPosition()) {
                            robot.VatorBoi.setPower(0.45);
                        }
                        robot.VatorBoi.setPower(0);

                        this.usingOutput = false;
                    });
                    runningThreads.add(threadrbumper);
                    threadrbumper.start();
                }

                if (gamepad1.left_bumper) {
                    robot.Sheesh.setPower(0);
                } else if (gamepad1.right_trigger > 0.3) {
                    robot.Sheesh.setPower(0.7);
                } else if (gamepad1.left_trigger > 0.3) {
                    robot.Sheesh.setPower(-0.71);
                }
    /**
                if (gamepad1.dpad_up) {
                    this.usingOutput2 = true;
                    Thread threadDUp = new Thread(() -> {
                        if (wascapping) {
                            mode = -1;
                            wascapping = false;
                        }
                        if (mode == 1) {
                        //arm goes down, open clamp

                            //BanHammer down 0.3, place 0.357, back 0.831
                            //clamp close 0.304, open 0.592
                            //elevator top -2550, bottom -7250

    //                        robot.StraightPulley.setTargetPosition(-7250);
    //                        while (robot.StraightPulley.getCurrentPosition() >= robot.StraightPulley.getTargetPosition()) {
    //                            robot.StraightPulley.setPower(1);
    //                        }
    //                        robot.StraightPulley.setPower(0);

                            robot.BanHammer.setPosition(0.3);
                            try {
                                Thread.sleep(250);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            robot.TightSqueezer.setPosition(0.592);
                            mode *= -1;

                        } else if (mode == -1) {
                        //close clamp, arm goes up

    //                        robot.StraightPulley.setTargetPosition(-7250);
    //                        while (robot.StraightPulley.getCurrentPosition() >= robot.StraightPulley.getTargetPosition()) {
    //                            robot.StraightPulley.setPower(1);
    //                        }
    //                        robot.StraightPulley.setPower(0);

                            robot.TightSqueezer.setPosition(0.304);
                            try {
                                Thread.sleep(250);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            robot.BanHammer.setPosition(0.831);
                            mode *= -1;
                        }

                        this.usingOutput2 = false;
                    });
                    runningThreads.add(threadDUp);
                    threadDUp.start();
                }

                if (gamepad1.dpad_down) {
                    this.usingOutput2 = true;
                    Thread threadDUp = new Thread(() -> {
                        wascapping = true;
                        robot.StraightPulley.setTargetPosition(robot.StraightPulley.getCurrentPosition() + 4700);
                        while (robot.StraightPulley.getCurrentPosition() <= robot.StraightPulley.getTargetPosition()) {
                            robot.StraightPulley.setPower(1);
                        }
                        robot.StraightPulley.setPower(0);

                        for (int i = 0; i < 5; i++) {
                            robot.BanHammer.setPosition(avg(robot.BanHammer.getPosition(), 0.357));
                            try {
                                Thread.sleep(50);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                        robot.BanHammer.setPosition(0.357);
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        robot.TightSqueezer.setPosition(0.592);

                        //raise BanHammer
                        robot.BanHammer.setPosition(0.4);
                        robot.TightSqueezer.setPosition(0.304);
                        robot.BanHammer.setPosition(0.831);

                        robot.StraightPulley.setTargetPosition(robot.StraightPulley.getCurrentPosition() - 4700);
                        while (robot.StraightPulley.getCurrentPosition() <= robot.StraightPulley.getTargetPosition()) {
                            robot.StraightPulley.setPower(1);
                        }
                        robot.StraightPulley.setPower(0);
                    });
                    runningThreads.add(threadDUp);
                    threadDUp.start();
                }
                **/
            } catch (Exception e) {
                telemetry.addData("Exception:", e.toString());
                telemetry.update();
            }
        }
        // End of OpMode

        for (Thread v: runningThreads) {
            if (v.isAlive()) v.interrupt();
            robot.VatorBoi.setPower(0);
        }
    }


//===========================================
// ***** Section 5              *****
// ***** User Defined Functions *****
//===========================================

    private double avg(double start, double end) {
        return (start + end) / 2;
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
        speed /= 2;
        leftInches /= 2;
        rightInches /= 2;

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

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // power is removed from the motor, set the current encoder position to zero
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All mortors will move forward
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

            case "BACKWARD": // mortor direction aame as FORWAED, because encoder will be "-"
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
//        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
//                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {
//
//
//            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
//            logMessage("Path2",
//                    robot.frontLeftMotor.getCurrentPosition() + ", " +
//                    robot.frontRightMotor.getCurrentPosition() + ", " +
//                            robot.rearLeftMotor.getCurrentPosition() + ", " +
//                            robot.rearRightMotor.getCurrentPosition());
//
//        }


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

    private void driveByJoystick(double lxValue, double lyValue, double rxValue) {
        double vD = Math.sqrt(Math.pow(lxValue, 2) + Math.pow(lyValue, 2));
        //double thetaD = Math.atan2(lyValue, lxValue);
        double thetaD = Math.atan2(lxValue, -lyValue);
        double vTheta = rxValue;

        // Convert desired motion to wheel power, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(vD, thetaD, vTheta);
        robot.frontLeftMotor.setPower(wheels.frontLeft);
        robot.frontRightMotor.setPower(wheels.frontRight);
        robot.rearLeftMotor.setPower(wheels.backLeft);
        robot.rearRightMotor.setPower(wheels.backRight);

        //telemetry.addData("Status", "==> " + vD +", "+thetaD+", "+vTheta);
        //telemetry.update();
    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    private void logMessage(String myDescription, String myMessage) throws InterruptedException {

        telemetry.addData(myDescription, myMessage);
        telemetry.update();
        RobotLog.d("11697CW - " + myDescription + " : " + myMessage);

    }

    private float getJoystickValue(float rawValue) {
        //clip the power values so that it only goes from -1 to 1
        rawValue = Range.clip(rawValue, -1, 1);
        if (Math.abs(rawValue) < 0.1) {
            return 0;
        } else {
            return rawValue * driveSpeed;
        }
    }

//===========================================
// ***** Section 6              *****
// ***** Backup Un-used Code    *****
//===========================================
}


