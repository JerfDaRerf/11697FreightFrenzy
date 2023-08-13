package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



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
 ***  Hardware Motor Controllers Settings ***
 RL Controller (AL025T7Z)
 0 - NeveRest 40 Gearmotor (FLMotor)
 RR Controller (AL025T80)
 1 - NeveRest 40 Gearmotor (FRMotor)
 FL Controller (A7008KTV)
 2 - NeveRest 40 Gearmotor (LRMotor)
 FR Controller (A7008KBB)
 3 - NeveRest 40 Gearmotor (RRMotor)
 */
public class Hardware  {


    /* Public Motors */
    public DcMotor frontLeftMotor;          // CR Hub port 0     FL
    public DcMotor frontRightMotor;         // CR Hub port 1     FR
    public DcMotor rearLeftMotor;           // CR Hub port 2     RL
    public DcMotor rearRightMotor;          // CR Hub port 3     RR
    public DcMotor VatorBoi;                // EX Hub port 0     Elevator(NeveRest 20 Gearmotor)
    public DcMotor Sheesh;                  // EX Hub port 1     Sheesh(NeveRest 40 Gearmotor)
    public DcMotor StraightPulley;          // EX Hub port 2     StraightPulley(NeveRest 40 Gearmotor)

    public CRServo Intake;                  // CR Hub port 0     Intake
    public Servo Output;                    // CR Hub port 1     Output
    public Servo FlipperJohn;               // CR Hub port 2     FlipperJohn
    public Servo BanHammer;                 // Ex Hub port 0     BanHammer
    public Servo TightSqueezer;             // EX Hub port 1     TightSqueezer

    BNO055IMU imu;
    /* local OpMode members. */
    HardwareMap hwMap;
    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("FL");
        frontRightMotor = hwMap.dcMotor.get("FR");
        rearLeftMotor   = hwMap.dcMotor.get("RL");
        rearRightMotor  = hwMap.dcMotor.get("RR");
        VatorBoi        = hwMap.dcMotor.get("Elevator");
        Sheesh          = hwMap.dcMotor.get("Sheesh");
        StraightPulley  = hwMap.dcMotor.get("StraightPulley");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VatorBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Sheesh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StraightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VatorBoi.setTargetPosition(-70);
        VatorBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Sheesh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StraightPulley.setTargetPosition(-7250);
        StraightPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        VatorBoi.setPower(0);
        Sheesh.setPower(0);

        // IntakePush.setPower(0);

        //Servo config
        Intake         = hwMap.get(CRServo.class, "Intake");
        Output         = hwMap.get(Servo.class, "Output");
        FlipperJohn    = hwMap.get(Servo.class, "FlipperJohn");
        BanHammer      = hwMap.get(Servo.class, "BanHammer");
        TightSqueezer  = hwMap.get(Servo.class, "TightSqueezer");


        //SENSOR CONFIG
        //cantTouchThis = hwMap.get(TouchSensor.class,"cantTouchThis");


    }
}