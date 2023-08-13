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
    //Define values for servos
    public static final double stabbyIniOpen = 0.00;


    /* Public Motors */
    public DcMotor frontLeftMotor;          // CR Hub port 0     FL
    public DcMotor frontRightMotor;         // CR Hub port 1     FR
    public DcMotor rearLeftMotor;           // CR Hub port 2     RL
    public DcMotor rearRightMotor;          // CR Hub port 3     RR
    public DcMotor VatorBoi;                // EX Hub port 2     Elevator
    public DcMotor FlipperJohn;             // EX Hub port 0     FlipperJohn
    public DcMotor Sheesh;                  // EX Hub port 1     Sheesh

    public CRServo Intake;                  // CR Hub port 0 Intake
    public Servo Output;                    // CR Hub port 1 Output

    BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

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
        FlipperJohn     = hwMap.dcMotor.get("FlipperJohn");
        Sheesh          = hwMap.dcMotor.get("Sheesh");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VatorBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlipperJohn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Sheesh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VatorBoi.setTargetPosition(100);
        VatorBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FlipperJohn.setTargetPosition(100);
        FlipperJohn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Sheesh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        VatorBoi.setPower(0);
        FlipperJohn.setPower(0);
        Sheesh.setPower(0);

        // IntakePush.setPower(0);

        //Servo config
        Intake         = hwMap.get(CRServo.class, "Intake");
        Output         = hwMap.get(Servo.class, "Output");


        //SENSOR CONFIG
        //cantTouchThis = hwMap.get(TouchSensor.class,"cantTouchThis");


    }
}