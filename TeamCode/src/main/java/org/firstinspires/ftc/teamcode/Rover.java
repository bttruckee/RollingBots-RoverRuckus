package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Rover
{
    //Positions of the lock servo (closed, open)
    private static final int[] lockPositions = {0, 180};

    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    //Declaration of wheel motors
    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;

    /*
    Declaration of power set to each wheel
    0 = front left
    1 = rear left
    2 = front right
    3 = rear right
    */
    private double[] wheelPower = new double[4];

    /*private double frontLeftPower;
    private double rearLeftPower;
    private double frontRightPower;
    private double rearRightPower;*/

    //Declaration of arm motors
    private DcMotor clamp_arm1;
    private DcMotor clamp_arm2;

    //Declaration of arm servos
    private CRServo unobtanium_turbine;
    private Servo armLock_Left;
    private Servo armLock_Right;

    public Rover()
    {}

    public void init(HardwareMap ahwMap)
    {
        //Set hwMap to parameter
        hwMap = ahwMap;

        //Set motor/servo variables to motors/servos in hwMap
        left_front = hwMap.get(DcMotor.class, "left_front");
        left_rear = hwMap.get(DcMotor.class, "left_rear");
        right_front = hwMap.get(DcMotor.class, "right_front");
        right_rear = hwMap.get(DcMotor.class, "right_rear");

        clamp_arm1 = hwMap.get(DcMotor.class, "clamp_arm_1");
        clamp_arm2 = hwMap.get(DcMotor.class, "clamp_arm_2");

        unobtanium_turbine = hwMap.get(CRServo.class, "mineral_turbine");
        armLock_Left = hwMap.get(Servo.class, "left_lock");
        armLock_Right = hwMap.get(Servo.class, "right_lock");

        //Sets motors to the correct directions
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.FORWARD);

        clamp_arm1.setDirection(DcMotor.Direction.REVERSE);
        clamp_arm2.setDirection(DcMotor.Direction.FORWARD);
        unobtanium_turbine.setDirection(CRServo.Direction.FORWARD);
        armLock_Left.setDirection(Servo.Direction.FORWARD);
        armLock_Right.setDirection(Servo.Direction.REVERSE);


        //Sets rovers power to 0
        left_front.setPower(0);
        left_rear.setPower(0);
        right_front.setPower(0);
        right_rear.setPower(0);

        clamp_arm1.setPower(0);
        clamp_arm2.setPower(0);

        /*frontLeftPower = 0;
        rearLeftPower = 0;
        frontRightPower = 0;
        rearRightPower = 0;*/
    }

    /**
     * Sets the speed corresponding to forward movement on the remote to each wheel motor
     *
      * @param speed = the speed each wheel motor will be set to (negative for reverse)
     */
    public void setForwardSpeed(double speed)
    {
        wheelPower[0] = speed;
        wheelPower[1] = speed;
        wheelPower[2] = speed;
        wheelPower[3] = speed;
    }

    /**
     * Sets the speed corresponding to right rotation on the remote to each wheel motor, overwriting the forward speed if not 0
     *
     * Precondition: setForwardSpeed() (and checkStop()) has already been activated
     * Precondition: move() has been activated between uses of this method
     *
     * @param speed = the speed the wheels will rotate to rotate the robot right
     */
    public void setTurnSpeed(double speed)
    {
        if(speed != 0)
        {
            wheelPower[0] = speed;
            wheelPower[1] = speed;
            wheelPower[2] = -speed;
            wheelPower[3] = -speed;
        }
    }

    /**
     * Overwrites all speeds defines and sets the speed of each wheel motor individually
     * Used primarily for testing
     *
     * @param lfSpeed: speed of left front wheel
     * @param lrSpeed: speed of left rear wheel
     * @param rfSpeed: speed of right front wheel
     * @param rrSpeed: speed of right rear wheel
     */
    public void setTestSpeed(double lfSpeed, double lrSpeed, double rfSpeed, double rrSpeed)
    {
        wheelPower[0] = lfSpeed;
        wheelPower[1] = lrSpeed;
        wheelPower[2] = rfSpeed;
        wheelPower[3] = rrSpeed;
    }

    /**
     * Activates the motors to move the speed the above methods called for then resets the power variables
     */
    public void move()
    {
        left_front.setPower(wheelPower[0]);
        left_rear.setPower(wheelPower[1]);
        right_front.setPower(wheelPower[2]);
        right_rear.setPower(wheelPower[3]);
    }

    //Stops the robot by reversing the power of the wheels then setting them to 0
    public void stop()
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = -wheelPower[i];
        }

        move();

        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = 0;
        }

        move();
    }

    /**
     *  Moves the motors and servos corrosponding to the rover arm
     *
     * @param armPower = The direction the arm is moving (0 = down, 1 = neutral, 2 = up)
     * @param inOrOut = The direction unobtanuim is being moved by the unobtanium turbine (0 = out, 1 = neutral, 2 = in)
     */
    public void moveArm(double armPower, int inOrOut)
    {
        clamp_arm1.setPower(armPower);
        clamp_arm2.setPower(armPower);

        if(inOrOut != 1)
        {
            if(inOrOut > 1)
            {
                unobtanium_turbine.setPower(1);
            }
            else
            {
                unobtanium_turbine.setPower(-1);
            }
        }
        else
        {
            unobtanium_turbine.setPower(0.5);
        }
    }

    public void setArmLocks(boolean locked)
    {
        if(locked)
        {
            armLock_Left.setPosition(lockPositions[0]);
            armLock_Right.setPosition(lockPositions[0]);
        }
        else
        {
            armLock_Left.setPosition(lockPositions[1]);
            armLock_Right.setPosition(lockPositions[1]);
        }
    }

    /**
     * Returns the position of the lock servo
     * @param index the servo in question (0 = left lock, 1 = right lock)
     * @return the position of the given servo
     */
    public double getLockPositions(int index)
    {
        if(index == 0)
        {
            return armLock_Left.getPosition();
        }
        return armLock_Right.getPosition();
    }
}