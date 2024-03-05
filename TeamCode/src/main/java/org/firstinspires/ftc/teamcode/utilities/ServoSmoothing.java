package org.firstinspires.ftc.teamcode.utilities;

public class ServoSmoothing {

    public static double servoSmoothing(double currPos, double targetPos){
        double smoothedPos;
        smoothedPos = (targetPos*0.07)+(currPos*0.93);
    return smoothedPos;
    }
}





