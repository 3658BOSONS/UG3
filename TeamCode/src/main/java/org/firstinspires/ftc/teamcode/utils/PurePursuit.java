package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

import java.util.ArrayList;

public class PurePursuit //class declaration
{

    private ArrayList<MovementPoint> points; //array of points
    private int currentPoint; //current point in iteration
    private int currentPointHeading;

    private final int lookahead = 400;
    private final int lookaheadHeading = 600;

    public PurePursuit(ArrayList<MovementPoint> points)
    {
        this.points = points;
        this.currentPoint = 1;
        this.currentPointHeading = 1;
    }

    public boolean purePursuit(Drivetrain dt, State state, OpMode op, double speed, double turnSpeed, int reverse)
    {
        op.telemetry.addLine("point: " + currentPoint);//printing current point to phone
        if(currentPoint == points.size() - 1) //Last point
        {
            RobotMovement.setPoint(points.get(currentPoint), speed, turnSpeed); //set point to last point
            return RobotMovement.driveTowardPoint(dt, state, op.telemetry, true, reverse);//drive towards last point and return result
        }
        else
        {
            double[] intersections1 = getIntersections(points.get(currentPoint - 1), points.get(currentPoint), dt.getPosition().x, dt.getPosition().y, lookahead); //first intersect
            double[] intersections2 = getIntersections(points.get(currentPoint), points.get(currentPoint + 1), dt.getPosition().y, dt.getPosition().y, lookahead); //second intersect

            double[] intersections1theta = getIntersections(points.get(currentPointHeading - 1), points.get(currentPointHeading), dt.getPosition().x, dt.getPosition().y, lookaheadHeading); //first intersect
            double[] intersections2theta = getIntersections(points.get(currentPointHeading), points.get(currentPointHeading + 1), dt.getPosition().y, dt.getPosition().y, lookaheadHeading); //second intersect

            double x = 0;
            double y = 0;
            if(intersections2[0] > -1 || intersections2[2] > -1)
            {
                currentPoint++;
            }
            else if(intersections1[0] > -1 || intersections1[2] > -1)
            {
                if((Math.abs(intersections1[0] - points.get(currentPoint).getX()) < Math.abs(intersections1[2] - points.get(currentPoint).getX()) && intersections1[0] > -1) || intersections1[2] <= -1)
                {
                    x = intersections1[0];
                    y = intersections1[1];
                }
                else
                {
                    x = intersections1[2];
                    y = intersections1[3];
                }
            }
            else
            {
                x = points.get(currentPoint).getX();
                y = points.get(currentPoint).getY();
            }

            double headingX = 0;
            double headingY = 0;
            if(intersections2theta[0] > -1 || intersections2theta[2] > -1)
            {
                currentPointHeading++;
            }
            else if(intersections1theta[0] > -1 || intersections1theta[2] > -1)
            {
                if((Math.abs(intersections1theta[0] - points.get(currentPointHeading).getX()) < Math.abs(intersections1theta[2] - points.get(currentPointHeading).getX()) && intersections1theta[0] > -1) || intersections1theta[2] <= -1)
                {
                    headingX = intersections1[0];
                    headingY = intersections1[1];
                }
                else
                {
                    headingX = intersections1[2];
                    headingY = intersections1[3];
                }
            }
            else
            {
                headingX = points.get(currentPoint).getX();
                headingY = points.get(currentPoint).getY();
            }
            RobotMovement.setPoint(new MovementPoint(x, y, points.get(currentPoint).getTheta(), 0, headingX, headingY), speed, turnSpeed);
        }
        RobotMovement.driveTowardPoint(dt, state, op.telemetry,false, reverse);
        return false;
    }

    public int getCurrentPoint(){
        return currentPoint;
    }

    public double[] getIntersections(MovementPoint p1, MovementPoint p2, double x, double y, double r) //x1,y1,x2,y2;
    {//math what are you a nerd haha
        double x1 = p1.getX() - x;
        double x2 = p2.getX() - x;
        double y1 = p1.getY() - y;
        double y2 = p2.getY() - y;

        double m = (y2 - y1) / (x2 - x1);
        double sqrt = -(m * m * x1 * x1) + (2 * m * x1 * y1) + (m * m * r * r) + (r * r) - (y1 * y1);

        if(sqrt < 0){
            return new double[]{-1, -1, -1, -1};
        }

        double xi1 = ((m * m * x1) - (m * y1) + Math.sqrt(sqrt)) / (m * m + 1);
        double xi2 = ((m * m * x1) - (m * y1) - Math.sqrt(sqrt)) / (m * m + 1);

        double yi1 = m*(xi1 - x1) + y1 + y;
        double yi2 = m*(xi2 - x1) + y1 + y;
        xi1 += x;
        xi2 += x;
        //THE KNIGHTS OF LAMBDA WILL RISE
        if(!((xi1 - x >= x1 && xi1 - x <= x2) || (xi1 - x <= x1 && xi1 - x >= x2)))
        {
            xi1 = -1;
            yi1 = -1;
        }
        if(!((xi2 - x >= x1 && xi2 - x <= x2) || (xi2 - x <= x1 && xi2 - x >= x2)))
        {
            xi2 = -1;
            yi2 = -1;
        }

        return new double[]{xi1, yi1, xi2, yi2};
    }

}