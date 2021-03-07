package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.PurePursuit;

import java.util.ArrayList;

public class Red4
{

    private ArrayList<PurePursuit> paths;

    public Red4()
    {
        paths = new ArrayList<>();

        ArrayList<MovementPoint> points1 = new ArrayList<>();
        points1.add(new MovementPoint(1700, 1300, 0));
        points1.add(new MovementPoint(2600, 1600, 0));
        points1.add(new MovementPoint(3100, 850, Math.toRadians(-45), 50));
        paths.add(new PurePursuit(points1));

        ArrayList<MovementPoint> points2 = new ArrayList<>();
        points2.add(new MovementPoint(2900, 600, 0));
        points2.add(new MovementPoint(1500, 300, 0));
        points2.add(new MovementPoint(900, 375, Math.toRadians(180), 15));
        paths.add(new PurePursuit(points2));

        ArrayList<MovementPoint> points3 = new ArrayList<>();
        points3.add(new MovementPoint(900, 400, 0));
        points3.add(new MovementPoint(600, 850, Math.toRadians(0), 25));
        paths.add(new PurePursuit(points3));

        ArrayList<MovementPoint> points4 = new ArrayList<>();
        points4.add(new MovementPoint(600, 850, 0));
        points4.add(new MovementPoint(950, 850, Math.toRadians(0), 25));
        paths.add(new PurePursuit(points4));

        ArrayList<MovementPoint> points5 = new ArrayList<>();
        points5.add(new MovementPoint(950, 850, Math.toRadians(0),0));
        points5.add(new MovementPoint(1050, 850, Math.toRadians(0),25));
        paths.add(new PurePursuit(points5));

        ArrayList<MovementPoint> points6 = new ArrayList<>();
        points6.add(new MovementPoint(1800, 1000, 0));
        points6.add(new MovementPoint(2800, 900, 0));
        points6.add(new MovementPoint(3100, 800, Math.toRadians(-45), 25));
        paths.add(new PurePursuit(points6));
    }

    public ArrayList<PurePursuit> getPaths(){
        return paths;
    }

}
