package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.PurePursuit;

import java.util.ArrayList;

public class Red1
{

    private ArrayList<PurePursuit> paths;

    public Red1()
    {
        paths = new ArrayList<>();

        ArrayList<MovementPoint> points1 = new ArrayList<>();
        points1.add(new MovementPoint(900, 1300, 0));
        points1.add(new MovementPoint(1500, 1600, 0));
        points1.add(new MovementPoint(2575, 1150, Math.toRadians(-30), 50));
        paths.add(new PurePursuit(points1));

        ArrayList<MovementPoint> points2 = new ArrayList<>();
        points2.add(new MovementPoint(2300, 800, 0));
        points2.add(new MovementPoint(1400, 265, 0));
        points2.add(new MovementPoint(925, 325, Math.toRadians(180), 15));
        paths.add(new PurePursuit(points2));

        ArrayList<MovementPoint> points3 = new ArrayList<>();
        points3.add(new MovementPoint(925, 400, 0));
        points3.add(new MovementPoint(300, 1400, Math.toRadians(0), 0));
        points3.add(new MovementPoint(1500, 1000, Math.toRadians(-15), 100));
        paths.add(new PurePursuit(points3));

        ArrayList<MovementPoint> points4 = new ArrayList<>();
        points4.add(new MovementPoint(1600, 900, 0));
        points4.add(new MovementPoint(2250, 1175, Math.toRadians(-20), 25));
        paths.add(new PurePursuit(points4));
    }

    public ArrayList<PurePursuit> getPaths(){
        return paths;
    }

}
