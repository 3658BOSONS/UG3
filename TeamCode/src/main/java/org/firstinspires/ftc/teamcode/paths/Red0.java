package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.PurePursuit;

import java.util.ArrayList;

public class Red0
{

    private ArrayList<PurePursuit> paths;

    public Red0()
    {
        paths = new ArrayList<>();

        ArrayList<MovementPoint> points1 = new ArrayList<>();
        points1.add(new MovementPoint(900, 1300, 0));
        points1.add(new MovementPoint(2700, 1000, 0));
        points1.add(new MovementPoint(2650, 550, Math.toRadians(-135), 50));
        paths.add(new PurePursuit(points1));

        ArrayList<MovementPoint> points2 = new ArrayList<>();
        points2.add(new MovementPoint(2400, 900, 0));
        points2.add(new MovementPoint(1200, 500, 0));
        points2.add(new MovementPoint(1000, 350, Math.toRadians(-170), 15));
        PurePursuit path2 = new PurePursuit(points2);
        paths.add(path2);

        ArrayList<MovementPoint> points3 = new ArrayList<>();
        points3.add(new MovementPoint(800, 800, 0));
        points3.add(new MovementPoint(1650, 600, Math.toRadians(-30), 50));
        paths.add(new PurePursuit(points3));
    }

    public ArrayList<PurePursuit> getPaths(){
        return paths;
    }

}
