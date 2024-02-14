package frc.robot.sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class DistanceCalcs {

    private static final Comparator<DataPoint> compPoint = Comparator.comparingDouble(p -> p.cameraDistance);

    private ShuffleboardTab tab = Shuffleboard.getTab("Wheel Speed Calculations");
  private int CurrentOffset = 0;


    private class DataPoint {
        double cameraDistance;
        double usableDistance;

        public DataPoint(double cam, double real) {
            cameraDistance = cam;
            this.usableDistance = real;
        }

        public DataPoint(String point) {
            if (point.contains(",")) {
                String[] parts = point.split(",");
                cameraDistance = Double.parseDouble(parts[0]);
                usableDistance = Double.parseDouble(parts[1]);
            }
        }

        public void AddReal(double real) {
            usableDistance += real;
        }

        @Override
        public String toString() {
            return "(" + cameraDistance + "," + usableDistance + ")";
        }

        @SuppressWarnings("unused")
        public boolean equals(DataPoint other) {
            return cameraDistance == other.cameraDistance;
        }
    }


    private ArrayList<DataPoint> data_MainStorage;
    private LinearInterp distCurve;
    private String FILEPATH;


    public DistanceCalcs() {
        FILEPATH = Filesystem.getDeployDirectory() + File.separator + "curves";


        FILEPATH = FILEPATH + File.separator + "distCurve.csv";



        data_MainStorage = new ArrayList<>();
        readFile();
        distCurve = null;
        loadCurve();
    }

    public void loadCurve() {
        ArrayList<Double> data_Dist = new ArrayList<>();
        ArrayList<Double> data_Speed = new ArrayList<>();

        for (DataPoint pt : data_MainStorage) {
            data_Dist.add(pt.cameraDistance);
            data_Speed.add(pt.usableDistance);
        }

        distCurve = new LinearInterp(data_Dist, data_Speed);
    }
    
    public void readFile() {
        data_MainStorage.clear();

        try (BufferedReader br = new BufferedReader(new FileReader(FILEPATH))) {
            for (String line; (line = br.readLine()) != null; ) {
                if (!line.equals("")) {
                    data_MainStorage.add(new DataPoint(line));
                }
            }
        }
        catch (IOException e) {
            data_MainStorage.addAll(List.of(
                    new DataPoint(0, 0),
                    new DataPoint(0,  0)));
            DriverStation.reportError("Could not read CSV, defaulting", false);
        }

        data_MainStorage.sort(compPoint);
        loadCurve();
    }

    public double getREALDistanceToTarget(Double camDist) {
        return distCurve.getY(camDist);
    }

}

