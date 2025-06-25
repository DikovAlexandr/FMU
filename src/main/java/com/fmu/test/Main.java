package com.fmu.test;

public class Main {
    public static void main(String[] args) {
        if (args.length < 2) {
            System.out.println("Usage: java -jar fmu-test.jar <fmu_path> <type>");
            System.out.println("type: controller, dynamics, or trajectory");
            return;
        }

        String fmuPath = args[0];
        String type = args[1].toLowerCase();

        FmuTester tester = new FmuTester(fmuPath);
        try {
            tester.init();
            switch (type) {
                case "controller":
                    tester.testController();
                    break;
                case "dynamics":
                    tester.testDynamics();
                    break;
                case "trajectory":
                    tester.testTrajectory();
                    break;
                default:
                    System.out.println("Unknown FMU type: " + type);
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            tester.close();
        }
    }
} 