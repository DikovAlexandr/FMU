package com.fmu.test;

import no.ntnu.ihb.fmi4j.importer.fmi2.Fmu;
// import no.ntnu.ihb.fmi4j.importer.fmi2.FmuSlave;
import no.ntnu.ihb.fmi4j.importer.fmi2.CoSimulationSlave;

import no.ntnu.ihb.fmi4j.modeldescription.ModelDescription;
import no.ntnu.ihb.fmi4j.modeldescription.variables.Causality;
import no.ntnu.ihb.fmi4j.modeldescription.variables.ModelVariables;
import no.ntnu.ihb.fmi4j.modeldescription.variables.ScalarVariable;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

public class FmuTester {

    private final String fmuPath;
    private Fmu fmu;
    CoSimulationSlave slave;
    private ModelDescription modelDescription;

    private long[] inVrs;
    private long[] outVrs;
    private String[] outNames;

    long[] vrs_ = new long[10];
    long[] inVrs_ = new long[10];

    private String[] inNames;
    private Map<String,Integer> name2outIdx;
    private Map<String,Integer> name2inIdx;

    public FmuTester(String fmuPath) {
        this.fmuPath = fmuPath;
    }

    public void init() throws Exception {
        File fmuFile = new File(fmuPath);
        if (!fmuFile.exists()) {
            throw new IOException("FMU file not found: " + fmuPath);
        }

        fmu = Fmu.from(fmuFile);
        modelDescription = fmu.getModelDescription();
        slave = fmu.asCoSimulationFmu().newInstance();
        slave.simpleSetup();

        System.out.println("Loaded FMU: " + modelDescription.getModelName());

        var vars = slave.getModelVariables();
        int size = vars.getSize();

        int localCount = 0;
        int outputCount = 0;
        int inputCount = 0;

        List<Long> tmpOutVrs = new ArrayList<>();
        List<String> tmpOutNames = new ArrayList<>();
        List<Long> tmpInVrs = new ArrayList<>();
        List<String> tmpInNames = new ArrayList<>();

        for (int i = 0; i < size; i++) {
            var sv = vars.get(i);
            if (sv.getCausality() == Causality.LOCAL) {
                localCount++;
                tmpOutVrs.add(sv.getValueReference());
                tmpOutNames.add(sv.getName());
            }
            if (sv.getCausality() == Causality.OUTPUT) {
                outputCount++;
                tmpOutVrs.add(sv.getValueReference());
                tmpOutNames.add(sv.getName());
            }
            if (sv.getCausality() == Causality.INPUT
            || sv.getCausality() == Causality.PARAMETER) {
                inputCount++;
                tmpInVrs.add(sv.getValueReference());
                tmpInNames.add(sv.getName());
            }
        }

        System.out.println("INVAR: " + tmpInNames);

        int outCount = localCount + outputCount;
        int inCount = inputCount;

        vrs_ = new long[outCount];
        inVrs_ = new long[inCount];

        for (int i = 0; i < tmpOutVrs.size(); i++) {
            vrs_[i] = (long) tmpOutVrs.get(i);
        }

        for (int i = 0; i < tmpInVrs.size(); i++) {
            inVrs_[i] = (long) tmpInVrs.get(i);
        }

        System.out.println("inVrs: " + inVrs_);
        System.out.println("inVrs: " + tmpInVrs);

        outVrs = tmpOutVrs.stream().mapToLong(Long::longValue).toArray();
        outNames = tmpOutNames.toArray(new String[0]);

        inVrs = tmpInVrs.stream().mapToLong(Long::longValue).toArray();
        inNames = tmpInNames.toArray(new String[0]);

        name2outIdx = new HashMap<>();
        for (int i = 0; i < outNames.length; i++) {
            name2outIdx.put(outNames[i], i);
        }
        name2inIdx = new HashMap<>();
        for (int i = 0; i < inNames.length; i++) {
            name2inIdx.put(inNames[i], i);
        }

        System.out.printf("Discovered %d inputs, %d outputs (+%d locals)%n",
                          inCount, outputCount, localCount);
    }

    public void testTest() throws Exception {
        CoSimulationSlave slave = fmu.asCoSimulationFmu().newInstance();

        slave.simpleSetup();

        double[] inVals = new double[inVrs.length];

        boolean[] setFlags = new boolean[inVrs.length];

        inVals[name2inIdx.get("u1")] = 0;
        setFlags[name2inIdx.get("u1")] = true;

        System.out.println("=== Input coverage ===");
        for (int i = 0; i < inNames.length; i++) {
            System.out.printf("%-25s : %s%n",
                inNames[i],
                setFlags[i] 
                ? String.format("SET (value=%.5f)", inVals[i]) 
                : "NOT SET!");
        }

        slave.writeReal(inVrs, inVals);

        slave.exitInitializationMode();
        
        slave.doStep(0.0, 0.1);

        double[] outVals = new double[outVrs.length];
        slave.readReal(outVrs, outVals);

        System.out.println("Output:");
        double tq = outVals[name2outIdx.get("y1")];
        System.out.printf(" Joint %.3f%n", tq);

        slave.terminate();
        fmu.close();
    }

    public void testController() throws Exception {
        // try (CoSimulationSlave slave = fmu.asCoSimulationFmu().newInstance()) {
        CoSimulationSlave slave = fmu.asCoSimulationFmu().newInstance();

        slave.simpleSetup();

        double[] angles = new double[]{0.0, Math.PI/2, -Math.PI/2, 0.0, 0.0, 0.0};
        double[] velocities = new double[6];

        double[] targetAngles = new double[]{ 7.7366e-06, 1.5708e+00, -1.5708e+00, 7.7366e-06, 7.7366e-06, 7.7366e-06};
        double[] targetVel = new double[]{0.0005, -0.0009, 0.0009, 0.0005, 0.0005, 0.0005};
        double[] targetsAcc = new double[]{0.0183, -0.0366, 0.0366, 0.0183, 0.0183, 0.0183};

        double[] inVals = new double[inVrs.length];

        boolean[] setFlags = new boolean[inVrs.length];

        double kp = 3000.0;
        // inVals[name2inIdx.get("kp")] = kp;
        // setFlags[name2inIdx.get("kp")] = true;

        int idxKp = name2inIdx.get("kp");
        long vrKp = inVrs[idxKp];
        
        double kd = 10.0;
        // inVals[name2inIdx.get("kd")] = kd;
        // setFlags[name2inIdx.get("kd")] = true;

        int idxKd = name2inIdx.get("kd");
        long vrKd = inVrs[idxKd];
        
        for (int j = 0; j < 6; j++) {
            inVals[name2inIdx.get("angle_" + j)] = angles[j];
            setFlags[name2inIdx.get("angle_" + j)] = true;

            inVals[name2inIdx.get("angle_target_" + j)] = targetAngles[j];
            setFlags[name2inIdx.get("angle_target_" + j)] = true;

            inVals[name2inIdx.get("velocity_" + j)] = velocities[j];
            setFlags[name2inIdx.get("velocity_" + j)] = true;

            inVals[name2inIdx.get("velocity_target_" + j)] = targetVel[j];
            setFlags[name2inIdx.get("velocity_target_" + j)] = true;

            inVals[name2inIdx.get("acceleration_target_" + j)] = targetsAcc[j];
            setFlags[name2inIdx.get("acceleration_target_" + j)] = true;
        }

        System.out.println("=== Input coverage ===");
        for (int i = 0; i < inNames.length; i++) {
            System.out.printf("%-25s : %s%n",
                inNames[i],
                setFlags[i] 
                ? String.format("SET (value=%.5f)", inVals[i]) 
                : "NOT SET!");
        }
        
        // slave.setupExperiment();
        // slave.enterInitializationMode();

        slave.writeReal(new long[]{vrKp, vrKd}, new double[]{kp, kd});
        slave.writeReal(inVrs, inVals);

        slave.exitInitializationMode();
        
        slave.doStep(0.0, 0.01);

        double[] outVals = new double[outVrs.length];
        slave.readReal(outVrs, outVals);

        System.out.println("Torques:");
        for (int j = 0; j < 6; j++) {
            double tq = outVals[name2outIdx.get("torque_" + j)];
            System.out.printf(" Joint %d: %.3f%n", j, tq);
        }

        slave.terminate();
        fmu.close();
    }

    public void testDynamics() throws Exception {
        try (CoSimulationSlave slave = fmu.asCoSimulationFmu().newInstance()) {
            double[] angles = {0.0, Math.PI/2, -Math.PI/2, -Math.PI/2, 0.0, 0.0};
            double[] velocities = new double[6];
            double[] torques = new double[6];
    
            double[] inVals = new double[inVrs.length];
            for (int j = 0; j < 6; j++) {
                inVals[name2inIdx.get("angle_" + j)] = angles[j];
                inVals[name2inIdx.get("velocity_" + j)] = velocities[j];
                inVals[name2inIdx.get("torque_" + j)] = torques[j];
            }
    
            slave.writeReal(inVrs, inVals);
    
            slave.setupExperiment();
            slave.enterInitializationMode();
            slave.exitInitializationMode();
    
            slave.doStep(0.0, 0.05);
    
            double[] outVals = new double[outVrs.length];
            slave.readReal(outVrs, outVals);
    
            System.out.println("Dynamics outputs:");
            for (int j = 0; j < 6; j++) {
                int idxAngle = name2outIdx.get("new_angle_" + j);
                int idxVel = name2outIdx.get("new_velocity_" + j);
                int idxAcc = name2outIdx.get("acceleration_"+ j);
                System.out.printf("Joint %d: angle=%.3f, vel=%.3f, acc=%.3f%n",
                                  j,
                                  outVals[idxAngle],
                                  outVals[idxVel],
                                  outVals[idxAcc]);
            }
        }
    }

    public void testTrajectory() throws Exception {
        try (CoSimulationSlave slave = fmu.asCoSimulationFmu().newInstance()) {
            double[] startAngles = {0.0, Math.PI/2, -Math.PI/2, 0.0, 0.0, 0.0};
            double[] endAngles = {Math.PI/4, 0.0, 0.0, Math.PI/4, Math.PI/4, Math.PI/4};
            double tStart = 0.0;
            double tEnd = 5.0;
            int nPoints = 101;
            int index = 0;
    
            double[] inVals = new double[inVrs.length];
    
            for (int j = 0; j < 6; j++) {
                inVals[name2inIdx.get("start_" + j)] = startAngles[j];
                inVals[name2inIdx.get("end_" + j)] = endAngles[j];
            }
            inVals[name2inIdx.get("t_start")] = tStart;
            inVals[name2inIdx.get("t_end")] = tEnd;
            inVals[name2inIdx.get("n_points")] = nPoints;
            inVals[name2inIdx.get("index")] = index;
    
            slave.writeReal(inVrs, inVals);
    
            slave.setupExperiment();
            slave.enterInitializationMode();
            slave.exitInitializationMode();
    
            slave.doStep(0.0, 0.05);
    
            double[] outVals = new double[outVrs.length];
            slave.readReal(outVrs, outVals);
    
            System.out.println("Trajectory point " + index + ":");
            for (int j = 0; j < 6; j++) {
                double q = outVals[name2outIdx.get("q_" + j)];
                double qd = outVals[name2outIdx.get("qd_" + j)];
                double qdd = outVals[name2outIdx.get("qdd_" + j)];
                System.out.printf(
                    " Joint %d: q=%.3f, qd=%.3f, qdd=%.3f%n",
                    j, q, qd, qdd
                );
            }
        }
    }

    public void close() {
        if (fmu != null) {
            fmu.close();
        }
    }
} 