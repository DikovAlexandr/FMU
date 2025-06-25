# Python-Java FMU

- On Windows, the checker doesn't work (the one that is test_fmi.jar, which also has a non-working version in the repository)
- On Linux, there is no way to install libraries into the global python
- fmi4j cannot be obtained from the repository via maven, only from the repository http://10.158.223.162:8189/#/releases/no/ntnu/ihb/fmi4j
= In the repository itself (https://github.com/petercorke/robotics-toolbox-python) dependence hell, versions where not specified, somewhere limited lower, and the project is poorly supported
- The problem with the GLIBC version is that you need to create files on the same system where they will be executed, since errors with different versions will follow and next
- Just meme: https://github.com/NTNU-IHB/PythonFMU/issues/202
- Attention to numpy==1.24.4 and scipy==1.10.1

## Make and execute FMU files with python dependence

Build container:
```sh
docker build -t dev-env .
```

Run container:
```sh
docker run -it --rm -v "${PWD}:/workspace" dev-env
```

Test FMU files:
```sh
java -jar ../../lib/fmu_test.jar ControllerFMU.fmu
```

But in common case:
```sh
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpython3.10.so java -jar ../../lib/fmu_test.jar ControllerFMU.fmu
```

## FMU Tester

Build:
```bash
mvn clean package
```

Usage:
```bash
java -jar target/fmu-test-1.0-SNAPSHOT-jar-with-dependencies.jar <path_to_fmu> <type>
```
where:
- `<type>` - one of: controller, dynamics, trajectory

Examples:
```bash
java -jar target/fmu-test-1.0-SNAPSHOT-jar-with-dependencies.jar ControllerFMU.fmu controller
java -jar target/fmu-test-1.0-SNAPSHOT-jar-with-dependencies.jar DynamicsFMU.fmu dynamics
java -jar target/fmu-test-1.0-SNAPSHOT-jar-with-dependencies.jar TrajectoryFMU.fmu trajectory
```