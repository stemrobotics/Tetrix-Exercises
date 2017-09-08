DCMotor leftMotor, rightMotor;

leftMotor = hardwareMap.dcMotor.get("left_motor");
rightMotor = hardwareMap.dcMotor.get("right_motor");

leftMotor.setPower(1.0);
rightMotor.setPower(1.0);
