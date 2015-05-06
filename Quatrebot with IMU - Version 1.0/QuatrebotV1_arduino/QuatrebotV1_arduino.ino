#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <SpeedTrig.h>
#include <Servo.h>
#include <PID_v1.h>



#define  NUM_POINTS_TRAJECTORY    70
#define  D1  130.0
#define  D2  260.0
#define  NUM_VIA_POINTS  5
#define  NUM_SEGMENTS    NUM_VIA_POINTS-1
#define  MIN_Q1_SPEED  0
#define  MAX_Q1_SPEED  184
#define  MIN_Q2  12
#define  MAX_Q2  155
#define  START_Q2  143
#define  Q1_ZERO 92
#define  DT_PID_Q1DOT  20
#define  DT_PID_Q1    50
#define  DT_PID_Q2DOT  30
#define  DT_PID_Q2    10
#define  DT_TRAJECTORY  100
#define  PRINT_VIA_POINTS  1
#define  PRINT_TRAJECTORY  1
#define  PRINT_STATE    1


unsigned long lastGyroSampling = 0;
int gyroSampleTime = 10000;
int gyroSampleNum = 100;
int gyroZ_dc_offset = 0;
double dt_gyro = 0;
L3G gyro;
unsigned long lastCompassSampling = 0;
int compassSampleTime = 50;
LSM303 compass;
double q1 = 0;
double initial_q1_compass = 87.92;
double q1_compass = 0;
double q1_gyro = 0;
double k_complementary_filter = 0.3;
double q2 = 0;
double q1_desired = 0.0;
double q1dot_desired = 0;
double q1dot = 0;
double cos_q2 = 0;
double temp1_q1 = 0;
double temp2_q1 = 0;
double xEF = 0;
double yEF = 0;
double xEF_desired = 0;
double yEF_desired = 0;


double x_via_points_world[NUM_VIA_POINTS] = {200, 300, 380, 20, 380};
double y_via_points_world[NUM_VIA_POINTS] = {80, 20, 320, 380, 380};
double x_via_points_robot[NUM_VIA_POINTS];
double y_via_points_robot[NUM_VIA_POINTS];
double origin_robot[2] = {132.0, 202.0};
double x_trajectory[NUM_POINTS_TRAJECTORY + 1];
double y_trajectory[NUM_POINTS_TRAJECTORY + 1];
unsigned long lastPointTime = 0;
double tolerancia_distancia = 20;
double distancia_punto = 0;

int point_iterator = 0;
Servo q1_servo;
double q1_servoSpeed = 0;
Servo q2_servo;
int q2_servo_position = 0;




PID PID_q1(&q1, &q1dot_desired, &q1_desired, 0.0, 0.0, 0.0, REVERSE);
PID PID_q1dot(&q1dot, &q1_servoSpeed, &q1dot_desired, 0.0, 0.0, 0.0, REVERSE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.begin();
  compass.init();
  compass.enableDefault();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();

  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  //min: {  -289,   -426,   -222}    max: {  +436,   +231,   +453}
  compass.m_min = (LSM303::vector<int16_t>) {
    -289, -426, -222
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +436, +231, +453
  };
  // put your setup code here, to run once:
  TranslateViaPoints();
  GenerateTrajectory();
  q1_servo.write(Q1_ZERO);
  q1_servo.attach(6);
  q2_servo.attach(5);
  PID_q1.SetSampleTime(DT_PID_Q1);
  PID_q1dot.SetSampleTime(DT_PID_Q1DOT);
  PID_q1.SetOutputLimits(-10000, 10000);
  PID_q1dot.SetOutputLimits(-MAX_Q1_SPEED / 2, MAX_Q1_SPEED / 2);
  PID_q1.SetMode(AUTOMATIC);
  PID_q1.SetTunings(23, 0, 0);
  PID_q1dot.SetMode(AUTOMATIC);
  PID_q1dot.SetTunings(0.015, 0, 0);
  delay(2000);
  CalculateGyroDCOffset();
  //    EstimateInitialHeadingQ1();
  Serial.println("\n Robot ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  for (i = 0; i < 50 ; i++)
  {
    EstimateHeadingQ1();
  }

  q2_servo_position = map(int(160), 0, 180, MIN_Q2, MAX_Q2);
  q2_servo.write(q2_servo_position);
  delay(1000);

  point_iterator = 2;
  while (point_iterator < NUM_POINTS_TRAJECTORY)
  {
    if ((millis() - lastPointTime >= DT_TRAJECTORY) && (distancia_punto < tolerancia_distancia))
    {
      xEF_desired = x_trajectory[point_iterator];
      yEF_desired = y_trajectory[point_iterator];
      InverseKinematics();
      point_iterator++;
      lastPointTime = millis();
    }

    EstimateHeadingQ1();
    ComputePIDs();
    MoveMotors();
    //        Serial.println("Position of joint 1");
    //        Serial.println(SpeedTrig.cos(q1) * D1 + origin_robot[0]);
    //        Serial.println(SpeedTrig.sin(q1) * D1 + origin_robot[1]);

    ForwardKinematics();
    distancia_punto = sqrt((xEF - xEF_desired)*(xEF - xEF_desired) + (yEF - yEF_desired)*(yEF - yEF_desired));

    Serial.print("Trajectory point #");
    Serial.print(point_iterator);
    Serial.print(":\n (");
    //      Serial.print(xEF_desired);
    Serial.print(xEF_desired + origin_robot[0]);
    Serial.print(", ");
    //      Serial.print(yEF_desired);
    Serial.print(yEF_desired + origin_robot[1]);
    Serial.println(")");
    Serial.print("Inverse kinematics of point #");
    Serial.print(point_iterator);
    Serial.println(":");
    Serial.println(q1_desired);
    Serial.println(q2);
    Serial.println("End effector position:");
    Serial.print("(");
    //    Serial.print(xEF);
    Serial.print(xEF + origin_robot[0]);
    Serial.print(", ");
    //    Serial.print(yEF);
    Serial.print(yEF + origin_robot[1]);
    Serial.println(")");
    Serial.println("Arm state");
    Serial.println(q1);
    //    Serial.println(q1dot);
    Serial.println(q2);
    //  Serial.println(gyroZ_dc_offset);
    //  Serial.println("Values computed by the q1 PID");
    //  Serial.println(q1dot_desired);
    //  Serial.println(int(q1_servoSpeed));
    //  Serial.println();
  }

}

// Reference: http://web.csulb.edu/~hill/ee444/Labs/5%20Gyro/5%20Arduino%20IDE%20Gyro.pdf
void CalculateGyroDCOffset()
{
  for (int n = 0; n < gyroSampleNum; n++) {
    gyro.read();
    gyroZ_dc_offset += (int)gyro.g.z;
  }
  gyroZ_dc_offset = gyroZ_dc_offset / gyroSampleNum;
}





void EstimateInitialHeadingQ1()
{
  lastCompassSampling = millis();
  compass.read();
  initial_q1_compass = double(compass.heading());
  Serial.println("Initial heading");
  Serial.println(initial_q1_compass);
}





void EstimateHeadingQ1()
{
  if (millis() - lastCompassSampling > compassSampleTime)
  {
    compass.read();
    //    q1 = initial_q1_compass - (compass.heading() - 180.0);
    //    q1 = (compass.heading() - 180.0);
    q1_compass = compass.heading();
    q1 = SpeedTrig.atan2(SpeedTrig.sin(initial_q1_compass - q1_compass), SpeedTrig.cos(initial_q1_compass - q1_compass)) * (180.0 / 3.1415);

    lastCompassSampling = millis();
  }
  if (micros() - lastGyroSampling > gyroSampleTime)
  {
    dt_gyro = double(micros() - lastGyroSampling) / 1000000.0;
    gyro.read();
    q1dot = double(gyro.g.z - gyroZ_dc_offset) / 100.0;
    lastGyroSampling = micros();
  }
}




void ComputePIDs()
{
  if(abs(q1_desired) > 160)
  {
    if (q1 * q1_desired < 0)
    {
      PID_q1.SetControllerDirection(DIRECT);
    }
    else
    {
      PID_q1.SetControllerDirection(REVERSE);
    }
  }
  else
  {
    PID_q1.SetControllerDirection(REVERSE);
  }
  PID_q1.Compute();
  PID_q1dot.Compute();
}




void MoveMotors()
{
  // Reference #1: http://www.epanorama.net/newepa/2013/04/11/rc-servo-modification-for-contiunous-rotation/
  // Reference #2: http://forum.arduino.cc/index.php?topic=280717.0
  q1_servoSpeed = q1_servoSpeed + double(MAX_Q1_SPEED / 2);
  if (q1_servoSpeed > 180)
  {
    q1_servoSpeed = 180;
  }
  q1_servo.write(int(q1_servoSpeed));
  q2_servo_position = map(int(180 - q2), 0, 180, MIN_Q2, MAX_Q2);
  q2_servo.write(q2_servo_position);
}




void ForwardKinematics()
{
  xEF = SpeedTrig.cos(q1 + q2) * D2 + SpeedTrig.cos(q1) * D1;
  yEF = SpeedTrig.sin(q1 + q2) * D2 + SpeedTrig.sin(q1) * D1;
}





void InverseKinematics()
{
  cos_q2 = double((xEF_desired * xEF_desired + yEF_desired * yEF_desired - D1 * D1 - D2 * D2) / (2.0 * D1 * D2));
  q2 = double(SpeedTrig.acos(cos_q2)) * (180.0 / 3.1415);
  //  q2 = SpeedTrig.atan2( sqrt(1-cos_q2*cos_q2), cos_q2);
  temp1_q1 = (SpeedTrig.atan2(yEF_desired, xEF_desired) - SpeedTrig.atan2(SpeedTrig.sin(q2) * D2, cos_q2 * D2 + D1)) * (180.0 / 3.1415);
  //  temp2_q1 = (SpeedTrig.atan2(yEF_desired, -sqrt(D1 * D1 + D2 * D2 + 2 * D1 * D2 * cos_q2 - yEF_desired * yEF_desired)) - SpeedTrig.atan2(SpeedTrig.sin(q2) * D2, cos_q2 * D2 + D1)) * (180.0 / 3.1415);

  //  if (abs(q1 - temp1_q1) > abs(q1 - temp2_q1))
  //  {
  //    q1_desired = temp2_q1;
  //  }
  //  else
  //  {
  q1_desired = temp1_q1;
  //  }

  //  q2 = q2 * (180.0 / 3.1415);
}




int sign(int value)
{
  return ((value > 0) - (value < 0));
}

void TranslateViaPoints()
{
  int i;
  for (i = 0; i < NUM_VIA_POINTS; i++)
  {
    x_via_points_robot[i] = x_via_points_world[i] - origin_robot[0];
    y_via_points_robot[i] = y_via_points_world[i] - origin_robot[1];
    if (PRINT_VIA_POINTS)
    {
      Serial.print("x = ");
      Serial.print(x_via_points_robot[i]);
      Serial.print(";  y = ");
      Serial.print(y_via_points_robot[i]);
      Serial.println();
    }
  }
}




void GenerateTrajectory()
{
  double distance_segment_via_points[NUM_VIA_POINTS];
  double total_distance_via_points = 0;
  int i, j;
  for (i = 0; i < NUM_SEGMENTS; i++)
  {
    distance_segment_via_points[i] = sqrt((x_via_points_robot[i + 1] - x_via_points_robot[i]) * (x_via_points_robot[i + 1] - x_via_points_robot[i]) + (y_via_points_robot[i + 1] - y_via_points_robot[i]) * (y_via_points_robot[i + 1] - y_via_points_robot[i]));
    total_distance_via_points += distance_segment_via_points[i];
    Serial.print("Segment #");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(distance_segment_via_points[i]);
    Serial.println();
  }


  Serial.print("Total distance = ");
  Serial.print(total_distance_via_points);
  Serial.println();

  int points_per_segment[NUM_SEGMENTS];
  int total_points = 0;
  for (i = 0; i < NUM_SEGMENTS ; i++)
  {
    points_per_segment[i] = int(double(distance_segment_via_points[i] / double(total_distance_via_points) * double(NUM_POINTS_TRAJECTORY)));

    if (double(distance_segment_via_points[i] / double(total_distance_via_points) * double(NUM_POINTS_TRAJECTORY)) - int(double(distance_segment_via_points[i] / double(total_distance_via_points) * double(NUM_POINTS_TRAJECTORY))) > 0.5)
    {
      points_per_segment[i] += 1;
    }
    Serial.println(double(distance_segment_via_points[i] / double(total_distance_via_points) * double(NUM_POINTS_TRAJECTORY)));
    Serial.print("Points in segment #");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(points_per_segment[i]);
    Serial.println();

    total_points += points_per_segment[i];
  }


  Serial.print("Total number of points = ");
  Serial.print(total_points);
  Serial.println();
  double dr_trajectory = NUM_POINTS_TRAJECTORY / total_distance_via_points;
  int points_calculated = 0;
  double s[NUM_SEGMENTS];

  for (i = 0; i < NUM_SEGMENTS ; i++)
  {
    s[i] = 1.0 / double(points_per_segment[i]);

    for (j = 0; j < points_per_segment[i] ; j++)
    {
      x_trajectory[points_calculated + j] = x_via_points_robot[i] * (1 - s[i] * j) + s[i] * j * x_via_points_robot[i + 1];
      y_trajectory[points_calculated + j] = y_via_points_robot[i] * (1 - s[i] * j) + s[i] * j * y_via_points_robot[i + 1];

      if (points_calculated + j == total_points - 1)
      {
        points_calculated += 1;
        x_trajectory[points_calculated + j] = x_via_points_robot[i + 1];
        y_trajectory[points_calculated + j] = y_via_points_robot[i + 1];
      }

    }
    points_calculated += j;
  }

  if (PRINT_TRAJECTORY)
  {

    Serial.print("Differential of cartesian movement (dR) = ");
    Serial.print(dr_trajectory);
    Serial.println();
    Serial.println();
    Serial.println("*** Trajectory ***");

    points_calculated = 0;
    for (i = 0; i < NUM_SEGMENTS ; i++)
    {
      Serial.print("s = ");
      Serial.println(s[i]);
      for (j = 0; j < points_per_segment[i] ; j++)
      {
        Serial.print("Point #");
        Serial.print(points_calculated + j);
        Serial.print(":  x = ");
        Serial.print(x_trajectory[points_calculated + j]);
        Serial.print(";  y = ");
        Serial.print(y_trajectory[points_calculated + j]);
        Serial.println();
        if (points_calculated + j == total_points - 1)
        {
          points_calculated += 1;
          if (PRINT_TRAJECTORY)
          {
            Serial.print("Point #");
            Serial.print(points_calculated + j);
            Serial.print(":  x = ");
            Serial.print(x_trajectory[points_calculated + j]);
            Serial.print(";  y = ");
            Serial.print(y_trajectory[points_calculated + j]);
            Serial.println();
          }
        }
      }
      points_calculated += j;
    }
    Serial.println("*** End of Trajectory ***");
  }
}
