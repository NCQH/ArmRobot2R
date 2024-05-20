#include <Servo.h>
#include <math.h>

#define SERVO1_PIN 9
#define SERVO2_PIN 10

Servo Servo1, Servo2;
const float a1 = 10.7, a2 = 8;

int cTheta1 = 0, cTheta2 = 0;
int timeBetweenWaypoints = 10;
double step = 0.4;

struct Point {
  int x;
  int y;
};
float distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}
Point waypoints[100]; // Mảng lưu trữ các điểm trung gian

void forward_kinematics(int theta1, int theta2);
void inverse_kinematics(int x, int y);
double calculateTheta2(double x, double y);
double calculateTheta1(double x, double y, double theta2);
void moveTo(int servo, int targetAngle);
void moveBoth(int theta1, int theta2);
double deg2rad(int deg);
double rad2deg(double rad);
void calculateLineWaypoints(Point startPoint, Point endPoint, int numPoints);
void moveAlongTrajectory();
void calculateCircularWaypoints(Point center, float radius, int numPoints);
void setup() {
  Serial.begin(9600);
  
  Servo1.attach(SERVO1_PIN);
  Servo2.attach(SERVO2_PIN);

  // Khởi tạo vị trí ban đầu của các servo ở góc 0 độ
  Servo1.write(0);
  Servo2.write(0);

  // Gán góc hiện tại cho biến toàn cục
  cTheta1 = 0;
  cTheta2 = 0;
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'f') {
      // Forward Kinematics
      Serial.println("Forward Kinematics");
      if (Serial.available() >= 4) {
        int number1 = Serial.parseInt();
        int number2 = Serial.parseInt();
        forward_kinematics(number1, number2);
      }
    } else if (command == 'i') {
      // Inverse Kinematics
      Serial.println("Inverse Kinematics");
      if (Serial.available() >= 4) {
        int number1 = Serial.parseInt();
        int number2 = Serial.parseInt();
        inverse_kinematics(number1, number2);
      }
    } else if (command == 'l') {
      Serial.println("Line");
      if (Serial.available() >= 8) {
        Point P1, P2;
        P1.x = Serial.parseInt();
        P1.y = Serial.parseInt();
        P2.x = Serial.parseInt();
        P2.y = Serial.parseInt();
        calculateLineWaypoints(P1, P2, 10);
        moveAlongTrajectory(10);
      }
    } else if (command == 'c') {
      Serial.println("Circle");   
      if (Serial.available() >= 6) {
        Point center;
        int radius;
        center.x = Serial.parseInt();
        center.y = Serial.parseInt();
        radius = Serial.parseInt();

        calculateCircularWaypoints(center, radius, 30);
        moveAlongTrajectory(30);
      }   
      }   
    }  
  delay(20);
}

void forward_kinematics(int theta1, int theta2) {
  moveTo(1, theta1);
  moveTo(2, theta2);

  double x = a1 * cos(deg2rad(theta1)) + a2 * cos(-deg2rad(theta1) + deg2rad(theta2));
  double y = a1 * sin(deg2rad(theta1)) - a2 * sin(-deg2rad(theta1) + deg2rad(theta2));
  Serial.print("Forward Kinematics - X = ");
  Serial.print(x);
  Serial.print(", Y = ");
  Serial.println(y);
}

void inverse_kinematics(int x, int y) {
  double theta2 = calculateTheta2(x, y);
  double theta1 = calculateTheta1(x, y, theta2);
  
  moveTo(1, rad2deg(theta1));
  moveTo(2, rad2deg(theta2));
  
  Serial.print("Inverse Kinematics - Theta1 = ");
  Serial.print(rad2deg(theta1));
  Serial.print(", Theta2 = ");
  Serial.println(rad2deg(theta2));
}

double calculateTheta2(double x, double y) {
  double theta2 = acos((x * x + y * y - (a1 * a1 + a2 * a2)) / (2.0 * a1 * a2));
  if (isnan(theta2))
  {
    return 10000;
  }
  return theta2;
}

double calculateTheta1(double x, double y, double theta2) {
  double theta1 = atan2(y, x) - atan2(-a2 * sin(theta2), a1 + a2 * cos(theta2));
  if (isnan(theta1))
  {
    return 10000;
  }
  return theta1;
}

double deg2rad(int deg) {
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

void moveTo(int servo, int targetAngle) {
  int currentAngle = (servo == 1) ? cTheta1 : cTheta2;
  
  if (currentAngle == targetAngle) return;

  int step = (targetAngle > currentAngle) ? 1 : -1;

  for (int i = currentAngle; i != targetAngle; i += step) {
    if (servo == 1) {
      Servo1.write(i);
      cTheta1 = i;
    } else {
      Servo2.write(i);
      cTheta2 = i;
    }
    delay(10);
  }
}
void moveBoth(double theta1, double theta2)
{
  if (cTheta1 == theta1 && cTheta2 == theta2) return;
  int step1 = (theta1 > cTheta1) ? 1 : -1;
  int step2 = (theta2 > cTheta2) ? 1 : -1;

  int i = cTheta1, j = cTheta2;
  while (!(i == theta1 && j == theta2))
  {
    if (i != theta1)
    {
      i++;
      Servo1.write(i);
    }
    if (j != theta2)
    {
      j++;
      Servo2.write(j);
    }
  }
  
}
void calculateLineWaypoints(Point startPoint, Point endPoint, int numPoints) {
  // Tính toán khoảng cách giữa hai điểm
  float dx = endPoint.x - startPoint.x;
  float dy = endPoint.y - startPoint.y;

  // Tính toán bước di chuyển giữa các điểm trung gian
  float stepX = dx / (numPoints - 1); // Sửa thành numPoints
  float stepY = dy / (numPoints - 1); // Sửa thành numPoints

  // Tính toán các điểm trung gian
  for (int i = 0; i < numPoints; i++) {
    waypoints[i].x = startPoint.x + stepX * i;
    waypoints[i].y = startPoint.y + stepY * i;
  }
}


void moveAlongTrajectory(int numPoints) {
  // Dựa vào quỹ đạo đã xác định, tính toán góc servo tương ứng
  for (int i = 0; i < numPoints; i++) {
    Serial.println(waypoints[i].x);
    inverse_kinematics(waypoints[i].x, waypoints[i].y);
    delay(200);
  }
}

void calculateCircularWaypoints(Point center, float radius, int numPoints) {
  // Tính toán các điểm trên quỹ đạo của đường tròn
  for (int i = 0; i < numPoints; i++) {
    // Tính toán góc của điểm trên đường tròn
    float angle = 2 * M_PI * i / numPoints;
    
    // Tính toán tọa độ x, y của điểm trên đường tròn
    int x = center.x + radius * cos(angle);
    int y = center.y + radius * sin(angle);
    
    // Lưu trữ điểm vào mảng waypoints
    waypoints[i].x = x;
    waypoints[i].y = y;
  }
}
