

#include <Servo.h>
#include <math.h>

const int X_pin = A0; // analog pin connected to X input
const int Y_pin = A1; // analog pin connected to Y input
const int btn_pin = 10;
const int led_pin = 13;

const int servo_X_pin = 8;
const int servo_Y_pin = 6;
const int servo_Z_pin = 5;

const double draw_speed = 80;

const double stepsize = 0.005;
const int waittime = 10;
const int draw_delay = 100;

double x = 0;
double y = 0.5;

// shape parameters
const double center_x = 0;
const double center_y = 0.7;
const double shape_r = 0.15;

class Sketcher {
  private:
    Servo sa, sb;
    double x, y;
  public:
    void init(int pin1, int pin2) {
      sa.attach(pin1);
      sb.attach(pin2);
    }
    
    void moveRaw(double a, double b) {

      // make sure values are in range
      if (a < 0) { a = 0; }
      if (a > 180) { a = 180; }
      if (b < 0) { b = 0; }
      if (b > 180) { b = 180; }

      // make sure the arms don't cross
      //if (b < 175 - a) { b = 175 - a; }
      
      sa.write(a);
      sb.write(b);
    }

    // Takes a point in Cartesian space and moves the arm there.
    // Valid positions correspond to roughly the top quarter of
    // a unit circle: about -0.5 < x < 0.5 and 0 < y < 1.
    void moveTo(double x, double y) {
      double r = sqrt(x*x + y*y);
      if (r > 0.9) { r = 0.9; }
      if (r < 0.1) { r = 0.1; }
      double theta = (y == 0) ? 0 : atan(x/y);

      double t = asin(r);
      double a = (theta + t) * 180 / PI;
      double b = -(theta - t) * 180 / PI;
      moveRaw(a, b);

      this->x = x;
      this->y = y;
    }

    void lineTo(double tx, double ty) {
      double stepsize = draw_speed / waittime / 1000;

      double x = this->x;
      double y = this->y;
      double d = sqrt((tx - x)*(tx - x) + (ty - y)*(ty - y));
      if (d < stepsize) { return; }

      double t = d / stepsize;
      double dx = (tx - x) / t;
      double dy = (ty - y) / t;

      for (int i = 0; i < t; i++) {
        x += dx;
        y += dy;
        moveTo(x, y);
        delay(waittime);
      }
    }

    void drawShape(int sides) {
      double theta = 0;
      double dtheta = 2*PI / sides;

      for (int i = 0; i <= 4*sides; i++) {
        double x = center_x - shape_r * sin(theta);
        double y = center_y - shape_r * cos(theta);
        lineTo(x,y);
        theta += dtheta;
        delay(draw_delay);
      }
    }

    void drawCircle() {
      double theta = 0;
      double dtheta = 2*PI / 180;

      for (int i = 0; i <= 720; i++) {
        double x = center_x - shape_r * sin(theta);
        double y = center_y - shape_r * cos(theta);
        lineTo(x,y);
        theta += dtheta;
      }
    }

};

Sketcher s;

int last_shape = 5;

void setup() {
  s.init(servo_X_pin, servo_Y_pin);
  s.moveTo(center_x, center_y - shape_r);
  pinMode(btn_pin, INPUT_PULLUP);

//  Serial.begin(9600);
}

void loop() {
//  int dx = analogRead(X_pin);
//  int dy = analogRead(Y_pin);

  int btn = digitalRead(btn_pin);
  if (btn == HIGH) {
//    double px = (dx / 1024.0) - 0.5;
//    double py = (dy / 1024.0);
//    s.moveTo(px, py);
//
    digitalWrite(led_pin, LOW);
  } else {
//    int px = dx * 180.0 / 1023.0;
//    int py = dy * 180.0 / 1023.0;
//    s.moveRaw(px, py);

    digitalWrite(led_pin, HIGH);

    if (last_shape == 5) {
      last_shape = 2;
    } else {
      last_shape++;
    }

    if (last_shape == 2) {
      s.drawCircle();
    } else {
      s.drawShape(last_shape);
    }
  }
  

  delay(waittime);
}
