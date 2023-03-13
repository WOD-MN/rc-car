#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define the pins for the motor driver
#define motor1Pin1 2
#define motor1Pin2 3
#define motor2Pin1 4
#define motor2Pin2 5

// Define the threshold radius
#define thresholdRadius 10

// Define the GPS serial communication pins
#define GPSBaud 9600
#define GPSSerialRX 8
#define GPSSerialTX 9

// Create an instance of the TinyGPS++ library
TinyGPSPlus gps;

// Create a SoftwareSerial object for the GPS module
SoftwareSerial gpsSerial(GPSSerialRX, GPSSerialTX);

const double EARTH_RADIUS_KM = 6371.0;

// Define the GPS coordinates for the starting point and destination point
double startLat = 37.7749;
double startLon = -122.4194;
double destLat = 37.7748;
double destLon = -122.4193;

// Define an array to store the 20 waypoints
double waypoints[20][2];

// Define the index of the current waypoint
int currentWaypointIndex = 0;

void setup() {
  // Initialize the motor driver pins as output pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Initialize the GPS module serial communication
  gpsSerial.begin(GPSBaud);

  // Calculate the path between the starting point and destination point
  calculatePath(startLat, startLon, destLat, destLon, waypoints, 20);

  // Wait for the GPS module to get a valid GPS fix
  while (gps.location.isUpdated() == false) {
    // Do nothing
  }
}

void loop() {
  // Get the current GPS location
  gpsSerial.listen();
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      double currentLat = gps.location.lat();
      double currentLon = gps.location.lng();
      
      // Calculate the distance and direction to the current waypoint
      double distanceToWaypoint = distance(currentLat, currentLon, waypoints[currentWaypointIndex][0], waypoints[currentWaypointIndex][1]);
      double directionToWaypoint = direction(currentLat, currentLon, waypoints[currentWaypointIndex][0], waypoints[currentWaypointIndex][1]);
      
      // Move the RC car towards the current waypoint
      if (distanceToWaypoint > thresholdRadius) {
        moveRCcar(directionToWaypoint);
      } else {
        // If the RC car is within the threshold radius of the current waypoint, move to the next waypoint
        currentWaypointIndex++;
      }
      
      // If the RC car has reached the final waypoint, stop the motors
      if (currentWaypointIndex >= 20) {
        stopRCcar();
      }
    }
  }
}

void calculatePath(double startLat, double startLon, double destLat, double destLon, double waypoints[][2], int numWaypoints) {
  // Convert degrees to radians
  startLat *= M_PI / 180.0;
  startLon *= M_PI / 180.0;
  destLat *= M_PI / 180.0;
  destLon *= M_PI / 180.0;

  // Calculate the distance between the two points using the Haversine formula
  double deltaLat = destLat - startLat;
  double deltaLon = destLon - startLon;
  double a = std::pow(std::sin(deltaLat / 2.0), 2.0) + std::cos(startLat) * std::cos(destLat) * std::pow(std::sin(deltaLon / 2.0), 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  double distance = EARTH_RADIUS_KM * c;

  // Divide the path into equal segments
  double segmentDistance = distance / (numWaypoints - 1);
  double bearing = std::atan2(std::sin(deltaLon) * std::cos(destLat), std::cos(startLat) * std::sin(destLat) - std::sin(startLat) * std::cos(destLat) * std::cos(deltaLon));

  // Calculate the GPS coordinates of each waypoint
  for (int i = 0; i < numWaypoints; i++) {
    double dist = i * segmentDistance;
    double lat = std::asin(std::sin(startLat) * std::cos(dist / EARTH_RADIUS_KM) + std::cos(startLat) * std::sin(dist / EARTH_RADIUS_KM) * std::cos(bearing));
    double lon = startLon + std::atan2(std::sin(bearing) * std::sin(dist / EARTH_RADIUS_KM) * std::cos(startLat), std::cos(dist / EARTH_RADIUS_KM) - std::sin(startLat) * std::sin(lat));
    waypoints[i][0] = lat * 180.0 / M_PI;
    waypoints[i][1] = lon * 180.0 / M_PI;
  }
}

double direction(double lat1, double lon1, double lat2, double lon2) {
// Calculate the direction from the starting point to the destination point
// and return the result in radians
double y = sin(lon2-lon1) * cos(lat2);
double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
double direction = atan2(y, x);
return direction;
}

void moveRCcar(double direction) {
// Convert the direction from radians to degrees
double heading = direction * 180 / PI;

// Adjust the heading to be between 0 and 360 degrees
if (heading < 0) {
heading += 360;
}

// Calculate the difference between the heading and the RC car's current direction
double headingDifference = heading - gps.course.deg();

// Adjust the heading difference to be between -180 and 180 degrees
if (headingDifference > 180) {
headingDifference -= 360;
} else if (headingDifference < -180) {
headingDifference += 360;
}

// Calculate the turning speed based on the heading difference
int turningSpeed = map(abs(headingDifference), 0, 180, 0, 255);

// Calculate the forward speed based on the distance to the waypoint
int forwardSpeed = map(distanceToWaypoint, 0, 100, 0, 255);

// Adjust the motor speeds based on the heading difference
if (headingDifference > 0) {
// Turn right
analogWrite(motor1Pin1, forwardSpeed + turningSpeed);
analogWrite(motor1Pin2, 0);
analogWrite(motor2Pin1, forwardSpeed - turningSpeed);
analogWrite(motor2Pin2, 0);
} else {
// Turn left
analogWrite(motor1Pin1, forwardSpeed - turningSpeed);
analogWrite(motor1Pin2, 0);
analogWrite(motor2Pin1, forwardSpeed + turningSpeed);
analogWrite(motor2Pin2, 0);
}
}

void stopRCcar() {
// Stop the motors
digitalWrite(motor1Pin1, LOW);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, LOW);
digitalWrite(motor2Pin2, LOW);
}
