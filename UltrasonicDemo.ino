// Replace with pins connected from microproccessor to HC-SR04
const int trigPin = REPLACEME;
const int echoPin = REPLACEME2;

// Defines floats duration and distance
float duration, distance;

// Runs once
void setup() {
  // Sets serial monitor to 9600 baud
  Serial.begin(9600);
  // Sets the trigger pin as an output
  pinMode(trigPin, OUTPUT);
  // Sets the echo pin as an input
  pinMode(echoPin, INPUT);
}
// Runs continously
void loop() {
  // Sets the trigger pin to LOW
  digitalWrite(trigPin, LOW);
  //Delay
  delayMicroseconds(5);
  // Sets the trigger pin to HIGH
  digitalWrite(trigPin, HIGH);
  //Delay
  delayMicroseconds(10);
  // Sets the trigger pin back to LOW
  digitalWrite(trigPin, LOW);
  // Stores the data held in echoPin to float duration
  duration = pulseIn(echoPin, HIGH);
  // Calculates distance using duration*the speed of sound in air/2
  distance = ((duration*0.0343)/2);
  // Prints the word Distance
  Serial.print("Distance: ");
  // Prints the distance
  Serial.println(distance);
  // Delay until next loop
  delay(100);
}
