const int pulsePin = A0;    // PUL+ pin
const int dirPin = A1;      // DIR+ pin
const int J1rotdir = 1;

String inData;
String function;
char WayPt[101][50];
int WayPtDel;

void driveMotorsJ(String inData) {
  int J1start = inData.indexOf('A');
  int J2start = inData.indexOf('B');
 
  
  int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
  int J1step = inData.substring(J1start + 2, J2start).toInt();


  if (J1dir==1)
    digitalWrite(dirPin, HIGH);
  else if (J1dir==0)
    digitalWrite(dirPin, LOW);
  else
    digitalWrite(dirPin, HIGH);
  Serial.println(String("Dir")+J1dir+String("Step")+J1step);
  
  for (int i = 0; i < J1step; i++) {
    // Generate a pulse
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(500);  // Delay for 500 microseconds (50 steps per second)
    //delay(2);
    digitalWrite(pulsePin, LOW);
    //delay(2);
    delayMicroseconds(500);  // Delay for 500 microseconds
  }

  
}


void setup() {
  // put your setup code here, to run once:
  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);

}



void loop() {

  WayPtDel = 0;
  while (Serial.available() > 0 or WayPtDel == 1)
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);
      
      //----- MOVE J ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")
      {
        Serial.print("command recieved");
        driveMotorsJ(inData);
        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }

      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}
