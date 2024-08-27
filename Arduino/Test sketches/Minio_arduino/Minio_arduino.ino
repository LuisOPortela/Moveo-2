const int J1rotdir = 1;

String inData;
String function;
char WayPt[101][50];
int WayPtDel;

const int J1stepPin = 2;
const int J1dirPin = 3;


const int J1calPin = 14;

void driveMotorsJ(String inData) {
  int J1start = inData.indexOf('A');
  int J2start = inData.indexOf('B');
  int SPstart = inData.indexOf('S');
  int Adstart = inData.indexOf('G');
  
  int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
  int J1step = inData.substring(J1start + 2, J2start).toInt();
  float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();


  int J1cur = 0;

  if (J1dir == 1 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, LOW);
  }
  else if (J1dir == 1 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, LOW);
  }
  
  
  while (J1cur < J1step){
            
    J1cur = ++J1cur;
    J1_PEcur = 0;
    digitalWrite(J1stepPin, LOW);
    delayMicroseconds(curDelay);
    digitalWrite(J1stepPin, HIGH);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);

  
  pinMode(J1calPin, INPUT_PULLUP);

  pinMode(Input22, INPUT_PULLUP);
  pinMode(Input23, INPUT_PULLUP);
  pinMode(Input24, INPUT_PULLUP);
  pinMode(Input25, INPUT_PULLUP);


  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
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

}
