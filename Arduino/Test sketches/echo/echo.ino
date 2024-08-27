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
  Serial.println(String("Dir")+J1dir+String("Step")+J1step);


}


void setup() {
  // put your setup code here, to run once:
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
        //Serial.print("command recieved");
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
