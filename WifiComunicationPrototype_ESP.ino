#include <EasyTransfer.h>

float convertToFloat(unsigned char a,unsigned char b,unsigned char c,unsigned char d){
union {
    float f;
    unsigned long ul;
 } u;
 
   u.ul = (a << 24) | (b << 16) | (c << 8) | d;
   return u.f;
}

void float2Bytes(float input , byte* arr){
 union float2bytes { float f; char b[sizeof(float)]; };
 
 float2bytes f2b;
 
 float x;
 f2b.f = x;
 for ( int i=0; i < sizeof(float); i++ )
   send_byte(f2b.b[i]);

 for ( int i=0; i < sizeof(float); i++ )
   f2b.b[i] = read_byte();

 x = f2b.f;
  
  for(int i; i < 4; i++){
  arr[i] = f2b.b[i];
  }
}




//create two objects
EasyTransfer ETin, ETout; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  orintationTable gyro1;
  orintationTable gyro2;
  orintationTable gyro3;
  
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int[3][2] poseTable
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

// wifi connection variables
const char* ssid = "Jupi MK dom";
const char* password = "pimpekpimpek";
boolean wifiConnected = false;

// UDP variables
unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged"; // a string to send back

void setup() {
// Initialise Serial connection
Serial.begin(115200);

// Initialise wifi connection
wifiConnected = connectWifi();

// only proceed if wifi connection successful
if(wifiConnected){
udpConnected = connectUDP();
if (udpConnected){
// initialise pins
pinMode(5,OUTPUT);
}
}
}

void loop() {
// check if the WiFi and UDP connections were successful
    if(wifiConnected){
    if(udpConnected){

// if there’s data available, read a packet
    int packetSize = UDP.parsePacket();
    if(packetSize)
    {
    Serial.println("");
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = UDP.remoteIP();
    for (int i =0; i < 4; i++)
    {
    Serial.print(remote[i], DEC);
    if (i < 3)
    {
    Serial.print(".");
    }
    }
    Serial.print(", port ");
    Serial.println(UDP.remotePort());

// read the packet into packetBufffer
    UDP.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
      
      float fullPose[3][2];
      
      int bP = 0;
      for(int i; i < 4; i++){
        for(int n; n < 3; n++){
         fullPose[i][n] = convertToFloat(packetBuffer[bP] , packetBuffer[bP + 1] , packetBuffer[bP + 2] , packetBuffer[bP + 3]);
          bP = bP + 4;
        }
      }
         
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ReplyBuffer);
    UDP.endPacket();
  }
  delay(10);

}

}

}

// connect to UDP – returns true if successful or false if not
  boolean connectUDP(){
    boolean state = false;

    Serial.println("");
    Serial.println("Connecting to UDP");

    if(UDP.begin(localPort) == 1){
      Serial.println("Connection successful");
      state = true;
    }
    else{
      Serial.println("Connection failed");
    }

    return state;
  }
    // connect to wifi – returns true if successful or false if not
 boolean connectWifi(){
  boolean state = true;
  int i = 0;
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi");

    // Wait for connection
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
   if (i > 10){
   state = false;
   break;
  }
   i++;
  }
  if (state){
   Serial.println("");
   Serial.print("Connected to ");
   Serial.println(ssid);
   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());
  }
    else {
  Serial.println("");
  Serial.println("Connection failed.");
  }
  return state;
}
