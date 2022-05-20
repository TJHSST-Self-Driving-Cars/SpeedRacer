#include "rpLidar.h"
#include "rpLidarTypes.h"
#include <esp_task_wdt.h>

#include <math.h>
#include <Servo.h>
#include "Sequence.h"

Servo steering;
Servo throttle;

int BUBBLE_RADIUS = 50;
int THRESH_VALUE = 2000; // 2 meters
int count = 0;

rpLidar lidar(&Serial2,115200,13,12);
float lidarPoints [360];
float bubbleLidarPoints [360];
float cropLidarPoints [180];

float currentAngle;
float currentDistance;
float firstZeroIndex = -1;

Sequence largestSequence;
Sequence currentSequence;

static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
    Serial.println(result,HEX);
  }
}
void setup() {
   pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  Serial.begin(115200);
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  lidar.stopDevice(); //reset the device to be sure that the status is good
  delay(1);
  if(!lidar.start(express)){
    Serial.println("failed to start");
    return;
  } //start the express scan of the lidar\  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset

  xTaskCreatePinnedToCore(readPoints, "LidarPolling", 65536, NULL, 2, NULL, 0);

  steering.attach(27);
  throttle.attach(32);

}

void loop()
{
// number of data points in cache: lidar._cached_scan_node_hq_count
// float angle = (((float)_cached_scan_node_hq_buf[index].angle_z_q14) * 90.0 / 16384.0);
// float distance = _cached_scan_node_hq_buf[index].dist_mm_q2 /4.0f;
// each cache load contains a full 360 scan. If you slow down the rotations too much it will not fit and data will be lost (too many points per 360 deg for cache size allowable on ESP32)
//  lidar.DebugPrintMeasurePoints(lidar._cached_scan_node_hq_count);
//  delay(1);
//  moveMotors();
// update arrray
updateArray();

// generate bubble
generateBubble(true);

// greedy actuate
// greedyFindBestActuate();
findBestPoint();

}

static void moveMotors() {
  throttle.writeMicroseconds(1600);
  if (count % 2) 
    steering.writeMicroseconds(1000);
  else
    steering.writeMicroseconds(2000);

  count++;
  delay(500);
  Serial.println("I MOVED THE MOTORS");
}

void updateArray() {
  for(int i = 0; i < lidar._cached_scan_node_hq_count; i++) {
    currentAngle = (((float)lidar._cached_scan_node_hq_buf[i].angle_z_q14) * 90.0 / 16384.0); // TODO fix error message
    currentDistance = lidar._cached_scan_node_hq_buf[i].dist_mm_q2 /4.0f;

    int arrayIndex = (int)currentAngle;
    if(arrayIndex >= 0 && arrayIndex < 360) {
      lidarPoints[arrayIndex] = currentDistance;
    }
    else {
      Serial.println("ERROR \t Array index of lidar point not within [0,360)"); 
    }
  }
  
  
}

void generateBubble(boolean preproc) {
  int min_idx = 0;
  int min_val = 999999;
  for(int i = 0; i < 360; i++) {
    bubbleLidarPoints[i] = lidarPoints[i];
    if(bubbleLidarPoints[i] > 0 && bubbleLidarPoints[i] < min_val) {
      min_val = bubbleLidarPoints[i];
      min_idx = i;
    }
  }

  // create safety bubble of 0's 
  for(int i = min_idx - BUBBLE_RADIUS; i < min_idx + BUBBLE_RADIUS; i++) {
    bubbleLidarPoints[i % 360] = 0;
  }

  if(preproc == true) {
    for(int j=  0; j < 360; j++) {
      if(bubbleLidarPoints[j] < THRESH_VALUE) {
        bubbleLidarPoints[j] = 0;
      }
    }
  }
}

void greedyFindBestActuate() {
  // use only front 120 degrees
  int croppedPoints [120];
  for(int a = 0; a < 120; a++) {
    int t = (300 + a) % 120;
    croppedPoints[a] = bubbleLidarPoints[t];
  }
  
  int maximumIdx = 0;
  int maximum = croppedPoints[0];
  for(int i = 1; i < 120; i++) {
    if(croppedPoints[i] > maximum) {
      maximum = croppedPoints[i];
      maximumIdx = i;
    }
  }
  float idx = maximumIdx - 60;
  idx = idx * 8.3; // bounds will be 
  idx = -idx + 1100;

  // if(idx > 1520) idx = 2000;
  // if(idx < 1480) idx = 1000;

  Serial.println(idx);
  
  // actuate
  steering.writeMicroseconds(idx);
  throttle.writeMicroseconds(1575);
}

void findBestPoint() {
  // find maximum length sequence of non zeros

  largestSequence.reset();
  
  //search for the first zero in the array and once you find it, exit the for loop
  for (int x = 0; x < 360; x++){
    if (bubbleLidarPoints[x] == 0){
      firstZeroIndex = x;
      break;
    }
  }


  //find sequences in the array and implement proper wrap around and find the biggest sequence
  for (int distancesArrayIndex = firstZeroIndex; distancesArrayIndex < 360 + firstZeroIndex; distancesArrayIndex++){
   
    //if the current index is zero, end the previous sequence, check if its bigger than the biggest sequence and if it is, save it as the biggest
    if (bubbleLidarPoints[distancesArrayIndex % 360] == 0){
     
      if (currentSequence.sequenceLength > largestSequence.sequenceLength)
        largestSequence.set(currentSequence);

      currentSequence.reset();
    }

    //else, iterate through and add one to the end and length of the sequence until you reach a zero (Remember to implement modulus statements)
    else if (bubbleLidarPoints[distancesArrayIndex % 360] != 0){
     
      currentSequence.sequenceLength++;
      currentSequence.sequenceEnd = distancesArrayIndex % 360;

      //find out if the sequence just started and if it did, set the beginning to the current index
      if (bubbleLidarPoints[(distancesArrayIndex - 1) % 360] == 0)
        currentSequence.sequenceBeginning = distancesArrayIndex % 360; //not very elegant but it works ig
    }
  }

  int maxIdx = largestSequence.sequenceBeginning;
  float maxValue = bubbleLidarPoints[maxIdx];
    for(int i = largestSequence.sequenceBeginning; i < largestSequence.sequenceEnd;) { 
      if(bubbleLidarPoints[i] > maxValue) {
        maxIdx = i;
        maxValue = bubbleLidarPoints[i];
      }

      
      if (largestSequence.sequenceBeginning < largestSequence.sequenceEnd) {
        i++;
      }
      if(largestSequence.sequenceBeginning > largestSequence.sequenceEnd) {
        i--;
      } 
    }

       
  

  // find steering angle
  int steeringAngle = (int) abs(maxIdx - 270.0)/180.0 * 1000 + 1000;

  // find throttle
  int throt = 1570; // To-Do: Change the throttle based on TTC

  steering.writeMicroseconds(steeringAngle);
  throttle.writeMicroseconds(throt);
}
