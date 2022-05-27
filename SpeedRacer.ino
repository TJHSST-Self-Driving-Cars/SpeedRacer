/* INFO
    number of data points in cache: lidar._cached_scan_node_hq_count
    float angle = (((float)_cached_scan_node_hq_buf[index].angle_z_q14) * 90.0 / 16384.0);
    float distance = _cached_scan_node_hq_buf[index].dist_mm_q2 /4.0f;
    each cache load contains a full 360 scan. If you slow down the rotations too much it will not fit and data will be lost (too many points per 360 deg for cache size allowable on ESP32)
 */

 
#include "rpLidar.h"
#include "rpLidarTypes.h"
#include <esp_task_wdt.h>

#include <math.h>
#include <Servo.h>
#include "Sequence.h"

Servo steering;
Servo throttle;

// Follow the Gap Constants
#define BUBBLE_RADIUS   50
#define THRESH_VALUE    2000 // 2 meters

// Disparity Extender Constants
#define rawLen 360 //length of array of stuff
#define lidarRef(x) (x + rawLen/8)
#define usedLen (rawLen - (rawLen/4))
#define SPEED  1570
#define CAR_WIDTH 0.5
#define SAFETY_PERCENTAGE 300
float radians_per_point;

rpLidar lidar(&Serial2,115200,13,12);
float lidarPoints [rawLen];
float disparities[usedLen];
float differences[usedLen];
float bubbleLidarPoints [360];
float cropLidarPoints [180];

float currentAngle;
float currentDistance;
float firstZeroIndex = -1;
int slowDriveCount = 0;
//int count = 0;
int previousIdx = 0;

SequenceArrayList allSequences;
Sequence currentSequence;

static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
    Serial.println(result,HEX);
  }
}

/*_______________ SETUP METHOD ________________ */
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

void disparity_extend(float ranges[rawLen], float disparities[usedLen]);
/*_______________ LOOP METHOD ________________ */
void loop()
{

  // update array
  updateArray();

  // generate bubble
  // generateBubble(true);

  // greedy actuate
  // greedyFindBestActuate();
  // findBestPoint();
  // actuate();

  // disparity extender
  
  disparity_extend(lidarPoints,disparities);
}

/* ______________________RESOURCE METHODS ______________________*/


static void updateArray() {
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

static void generateBubble(boolean preproc) {
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

static void greedyFindBestActuate() {
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

static void findBestPoint() {

  allSequences.reset();
 
  //search for the first zero in the array and once you find it, exit the for loop
  for (int x = 0; x < 360; x++){
    if (bubbleLidarPoints[x] == 0){
      firstZeroIndex = x;
      break;
    }
  }


  //find sequences in the array while implementing proper wrap around 
  for (int distancesArrayIndex = firstZeroIndex; distancesArrayIndex < 360 + firstZeroIndex; distancesArrayIndex++){
   
    //if the current index is zero, add the previous sequence to the ArrayList and then end the previous sequence
    if (bubbleLidarPoints[distancesArrayIndex % 360] == 0){
     
      allSequences.appendSequence(currentSequence);
      currentSequence.reset();
    }

    //else, iterate through and add one to the end and length of the sequence until you reach a zero (Remember to implement modulus statements for wrap around)
    else if (bubbleLidarPoints[distancesArrayIndex % 360] != 0){
     
      currentSequence.sequenceLength++;
      currentSequence.sequenceEnd = distancesArrayIndex % 360;

      //find out if the sequence just started and if it did, set the beginning to the current index
      if (bubbleLidarPoints[(distancesArrayIndex - 1) % 360] == 0)
        currentSequence.sequenceBeginning = distancesArrayIndex % 360;
    }
  }
}

/*
static void findBestPointInitial() {
  // find maximum length sequence of non zeros

  // largestSequence.reset(); //make sure that the previous largest sequence doesn't confound with this array's largest sequence
  
  //search for the first zero in the array and once you find it, exit the for loop
  for (int x = 0; x < 360; x++){
    if (bubbleLidarPoints[x] == 0){
      firstZeroIndex = x;
      break;
    }
  }


  //find sequences in the array while implementing proper wrap around and find the biggest sequence
  for (int distancesArrayIndex = firstZeroIndex; distancesArrayIndex < 360 + firstZeroIndex; distancesArrayIndex++){
   
    //if the current index is zero, end the previous sequence, check if its bigger than the biggest sequence and if it is, save it as the biggest
    if (bubbleLidarPoints[distancesArrayIndex % 360] == 0){
     
      if (currentSequence.sequenceLength > largestSequence.sequenceLength)
        largestSequence.set(currentSequence);

      currentSequence.reset();
    }

    //else, iterate through and add one to the end and length of the sequence until you reach a zero (Remember to implement modulus statements for wrap around)
    else if (bubbleLidarPoints[distancesArrayIndex % 360] != 0){
     
      currentSequence.sequenceLength++;
      currentSequence.sequenceEnd = distancesArrayIndex % 360;

      //find out if the sequence just started and if it did, set the beginning to the current index
      if (bubbleLidarPoints[(distancesArrayIndex - 1) % 360] == 0)
        currentSequence.sequenceBeginning = distancesArrayIndex % 360; //not very elegant but it works ig
    }
  }


  /* Hypothetical code if we want to go back to implementing midpoint index in the gap instead of the largest value for some reason

  int beginningIdx = largestSequence.sequenceBeginning;
  int endIdx = largestSequence.sequenceEnd;

  if (beginningIdx > endIdx) //case for if the endIdx ends up wrapping around the array
    endIdx += 360

  int midpointIdx = (beginningIdx + endIdx)/ 2;

  
  int steeringAngle;
  // find steering angle
  if(maxIdx < 30 && maxIdx > 330) {
    steeringAngle = 1500;
  }
  else {
    steeringAngle = (int) abs(maxIdx - 270.0)/180.0 * 1000 + 1000;
  }
  

  // find throttle
  int throt = 1570; 

  steering.writeMicroseconds(steeringAngle);
  // throttle.writeMicroseconds(throt);
  slowDrive(3, (float)throt, 1500.0);

  // set previous idx (for hysteresis)
  previousIdx = maxIdx;
}
*/


//Since the motors we are using can't go very slow in general, this method attempts to help with it by oscilating between stopSpeed (generally 1500) and the driveSpeed
//A higher speedFactor means that you call driveSpeed more and a lower speedFactor means that you call stopSpeed more
//Each of these variables should be positive and make sure that the esc is on slow braking mode
static void slowDrive(int speedFactor, float driveSpeed, float stopSpeed){

  if ((int)(slowDriveCount % speedFactor) == 0)
    throttle.writeMicroseconds(stopSpeed);
  else if((int)(slowDriveCount % speedFactor) != 0)
    throttle.writeMicroseconds(driveSpeed);
  
  slowDriveCount++;
  
}

static int findBest(Sequence largestSequence) {
  int maxIdx = largestSequence.sequenceBeginning;
  float maxValue = bubbleLidarPoints[maxIdx];
    for(int i = largestSequence.sequenceBeginning; i < largestSequence.sequenceEnd;) { 
      if(bubbleLidarPoints[i] > maxValue) {
        maxIdx = i;
        maxValue = bubbleLidarPoints[i];
      }

      
      if (largestSequence.sequenceBeginning < largestSequence.sequenceEnd)
        i++;
      if(largestSequence.sequenceBeginning > largestSequence.sequenceEnd)
        i--;
    }

  return maxIdx; 

}

static int generateScore(Sequence c){
  // This method uses heuristics to determine the "best gap" for racing 
  double score = 0;
  int avg = (c.sequenceBeginning + c.sequenceEnd)/2;

  // hysteresis
  if(avg > 0 && avg < 180 && previousIdx > 180 && previousIdx < 360) {
    score -= 100;
  }
  else if(avg > 180 && avg < 360 && previousIdx > 0 && previousIdx < 180) {
    score -= 100;
  }

  // forwards or backwards
  if(avg < 90 && avg > 270) {
    score += 200; // add score to going forwards
  }
  else {
    score -= 1000; // decrease score for going backwards
  }

  // Max Gap;
  score += c.sequenceLength * 2; // if the length is bigger that's good

  // Obstacle avoidance
  if(bubbleLidarPoints[avg] > THRESH_VALUE + 250) {
    score += 50;
  }
  else {
    score -= 50;
  }
  Serial.println(score);
  return score;  
}

static void actuate() {
  findBestPoint();
  int scores [allSequences.getSize()];
  for(int i = 0; i < allSequences.getSize(); i++) {
    scores[i] = generateScore(allSequences.getSequence(i));
  }

  int maxI = 0; 
  int maxScore = scores[0];
  for(int i = 1; i < allSequences.getSize(); i++) {
    if(scores[i] > maxScore) {
      maxI = i;
      maxScore = scores[maxI];
    }
  }

  Sequence bestSequence = allSequences.getSequence(maxI);
  int maxIdx = findBest(bestSequence);
  Serial.println(maxIdx);

  int steeringAngle = (int) abs(maxIdx - 270.0)/180.0 * 1000 + 1000;
  int throt = 1570;
  //int steeringAngle;
  // int throt;


  // bin throttle and angle
  /*
  if(maxIdx > 0 && maxIdx < 90) {
    steeringAngle = 2000;
    throt = 1570; 
    Serial.println("RIGHT FORWARDS");
  }
  else if(maxIdx > 90 && maxIdx < 180) {
    steeringAngle = 2000;
    throt = 1430;
    Serial.println("RIGHT BACKWARDS");
  }
  else if(maxIdx > 180 && maxIdx < 270) {
    steeringAngle = 1000;
    throt = 1430;
    Serial.println("LEFT BACKWARDS");
  }
  else if(maxIdx > 270 && maxIdx < 360) {
    steeringAngle = 1000;
    throt = 1570;
    Serial.println("LEFT FORWARDS");
  }
  else {
    Serial.println("error");
  }
  */
  steering.writeMicroseconds(steeringAngle);
  // throttle.writeMicroseconds(throt);
  slowDrive(5, (float)throt, 1500.0);

  // set previous idx (for hysteresis)
  previousIdx = maxIdx;
  
}

// Disparity Extender
void get_differences(float ranges[rawLen], float differences[usedLen]) {
  differences[0] = 0.0f;
  for (int i = 1; i < usedLen; i++) {
    differences[i] = abs(ranges[lidarRef(i)] - ranges[lidarRef(i - 1)]);
  }
}

int get_disparities(float differences[usedLen], float disparities[usedLen], float threshold) {
  int size = 0;
  for (int i = 0; i < usedLen; i++) {
    if (differences[i] > threshold) {
      disparities[size] = i;
      size++;
    }
  }
  return size;
}

int get_num_points_to_cover(float dist, float width) {
  int angle = 2 * asin(width / (2 * dist));
  int num_points = 1 + int((angle / radians_per_point));
  return num_points;
}

void cover_points(int num_points, int start_idx, boolean cover_right, float ranges[rawLen]) {

  float new_dist = ranges[lidarRef(start_idx)];
  if (cover_right) {
    for (int i = 0; i < num_points; i++) {
      int next_idx = start_idx + 1 + i;
      if (next_idx >= usedLen) {
        break;
      }
      if (ranges[lidarRef(next_idx)] > new_dist) {
        ranges[lidarRef(next_idx)] = new_dist;
      }
    }
  } else {
    for (int i = 0; i < num_points; i++) {
      int next_idx = start_idx - 1 - i;
      if (next_idx < 0) {
        break;
      }
      if (ranges[lidarRef(next_idx)] > new_dist) {
        ranges[lidarRef(next_idx)] = new_dist;
      }
    }
  }
}

void extend_disparities(float disparities[usedLen], float ranges[rawLen], float car_width, float extra_pct, int dispSiz) {
  /*
      For each pair of points we have decided have a large difference
      between them, we choose which side to cover (the opposite to
      the closer point), call the cover function, and return the
      resultant covered array.
      Possible Improvements: reduce to fewer lines
  */
  float width_to_cover = (car_width / 2) * (1 + extra_pct / 100);
  for (int i = 0; i < dispSiz; i++) {
    int index = disparities[i];
    int first_idx = index - 1;
    //points = ranges[lidarRef(first_idx:first_idx + 2]
    float minV = ranges[lidarRef(first_idx)];
    float maxV = ranges[lidarRef(first_idx)];
    maxV = max(ranges[lidarRef(first_idx + 1)], maxV);
    maxV = max(ranges[lidarRef(first_idx + 2)], maxV);
    minV = min(ranges[lidarRef(first_idx + 1)], minV);
    minV = min(ranges[lidarRef(first_idx + 2)], minV);

    int close_idx = first_idx + minV;
    int far_idx = first_idx + maxV;
    float close_dist = ranges[close_idx];

    float num_points_to_cover = get_num_points_to_cover(close_dist,
      width_to_cover)
    boolean cover_right = close_idx < far_idx;
    cover_points(num_points_to_cover, close_idx,
      cover_right, ranges)
  }
}

float clamp(float a, float bot, float top) {
  return min(top, max(a, bot));
}

float get_steering_angle(int range_index, float range_len) {
  float lidar_angle = (range_index - (range_len / 2)) * radians_per_point;
  float steering_angle = clamp(lidar_angle, -3.1415 / 2, 3.1415 / 2);
  return steering_angle;
}

void disparity_extend(float ranges[rawLen], float disparities[usedLen],float differences[usedLen]) {
  float radians_per_point = (2 * 3.1415) / rawLen;

  get_differences(ranges,differences);

  int numDisparities = get_disparities(differences, disparities, DIFFERENCE_THRESHOLD);

  extend_disparities(disparities, proc_ranges,
    CAR_WIDTH, SAFETY_PERCENTAGE, numDisparities);
  int max = 0;
  for (int i = 0; i < usedLen; i++) {
    if (ranges[lidarRef(i)] > ranges[lidarRef(max)])
      max = i;
  }

  steering_angle = get_steering_angle(max, usedLen);
  speed = SPEED;

  // on top of reference implementation, for controlling the car
  Servo.writeMicroseconds(speed);
  // calculate steeringvalue
  int steeringValue = 1500 + steering_angle * (500 / 1.57);
  Servo.writeMicroseconds(steeringValue);

}
