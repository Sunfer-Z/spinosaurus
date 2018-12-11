#include "imu.h"
#include "MPU9250.h"
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// threshold values for changing form -- 0 deg
float thresholds_ba[NUM_IMUS] = {-1.4, -1.2, -5.0}, thresholds_la[NUM_IMUS] = {-0.7, -0.6, -2.0};
float thresholds_uh[NUM_IMUS] = {0.6, 1.1, 0.95}, thresholds_bh[NUM_IMUS] = {5.0, 1.5, 1.5};

// threshold values for changing form -- 45 deg
float thresholds_ba45[NUM_IMUS] = {-5.0, -5.0, -5.0}, thresholds_la45[NUM_IMUS] = {-1.2, -1.3, -5.0};
float thresholds_uh45[NUM_IMUS] = {1.5, 1.5, 0.65}, thresholds_bh45[NUM_IMUS] = {5.0, 3.0, 1.0};

// threshold values for changing form -- 90 deg
float thresholds_ba90[NUM_IMUS] = {-5.0, -5.0, -5.0}, thresholds_la90[NUM_IMUS] = {-1.1, -1.2, -5.0};
float thresholds_uh90[NUM_IMUS] = {1.5, 1.2, 0.55}, thresholds_bh90[NUM_IMUS] = {5.0, 4.0, 2.0}; 

// array of FSMs that compose system
FSM sys[NUM_IMUS];
float diff_vals[NUM_IMUS] = {0.0, 0.0, 0.0};
int global_form = STRAIGHT;

// initial values for SpineAligner flags
bool mode = OFF;
bool cal = OFF;
bool load = OFF;
bool start_over = OFF;

int8_t count = 0, last_send = 0, reset = 0;

// array to hold info for all IMUs
IMU_T IMU[NUM_IMUS];
int pins[NUM_IMUS] = {37, 36, 33};

// IMU interface
MPU9250 imu_device(Wire, 0x68);

// Bluetooth interface
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(REQ_PIN, RDY_PIN, RST_PIN);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void calibrate_all_angles(int angle);     // calibrates at 0, 45, 90 degrees
void load_offsets(int &shouty_mark);      // loads offsets from phone app
void initSystem();                        // initialization
void imu_select(int imu_id);              // selects 1 IMU at a time
int sign(float z_val);                    // sign of z-axis acceleration 
void update_indices();                    // updates IMU acceleration indices for moving average
bool updateState(int fsm, int i);         // updates IMU states
void print_form_0(bool bad[3]);           // prints the spine form at 0 degree
void print_form_45(bool bad[3]);          // prints the spine form at 45 degrees
void print_form_90(bool bad[3]);          // prints the spine form at 90 degrees
void print_global_form();                 // prints the overall spine form and control vibration motors
void print_indiv_accel(int imu_id);       // debug: prints individual y-axis acceleration 
void print_accel_diff();                  // debug: prints differences of y-axis acceleration between any two IMUs 
void write_string(String s);              // Bluetooths transmission
void Bluetooth();                         // polls Bluetooth and receives commands from the phone app

void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  // setup motor pins
  pinMode(UPPER_MOTOR, OUTPUT);
  pinMode(LOWER_MOTOR, OUTPUT);

  digitalWrite(UPPER_MOTOR, HIGH);
  digitalWrite(LOWER_MOTOR, HIGH);

  initSystem();
  BTLEserial.begin();
  
  // setup and initialize each node
  for(int i = 0; i < NUM_IMUS; ++i) {
    IMU[i].init_pin(pins[i]);
  }

  int status = imu_device.begin();
  if(status < 0) {
      Serial.print("status = ");
      Serial.println(status);
  }
  
  for(int i = 0; i < NUM_IMUS; ++i) {
    imu_select(i);
    IMU[i].set_accel_scale(imu_device, 0);
  }

  while(cal == OFF && load == OFF) {
    Bluetooth();
  }

  int shouty_mark = 0;
  if(cal == ON) {
    write_string("y");
    for(int i = 0; i < 3; ++i) {
      calibrate_all_angles(i);
    }
  }
  else if(load == ON) {
    while(shouty_mark < 3) {
      BTLEserial.pollACI();
      if(BTLEserial.available()) {
        load_offsets(shouty_mark);
      }
    }
    load = OFF;
  }
}

void(* resetFunc) (void) = 0;

void loop() {
  if(mode == ON) {
    float z_val = 0;
    for(int i = 0; i < NUM_IMUS; ++i) {
      imu_select(i);
      IMU[i].update_accel_averages(imu_device);
      imu_select(-1);
      if(i == 1) {
        z_val = IMU[i].accel_xyz[2].avg;
      }
    }
  
    int index = 0;
    bool bad[3];
    if(z_val > DEG45) {
      index = 0;
    }
    else if (z_val <= DEG45 && z_val > DEG90){
      index = 1;
    }
    else { // z_val <= DEG90
      index = 2;
    }
    
    for(int i = 0; i < NUM_IMUS; ++i) {
      bad[i] = updateState(i, index);
    }
    
    if(z_val > DEG45) {
      print_form_0(bad);  
    }
    else if(z_val <= DEG45 && z_val > DEG90){
      print_form_45(bad);
    }
    else { // z_val <= DEG90
      print_form_90(bad);
    }
    
    update_indices();

    if(last_send == 10) {
      String s;
      if(global_form < 0) {
        s = String(global_form+6) + ";" + String(index) + "!";
      }
      else {
        s = String(global_form) + ";" + String(index) + "!";
      }
      write_string(s);
      last_send = 0;
    }
    else {
      ++last_send;
    }
    
  }
  else {
    digitalWrite(UPPER_MOTOR, HIGH);
    digitalWrite(LOWER_MOTOR, HIGH);
  }

  if(start_over == ON) {
    start_over = OFF;
    setup_again();
  }

  Bluetooth();
  if(cal == ON) {
    for(int i = 0; i < 3; ++i) {
      calibrate_all_angles(i);
    }
  }
}

/* initialize each FSM in the system */
void initSystem() {
  for(int i = 0; i < NUM_IMUS; ++i) {
    for(int j = i + 1; j < NUM_IMUS; ++j) {
      sys[i+j-1].imu_bottom = i;
      sys[i+j-1].imu_top = j;
      sys[i+j-1].state = VERY_GOOD;
      sys[i+j-1].form = STRAIGHT;
      sys[i+j-1].BA[0] = thresholds_ba[i+j-1];
      sys[i+j-1].LA[0] = thresholds_la[i+j-1];
      sys[i+j-1].UH[0] = thresholds_uh[i+j-1];
      sys[i+j-1].BH[0] = thresholds_bh[i+j-1];
      sys[i+j-1].BA[1] = thresholds_ba45[i+j-1];
      sys[i+j-1].LA[1] = thresholds_la45[i+j-1];
      sys[i+j-1].UH[1] = thresholds_uh45[i+j-1];
      sys[i+j-1].BH[1] = thresholds_bh45[i+j-1];
      sys[i+j-1].BA[2] = thresholds_ba90[i+j-1];
      sys[i+j-1].LA[2] = thresholds_la90[i+j-1];
      sys[i+j-1].UH[2] = thresholds_uh90[i+j-1];
      sys[i+j-1].BH[2] = thresholds_bh90[i+j-1];
      sys[i+j-1].THRE_GOOD = T_VAL;
      sys[i+j-1].THRE_BAD = T_VAL;
      sys[i+j-1].count_g = 0;
      sys[i+j-1].count_b = 0;
    }
  }
}

/* after receiving 'c', get offset values at 0, 45, 90 degrees */
void calibrate_all_angles(int angle) {
  float z_val = 0;
  cal = OFF;
  Serial.println("Stand at " + String(angle) + " degrees and press 'c' when ready");
  while(cal == OFF) {
    Bluetooth();
  }

  Serial.println("Stand still at " + String(angle) + " degrees!");
  for(int i = 0; i < NUM_IMUS; ++i) {
    imu_select(i);
    IMU[i].update_accel_averages(imu_device);
    imu_select(-1);
    if(i == 1) {
      z_val = IMU[i].accel_xyz[2].avg;
    }
  }
  update_indices();

  switch(angle) {
    case 0:
      while(z_val <= DEG45) {
        for(int i = 0; i < NUM_IMUS; ++i) {
          imu_select(i);
          IMU[i].update_accel_averages(imu_device);
          imu_select(-1);
          if(i == 1) {
            z_val = IMU[i].accel_xyz[2].avg;
          }
        }
        update_indices();
      }
      break;
    case 1:
      while(z_val > DEG45 || z_val <= DEG90) {
        for(int i = 0; i < NUM_IMUS; ++i) {
          imu_select(i);
          IMU[i].update_accel_averages(imu_device);
          imu_select(-1);
          if(i == 1) {
            z_val = IMU[i].accel_xyz[2].avg;
          }
        }
        update_indices();
        Serial.println(String(z_val));
      }
      break;
    case 2:
      while(z_val > DEG90) {
        for(int i = 0; i < NUM_IMUS; ++i) {
          imu_select(i);
          IMU[i].update_accel_averages(imu_device);
          imu_select(-1);
          if(i == 1) {
            z_val = IMU[i].accel_xyz[2].avg;
          }
        }
        update_indices();
      }
      break;
    default:
      Serial.println("ERROR, NO ANGLE FOR " + String(angle));
      break;
  }

  Serial.print("Calibrating...");
  delay(1000);
  for(int i = 0; i < NUM_IMUS; ++i) {
    imu_select(i);
    IMU[i].calibrate_IMU(imu_device, angle);
  }
  Serial.println("done calibrating!");
  cal = OFF;

  String s = String(IMU[0].accel_xyz[1].offset[angle]) + ";" + String(IMU[1].accel_xyz[1].offset[angle]) + ";" + String(IMU[2].accel_xyz[1].offset[angle]) + "!";

  uint8_t sendbuffer[20];
  s.getBytes(sendbuffer, 20);
  char sendbuffersize = min(20, s.length());
  BTLEserial.write(sendbuffer, sendbuffersize);
}

/* after receiving 'l', receive and store offset values from phone app */
void load_offsets(int &shouty_mark) {
  int i = 0;
  Serial.println("please send offset values");
  String node_offsets[3] = {"","",""};
  while(BTLEserial.available()) {
    char c = BTLEserial.read();
    Serial.println(c);
    if(c != ';' && c != '!') {
      Serial.println("node_offset val updated");
      node_offsets[i] += c;
    }
    if(c == ';') {
      Serial.println("incremented index");
      ++i;
    }
    if(c == '!') {
     ++shouty_mark;
     Serial.println("updated shouty mark: " + String(shouty_mark));
    }
  }

  for(int j = 0; j < 3; ++j) {
    Serial.print("Offset at IMU " + String(j) + " at angle " + String(shouty_mark-1) + " = ");
    Serial.println(String(node_offsets[j].toFloat()));
    IMU[j].accel_xyz[1].offset[shouty_mark-1] = node_offsets[j].toFloat();
  }
}

/* Selects an IMU by setting that particular IMU’s AD0 pin to high,
* while setting all others to low, thus giving it a unique address.
* imu_id ranges from 0-4 (or number of IMU’s we end up using).
* Should be called by other functions.
*/
void imu_select(int imu_id) {
  if(imu_id < 0) {
    for(int i = 0; i < NUM_IMUS; ++i) {
      digitalWrite(pins[i], HIGH);
    }
    return;
  }
  for(int i = 0; i < NUM_IMUS; ++i) {
    if(i == imu_id) {
      digitalWrite(pins[i], LOW);
    }
    else {
      digitalWrite(pins[i], HIGH);
    }
  }
}

/* return sign of given input */
int sign(float z_val) {
  if(z_val < 0) {
    return -1;
  }
  else {
    return 1;
  }
}

void update_indices() {
  for(int i = 0; i < NUM_IMUS; ++i) {
    IMU[i].accel_index += 1;
    if(IMU[i].accel_index >= RUNNING_AVG_SIZE) {
      IMU[i].accel_index = 0;
    }
  }
}

/* update state and form for given FSM, return 1 if state is bad */
bool updateState(int fsm, int i /*angle*/) {
  bool bad = 0;
  int imu1 = sys[fsm].imu_top, imu0 = sys[fsm].imu_bottom;
  float diff = sign(IMU[imu1].accel_xyz[2].avg)*(IMU[imu1].accel_xyz[1].avg - IMU[imu1].accel_xyz[1].offset[i]) - 
               sign(IMU[imu0].accel_xyz[2].avg)*(IMU[imu0].accel_xyz[1].avg - IMU[imu0].accel_xyz[1].offset[i]);
  // for Bluetooth
  diff_vals[i] = diff;

  float tmp_LA = sys[fsm].LA[i]; float tmp_UH = sys[fsm].UH[i]; float tmp_BH = sys[fsm].BH[i];
  // 20 degrees
  float z_val = IMU[1].accel_xyz[2].avg;
  if (i == 0 && z_val <= DEG20) {
    tmp_LA += TMP_LA20;
    tmp_UH += TMP_UH20;
    tmp_BH += TMP_BH20;
  };

  if(diff <= sys[fsm].BA[i]) {
    sys[fsm].form = BOTH_ARCH;
  }
  else if(diff <= tmp_LA && diff > sys[fsm].BA[i]) {
    sys[fsm].form = LOWER_ARCH;
  }
  else if(diff <= tmp_UH && diff > tmp_LA) {
    sys[fsm].form = STRAIGHT;
  }
  else if(diff <= tmp_BH && diff > tmp_UH) {
    sys[fsm].form = UPPER_HUNCH;
  }
  else { // diff > sys[fsm].BH[i]
    sys[fsm].form = BOTH_HUNCH;
  }

  switch(sys[fsm].state) {
    case VERY_GOOD:
      if(sys[fsm].form != STRAIGHT) {
        sys[fsm].state = GOOD;
      }
      break;
    case GOOD:
      if(sys[fsm].form != STRAIGHT && sys[fsm].count_b > sys[fsm].THRE_BAD) {
        sys[fsm].state = BAD;
        sys[fsm].count_b = 0;
      }
      else if(sys[fsm].form != STRAIGHT && sys[fsm].count_b <= sys[fsm].THRE_BAD) {
        ++sys[fsm].count_b;
      }
      else { // sys[fsm].form == STRAIGHT
        sys[fsm].state = VERY_GOOD;
        sys[fsm].count_b = 0;
      }
      break;
    case BAD:
      bad = 1;
      if(sys[fsm].form == STRAIGHT && sys[fsm].count_g > sys[fsm].THRE_GOOD) {
        sys[fsm].state = GOOD;
        sys[fsm].count_g = 0;
      }
      else if(sys[fsm].form == STRAIGHT && sys[fsm].count_g <= sys[fsm].THRE_GOOD) {
        ++sys[fsm].count_g;
      }
      else { // sys[fsm].form != STRAIGHT
        sys[fsm].state = VERY_BAD;
        sys[fsm].count_g = 0;
      }
      break;
    case VERY_BAD:
      bad = 1;
      if(sys[fsm].form == STRAIGHT) {
        sys[fsm].state = BAD;
      }
      break;
    default:
      Serial.print("INVALID STATE: "); Serial.println(sys[fsm].state);
      break;
  }
  return bad;
}

void print_form_0(bool bad[3]) {
  if(!bad[0] && !bad[1] && !bad[2]) {
    global_form = STRAIGHT;
    Serial.print("YOUR FORM IS GOOD :-D\t");
    // output 0 to all motors
  }
  else {
    if(sys[1].form == BOTH_HUNCH && sys[2].form == BOTH_HUNCH) {
      if(bad[1] || bad[2]) {
        global_form = BOTH_HUNCH;
      }
      Serial.print("CURRENT FORM = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to all motors
    }
    else if(sys[1].form == UPPER_HUNCH || sys[2].form == UPPER_HUNCH) { // test
      if((bad[1] && sys[1].form == UPPER_HUNCH) || (bad[2] && sys[2].form == UPPER_HUNCH)) {
        global_form = UPPER_HUNCH;
      }
      Serial.print("CURRENT FORM = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to top motors
    }
    else if(sys[0].form == LOWER_ARCH || sys[1].form == LOWER_ARCH) {
      if((bad[0] && sys[0].form == LOWER_ARCH) || (bad[1] && sys[1].form == LOWER_ARCH)) {
        global_form = LOWER_ARCH;
      }
      Serial.print("CURRENT FORM = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to bottom motors
    }
    else if(sys[0].form == BOTH_ARCH && sys[1].form == BOTH_ARCH) {
      if(bad[0] || bad[1]) {
        global_form = BOTH_ARCH;
      }
      Serial.print("CURRENT FORM = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to all motors
    }
    else if(sys[0].form == STRAIGHT && sys[1].form == STRAIGHT && sys[2].form == STRAIGHT) {
      Serial.print("CURRENT FORM = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
    else {
      Serial.print("ERROR = " + String(sys[0].form) + ":" + String(sys[0].state) + ", " + String(sys[1].form) + ":" + String(sys[1].state)
                      + ", " + String(sys[2].form) + ":" + String(sys[2].state) + "\t");
    }
  }
  print_global_form();
}

void print_form_45(bool bad[3]) {
  if(!bad[0] && !bad[1] && !bad[2]) {
    global_form = STRAIGHT;
    Serial.print("YOUR FORM45 IS GOOD :-D\t");
    // output 0 to all motors
  }
  else {
    if(sys[1].form == BOTH_HUNCH && sys[2].form == BOTH_HUNCH) {  //test
      if((bad[1] && sys[1].form == BOTH_HUNCH) || (bad[2] && sys[2].form == BOTH_HUNCH)) {
        global_form = BOTH_HUNCH;
      }
      Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to all motors
    }
    else if(sys[1].form >= UPPER_HUNCH) { // test
      if(sys[0].form >= UPPER_HUNCH && sys[2].form >= UPPER_HUNCH) {  // test
        if(bad[0] && bad[2]) {
          global_form = BOTH_HUNCH;
        }
        Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else if(sys[0].form >= UPPER_HUNCH) {
        if(bad[0]) {
          global_form = LOWER_HUNCH;
        }
        Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else if(sys[2].form >= UPPER_HUNCH) { // test
        if(bad[2]) {
          global_form = UPPER_HUNCH;
        }
        Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else {
        Serial.print("ERROR45: " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
    }
    else if(sys[0].form <= LOWER_ARCH || sys[1].form <= LOWER_ARCH) {
      if((bad[0] && sys[0].form <= LOWER_ARCH) || (bad[1] && sys[1].form <= LOWER_ARCH)) {
        global_form = LOWER_ARCH;
      }
      Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to bottom motors
    }    
    else if(sys[0].form == STRAIGHT && sys[1].form == STRAIGHT && sys[2].form == STRAIGHT) {
      Serial.print("CURRENT FORM45 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
    else {
      Serial.print("ERROR45: " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
  }
  print_global_form();
}

void print_form_90(bool bad[3]) {
  if(!bad[0] && !bad[1] && !bad[2]) {
    global_form = STRAIGHT;
    Serial.print("YOUR FORM90 IS GOOD :-D\t");
  }
  else {
    if(sys[1].form == BOTH_HUNCH && sys[2].form == BOTH_HUNCH) {  // test
      if((bad[1] && sys[1].form == BOTH_HUNCH) || (bad[2] && sys[2].form == BOTH_HUNCH)) {
        global_form = BOTH_HUNCH;
      }
      Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to all motors
    }
    else if(sys[1].form >= UPPER_HUNCH) { // test
      if(sys[0].form >= UPPER_HUNCH && sys[2].form == UPPER_HUNCH) {
        if(bad[0] && bad[2]) {
          global_form = BOTH_HUNCH;
        }
        Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else if(sys[0].form >= UPPER_HUNCH) {
        if(bad[0]) {
          global_form = LOWER_HUNCH;
        }
        Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else if(sys[2].form >= UPPER_HUNCH) { // test
        if(bad[2]) {
          global_form = UPPER_HUNCH;
        }
        Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
      else {
        Serial.print("ERROR90: " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      }
    }
    else if(sys[0].form <= LOWER_ARCH || sys[1].form <= LOWER_ARCH) {
      if((bad[0] && sys[0].form <= LOWER_ARCH) || (bad[1] && sys[1].form <= LOWER_ARCH)) {
        global_form = LOWER_ARCH;
      }
      Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
      // output 1 to bottom motors
    }
    else if(sys[2].form <= LOWER_ARCH) {
      if(bad[2]) {
        global_form = UPPER_HUNCH;
      }
      Serial.print("CURRENT FORM>90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
    else if(sys[0].form == STRAIGHT && sys[1].form == STRAIGHT && sys[2].form == STRAIGHT) {
      Serial.print("CURRENT FORM90 = " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
    else {
      Serial.print("ERROR90: " + String(sys[0].form) + ", " + String(sys[1].form) + ", " + String(sys[2].form) + "\t");
    }
  }
  print_global_form();
}

void print_global_form() {
  if(global_form == -2) {
    Serial.println("GLOBAL FORM = BOTH ARCH");
    digitalWrite(UPPER_MOTOR, LOW);
    digitalWrite(LOWER_MOTOR, LOW);
  }
  else if(global_form == -1) {
    Serial.println("GLOBAL FORM = LOWER ARCH");
    digitalWrite(UPPER_MOTOR, HIGH);
    digitalWrite(LOWER_MOTOR, LOW);
  }
  else if(global_form == 0) {
    Serial.println("GLOBAL FORM = STRAIGHT");
    digitalWrite(UPPER_MOTOR, HIGH);
    digitalWrite(LOWER_MOTOR, HIGH);
  }
  else if(global_form == 1) {
    Serial.println("GLOBAL FORM = UPPER HUNCH");
    digitalWrite(UPPER_MOTOR, LOW);
    digitalWrite(LOWER_MOTOR, HIGH);
  }
  else if(global_form == 2) {
    Serial.println("GLOBAL FORM = BOTH HUNCH");
    digitalWrite(UPPER_MOTOR, LOW);
    digitalWrite(LOWER_MOTOR, LOW);
  }
  else {
    Serial.println("GLOBAL FORM = LOWER HUNCH");
    digitalWrite(UPPER_MOTOR, HIGH);
    digitalWrite(LOWER_MOTOR, LOW);
  }
}

void print_indiv_accel(int imu_id, int angle) {
  Serial.print("ACCEL "); Serial.print(String(imu_id)); Serial.print(": ");
  for(int i = 1; i < 2; ++i) {
    Serial.print(String(IMU[imu_id].accel_xyz[i].avg));
  }
  Serial.print("\t");
}

void print_accel_diff(int angle) {
  for(int i = 0; i < NUM_IMUS; ++i) {
    for(int j = i + 1; j < NUM_IMUS; ++j) {
      Serial.print(sign(IMU[j].accel_xyz[2].avg)*(IMU[j].accel_xyz[1].avg - IMU[j].accel_xyz[1].offset[i]) - 
                     sign(IMU[i].accel_xyz[2].avg)*(IMU[i].accel_xyz[1].avg - IMU[i].accel_xyz[1].offset[i]));
      Serial.print(",");
    }
  }
  Serial.print("\n");
}

void update_sensitivity(int mult) {
  for(int i = 0; i < NUM_IMUS; ++i) {
    for(int j = 0; j < NUM_IMUS; ++j) {
      sys[i].BA[j] += mult*INCR_DECR;
      sys[i].LA[j] += mult*INCR_DECR;
      sys[i].UH[j] -= mult*INCR_DECR;
      sys[i].BH[j] -= mult*INCR_DECR;
    }
  }
  Serial.println("UPDATED BA AT 90 DEGREES: " + String(sys[1].BA[2]));
  Serial.println("UPDATED BH AT 90 DEGREES: " + String(sys[0].BH[2]));
}

void write_string(String s) {
  uint8_t num_pieces = s.length()/19 + 1;
  uint8_t** pieces = new uint8_t*[num_pieces];

  for(uint8_t i = 0; i < num_pieces; i++){
    pieces[i] = new uint8_t[20];
  }  
  
  int last_substr_size = (s.length()-1)%19;
  for(uint8_t i = 0; i < s.length(); i += 19){
    String substr = s.substring(i, min(i + 19, s.length() - 1));
    substr.toCharArray(pieces[i/19], 20);
  }

  for(uint8_t i = 0; i < num_pieces; i++){
    if(i == num_pieces-1)
      BTLEserial.write(pieces[i], last_substr_size+1);
    else
      BTLEserial.write(pieces[i], 20);
  }

  for(uint8_t i = 0; i < num_pieces; i++){
    delete[] pieces[i];
  }
  
  delete[] pieces;
}

void Bluetooth() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
      Serial.println("====9====");
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      if(reset > 0) {
        start_over = ON;
      }
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      ++reset;
      Serial.println(F("* Connected!"));
      
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
      resetFunc();
    }
    // OK set the last status change to this one
    laststatus = status;
  }
  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      //Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
      switch(c) {
        case 's': // stop
          mode = OFF;
          Serial.println("DEVICE IS OFF");
          break;
        case 'g': // start
          mode = ON;
          Serial.println("DEVICE IS ON");
          break;
        case 'c': // calibrate
          cal = ON;
          Serial.println("READY TO CALIBRATE");
          break;
        case 'r': // reset
          mode = OFF;
          update_sensitivity((-1)*count);
          count = 0;
          Serial.println("DEVICE IS OFF");
          cal = ON;
          Serial.println("READY TO CALIBRATE");
          break;
        case 'l': // load offsets
          load = ON;
          Serial.println("LOADING PREVIOUS OFFSETS");
          break;
        case 'i': // increase sensitivity
          if(count < MAX_CHANGE) {
            update_sensitivity(1);
            ++count;
            Serial.println("DEVICE IS MORE SENSITIVE");
          }
          break;
        case 'd': // decrease sensitivity
          if(count > (-1)*MAX_CHANGE) {
            update_sensitivity(-1);
            --count;
            Serial.println("DEVICE IS LESS SENSITIVE");
          }
          break;
        case 't': // return to starting sensitivity
          update_sensitivity((-1)*count);
          count = 0;
        default:
          break;
      }
    }

    // Next up, see if we have any data to get from the Serial console
    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();
      write_string(s);
    }
  }
}

void setup_again() {
  while(cal == OFF && load == OFF) {
    Bluetooth();
  }

  int shouty_mark = 0;
  if(cal == ON) {
    write_string("y");
    for(int i = 0; i < 3; ++i) {
      calibrate_all_angles(i);
    }
  }
  else if(load == ON) {
    while(shouty_mark < 3) {
      BTLEserial.pollACI();
      if(BTLEserial.available()) {
        load_offsets(shouty_mark);
      }
    }
    load = OFF;
  }
}
