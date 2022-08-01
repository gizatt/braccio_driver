#include <Arduino.h>
#include <BraccioV2.h>


double last_blink_t;
const double BLINK_PERIOD_S = 1.;
bool blink_state = false;

double last_arm_update_t;
const double ARM_UPDATE_PERIOD_S = 0.01;

Braccio arm;

double get_t(){
  return ((double)micros()) / 1E6;
}

const uint8_t MAX_NUM_KNOTS = 128;

// Receive buffer.
const uint32_t RECV_BUFFER_SIZE = 1 + MAX_NUM_KNOTS * 7 * 4 + 4 + 4;
uint32_t buffer_index = 0;
uint8_t serial_recv_buffer[RECV_BUFFER_SIZE];

// Piecewise trajectory buffer.
uint32_t num_knots = 0;
double ts[MAX_NUM_KNOTS]; // Absolute time targets.
float qs[MAX_NUM_KNOTS*6];
float q_commanded[6];

void set_default_knot(){
  num_knots = 1;
  ts[0] = 0.;
  qs[0] = 90;
  qs[1] = 90;
  qs[2] = 90;
  qs[3] = 90;
  qs[4] = 90;
  qs[5] = 70;
  for (int i = 0; i < 6; i++){
    q_commanded[i] = qs[i];
  }
}

void update_arm_command(){
  double t = get_t();
  // Figure out index into current command buffer.
  int ind_left = 0;
  int ind_right = 0;
  float weight_left = 1.0;
  float weight_right = 0.0;

  if (num_knots == 0){
    SerialUSB.write("No command in buffer!\n");
    return;
  } else if (num_knots == 1 || t < ts[0]){
    // The first knot is either the only knot, or is
    // after the current time, so use it excluseively.
    ind_left = 0;
    ind_right = 0;
    weight_left = 1.0;
    weight_right = 0.0;
  } else {
    // Iterate through until we find a pair of knots that
    // are on either side of our current time.
    bool found_one = false;
    for (uint8_t i = 0; i < num_knots - 1; i++){
      if (ts[i] <= t && ts[i+1] > t){
        ind_left = i;
        ind_right = i+1;
        float dur = ts[i+1] - ts[i];
        weight_right = (t - ts[i]) / dur;
        weight_left = 1. - weight_right; 
        found_one = true;
        break;
      }
      if (!found_one){
        // t must be after the last knot.
        ind_left = num_knots - 1;
        ind_right = 0;
        weight_left = 1.;
        weight_right = 0.;
      }
    }
  }
  // Command this weighted combination of knots to the robot. (Requires awkward
  // conversion to a list of ints due to API...)
  for (int i = 0; i < 6; i++){
    q_commanded[i] = qs[ind_left*6 + i]*weight_left + qs[ind_right*6 + i]*weight_right; 
  }
  arm.setAllNow(q_commanded[0], q_commanded[1], q_commanded[2], q_commanded[3], q_commanded[4], q_commanded[5]);
}


// Called when buffer has terminated with 4 received zeros in a row.
// Attempts to parse a piecewise trajectory as described in the README.
void parse_buffer(){
  // The first byte should be the # of knots.
  uint8_t num_knots_in_buffer = serial_recv_buffer[4];
  if (num_knots_in_buffer > MAX_NUM_KNOTS){
    SerialUSB.write("Too many knots in message.\n");
    return;
  }
  // Make sure buffer size makes sense.
  uint32_t expected_buffer_size = 4 + 1 + num_knots_in_buffer * 7 * 4 + 4 + 4;
  if (buffer_index != expected_buffer_size){
    SerialUSB.write("Bad buffer size ");
    SerialUSB.print(buffer_index);
    SerialUSB.write(" for requested # of knots ");
    SerialUSB.print(num_knots_in_buffer);
    SerialUSB.write(".\n");
    return;
  }
  // Evaluate checksum.
  uint32_t expected_checksum = *(uint32_t *) (serial_recv_buffer + buffer_index - 8);
  uint32_t checksum = 0;
  for (int i = 0; i < num_knots_in_buffer * 7; i++){
    checksum += ((uint32_t *)(serial_recv_buffer + 5))[i];
  }
  if (checksum != expected_checksum){
    SerialUSB.write("Checksum mismatch.\n");
    return;
  }
  // Make sure the time points are ascending.
  float * data = (float *) (serial_recv_buffer + 5);
  float last_t = data[0];
  for (int i = 1; i < num_knots_in_buffer; i++){
    float next_t = data[i * 7 + 0];
    if (next_t <= last_t){
      SerialUSB.write("Non-ascending knot times!\n");
      return;
    }
    last_t = next_t;
  }

  // Good buffer, as far as we can tell! Copy over our targets.
  auto t = get_t();
  uint32_t offset = 0;
  uint32_t knot_i = 0;
  // Insert a "fake" 0-time knot at the current commanded position
  // if the first commanded time is not zero.
  float t0 = *(float *)(serial_recv_buffer + 5);
  if (t0 > 1E-3){
    ts[0] = 0.;
    for (int i = 0; i < 6; i++){
      qs[i] = q_commanded[i];
    }
    offset += 7;
    knot_i += 1;
  }
  for (int i = 0; i < num_knots_in_buffer; i++){
    double t_command = data[offset + 0] + t;
    ts[knot_i] = t_command;
    for (int j = 0; j < 6; j++){
      qs[j + knot_i*6] = data[offset + 1 + j];
    }
    offset += 7;
    knot_i += 1;
  }
  num_knots = knot_i;
  // Done!
  SerialUSB.write("Parsed ");
  SerialUSB.print(num_knots);
  SerialUSB.write(" from buffer with last knot time ");
  SerialUSB.print(ts[num_knots-1]);
  SerialUSB.write("\n");
}

void handle_serial_char(uint8_t byte) {
  if (buffer_index >= RECV_BUFFER_SIZE){
    buffer_index = 0;
    SerialUSB.write("Buffer overflow!\n");
  }


  if (buffer_index < 4 && byte != 0xff){
    // Messages must start with 0xffffffff; this isn't
    // part of it, so reset the buffer.
    buffer_index = 0;
    return;
  }

  serial_recv_buffer[buffer_index] = byte;
  buffer_index++;

  // Check is message has terminated, and dispatch to
  // the message parser if so.
  if (buffer_index > 4){
    uint8_t num_knots_in_buffer = serial_recv_buffer[4];
    uint32_t expected_buffer_size = 4 + 1 + num_knots_in_buffer * 7 * 4 + 4 + 4;
    if (buffer_index == expected_buffer_size){
      parse_buffer();
      buffer_index = 0;
    }
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  last_blink_t = get_t();

  // Set up default knot.
  set_default_knot();

  // Set up Serial comms; using Arduino Due USB port for higher serial bandwidth.
  SerialUSB.begin(115200);
  while (!SerialUSB){
    digitalWrite(LED_BUILTIN, false);
    delay(200);
    digitalWrite(LED_BUILTIN, true);
    delay(200);
  }
  SerialUSB.write("Braccio Driver connected and arm starting up.\n");

  // Arm default calibration and setup.
  arm.setJointCenter(WRIST_ROT, 90);
  arm.setJointCenter(WRIST, 90);
  arm.setJointCenter(ELBOW, 90);
  arm.setJointCenter(SHOULDER, 90);
  arm.setJointCenter(BASE_ROT, 90);
  arm.setJointCenter(GRIPPER, 70);//Rough center of gripper, default opening position

  arm.setJointMax(GRIPPER, 110);//Gripper closed, can go further, but risks damage to servos
  arm.setJointMin(GRIPPER, 50);//Gripper open, can't open further

  // Start arm to this default position.
  arm.begin(true);
}

void loop() {
  double t = get_t();
  if (t - last_blink_t > BLINK_PERIOD_S){
    blink_state = !blink_state;
    digitalWrite(LED_BUILTIN, blink_state);
    last_blink_t = t;
    SerialUSB.write("Heartbeat.\n");
  }

  if (t - last_arm_update_t > ARM_UPDATE_PERIOD_S){
    update_arm_command();
    arm.update();
    last_arm_update_t = t;
  }

  while (SerialUSB.available()){
    handle_serial_char(SerialUSB.read());
  }
}