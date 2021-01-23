#include <SPI.h>
#include <mcp_can.h>
/*
Author: Mihai Moldovanu
Youtube: https://www.youtube.com/channel/UC1CS67BvP8PaVJRK6wxhbCg
Mail: mihaim@tfm.ro
*/

// Mini cheetah actuator testing code
//
// Hardware needed: Wemos d1 mini + mcp215 can
// Last Tested : 24 ian 2021



// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set values
float p_in = 0.0f;
float v_in = -40.0f;
float kp_in = 100.0f;
float kd_in = 5.0f;
float t_in = -4.0f;
// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

long previousMillis = 0;

// Wemos D1 mini CS pin is D8 ( translated to pin 15 )
const int spiCSPin = 15;

MCP_CAN CAN(spiCSPin);

// Motor Mode on command
byte MotorModeOn[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};

// Motor Mode off command
byte MotorModeOff[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};


void setup()
{
  Serial.begin(115200);
  Serial.println("Mihai - CANBUS firmware 1.0");

  while (CAN_OK != CAN.begin(CAN_1000KBPS,MCP_8MHz)) {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }

  Serial.println("CAN BUS Shield Init OK!");

  // Enable motor mode
  while ( CAN_OK != CAN.sendMsgBuf(0x01, 0, 8, MotorModeOn))
    {
      Serial.println("Error enabling motor mode");
      delay(1000);
    }
}


int a = 0;

void loop()
{
// Moving faster to 6.0 and slow back to 0
if ( a == 0 ) 
  p_in = p_in + 0.5;
else 
 p_in = p_in - 0.1;

if ( p_in > 6 ) 
  a = 1;

if ( p_in < 0.2 )
  a = 0;

  pack_cmd();

// receive CAN
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    unpack_reply();
    //print data
    Serial.print(p_in);
    Serial.print(" ");
    Serial.print(p_out);
    Serial.print(" ");
    Serial.print(v_out);
    Serial.print(" ");
    Serial.println(t_out);
  }

// during testing don't issue commands to fast
delay( 1500 );
}


/*
 Pack command and send it to canbus
*/
void pack_cmd(){
  byte buf[8];

  // CAN Command Packet Structure ///
  // 16 bit position command, between -4*pi and 4*pi
  // 12 bit velocity command, between -30 and + 30 rad/s
  // 12 bit kp, between 0 and 500 N-m/rad
  // 12 bit kd, between 0 and 100 N-m*s/rad
  // 12 bit feed forward torque, between -18 and 18 N-m
  // CAN Packet is 8 8-bit words
  // Formatted as follows.  For each quantity, bit 0 is LSB
  // 0: [position[15-8]]
  // 1: [position[7-0]] 
  // 2: [velocity[11-4]]
  // 3: [velocity[3-0], kp[11-8]]
  // 4: [kp[7-0]]
  // 5: [kd[11-4]]
  // 6: [kd[3-0], torque[11-8]]
  // 7: [torque[7-0]]

  // limit data to be within bounds 
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);
  // convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  // pack ints into the can buffer

  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

/*
Unpack message received on canbus
*/
void unpack_reply(){
  // CAN Reply Packet Structure ///
  // 16 bit position, between -4*pi and 4*pi
  // 12 bit velocity, between -30 and + 30 rad/s
  // 12 bit current, between -40 and 40;
  // CAN Packet is 5 8-bit words
  // Formatted as follows.  For each quantity, bit 0 is LSB
  // 0: [position[15-8]]
  // 1: [position[7-0]] 
  // 2: [velocity[11-4]]
  // 3: [velocity[3-0], current[11-8]]
  // 4: [current[7-0]]

  long unsigned int rxId;

  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&len, buf);

  // unpack ints from can buffer
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  // convert uints to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
} 

/*
 Converts a float to an unsigned int, given range and number of bits
*/
unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

/*
 Converts unsigned int to float, given range and number of bits
*/
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}
