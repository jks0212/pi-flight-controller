#include <iostream>
#include <iomanip>
/*
extern "C" {
	#include <linux/i2c.h>
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
*/
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <pigpio.h>
#include <RF24/RF24.h>

#define GYRO_CAL_NUMS 2000
#define LOOP_CYCLE 2000
#define I_INTEGRAL_LIMIT 400
#define PID_GAIN_LIMIT 400
#define LOOP_FREQUENCY (1000000 / LOOP_CYCLE)

#define FLIGHT_MODE_ACRO 1
#define FLIGHT_MODE_ANGLE 2
#define FLIGHT_MODE_HORIZON 3

#define SET_ROLL 1
#define SET_PITCH 2
#define SET_YAW 3
#define SET_POWER 4
#define SET_ROLL_OUTER_P 5
#define SET_ROLL_P 6
#define SET_ROLL_I 7
#define SET_ROLL_D 8
#define SET_PITCH_OUTER_P 9
#define SET_PITCH_P 10
#define SET_PITCH_I 11
#define SET_PITCH_D 12
#define SET_YAW_OUTER_P 13
#define SET_YAW_P 14
#define SET_YAW_I 15
#define SET_YAW_D 16
//#define TRIM_ROLL 20
//#define TRIM_PITCH 21
//#define TRIM_YAW 22

#define CMD_NONE              0x00
#define CMD_HEADER_START      0x6E
#define CMD_CONTROL           0x6E
#define CMD_GET_PID           0x6F
#define CMD_SET_PID           0x70
#define CMD_GET_TRIM          0X71
#define CMD_SET_TRIM          0x72
#define CMD_SET_TRIM_OFFSE    0x73

#define OFFSET_DIRECTION_FORWARD  1
#define OFFSET_DIRECTION_BACKWARD 2
#define OFFSET_DIRECTION_LEFT     3
#define OFFSET_DIRECTION_RIGHT    4

#define CONTROL_VALUE_THRESHOLD   40
#define POWER_VALUE_THRESHOLD     100
#define CMD_BUFF_LEN          8
//#define GREEN_LED_PIN         8
#define LED_GREEN 16
#define LED_RED   12

#define PID_START_ADDR        0
#define TRIM_START_ADDR       48

#define LED_ON_TIME   100

#define MOTOR_1 4
#define MOTOR_2 17
#define MOTOR_3 22
#define MOTOR_4 27
#define MOTOR_MIN 1000
#define MOTOR_MAX 1900
#define MOTOR_KEEP_RUNNING 1050
//#define MOTOR_KEEP_RUNNING 1000

enum PID_IDX {
	ROLL_OUTER_P, ROLL_P, ROLL_I, ROLL_D,
	PITCH_OUTER_P, PITCH_P, PITCH_I, PITCH_D,
	YAW_OUTER_P, YAW_P, YAW_I, YAW_D,
	PID_IDX_END
};

enum TRIM_IDX {
	TRIM_ROLL, TRIM_PITCH, TRIM_YAW, TRIM_IDX_END
};

using namespace std;

RF24 radio(RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);
const uint64_t pipes[2] = {0x0000000106L, 0x0000000100L};
char rfBuffer[CMD_BUFF_LEN] = {0};

struct timeval st, et, led_tv, sending_timer;
int green_led_time = 0;
int red_led_time = 0;
int sending_idx_pid = -1;
int sending_idx_trim = -1;

const double one = 1;
const double freq = LOOP_FREQUENCY;
const double rad = 65.5;
const double coeff_gyro_angle1 = one / freq / rad;
const double coeff_gyro_angle2 = coeff_gyro_angle1 * (3.14 / 180);

int target_power = 0;
float pid[12];

int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t mpu_temperature;
int32_t acc_total_vector;

int flight_mode = FLIGHT_MODE_ANGLE;
bool initialized = false;
bool delayed = false;

long gyro_x_cal, gyro_y_cal, gyro_z_cal;
double gyro_roll, gyro_pitch, gyro_yaw;
double angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
double angle_roll_output, angle_pitch_output;

double r_i_integral = 0, p_i_integral = 0, y_i_integral = 0;
double r_error_temp, p_error_temp, y_error_temp;
double last_r_error_temp, last_p_error_temp, last_y_error_temp;
double r_pid_control = 0, p_pid_control, y_pid_control;
double roll_sensitivity = 2, pitch_sensitivity = 2, yaw_sensitivity = 2;
int output[4];

double r_target_deg = 0, p_target_deg = 0, y_target_deg = 0;
float roll_trim = 0, pitch_trim = 0, yaw_trim = 0;

int mpu_i2c;

void init_mpu();
void init_rf24();
void read_mpu_data();
bool is_enough_sending_delay();
void send_pid_value(float value);
void send_trim_value();
void receive_messages();
void calibrate_gyro();
void calculate_angles();
void calculate_pid();
void set_motors_output();
void init_pid_values();
int utime_diff(timeval start, timeval end);
int parse_command_byte(char cmd, int default_val);
float make_float(uint8_t buf[]);
void write_pid();

// static int i = 0;

int main(int argc, char* argv[]) {

	if(gpioInitialise() < 0){
		cout << "Failed to initialize pigpio" << endl;
		return 0;
	}

	gpioSetMode(LED_RED, PI_OUTPUT);
	gpioSetMode(LED_GREEN, PI_OUTPUT);

	gpioWrite(LED_RED, 1);
	gpioWrite(LED_GREEN, 1);

	sleep(3);

	init_mpu();
	init_rf24();
	init_pid_values();
	calibrate_gyro();

	gpioSetPWMfrequency(MOTOR_1, 500);
	gpioSetPWMfrequency(MOTOR_2, 500);
	gpioSetPWMfrequency(MOTOR_3, 500);
	gpioSetPWMfrequency(MOTOR_4, 500);
	gpioSetPWMrange(MOTOR_1, 2000);
	gpioSetPWMrange(MOTOR_2, 2000);
	gpioSetPWMrange(MOTOR_3, 2000);
	gpioSetPWMrange(MOTOR_4, 2000);

	gettimeofday(&st, NULL);
	gettimeofday(&sending_timer, NULL);
	while(1){
		receive_messages();
		read_mpu_data();
		calculate_angles();
		calculate_pid();
		set_motors_output();

		gettimeofday(&et, NULL);
		int cycle_time = utime_diff(st, et);
		delayed = false;
		if(cycle_time >= LOOP_CYCLE){
			// cout << "Cycle over: " << cycle_time << "us" << endl;
			red_led_time = LED_ON_TIME;
			delayed = true;
		} else{
			while(utime_diff(st, et) < LOOP_CYCLE){
				gettimeofday(&et, NULL);
			}
		}

		if(!delayed){
			gpioPWM(MOTOR_1, output[0]);
			gpioPWM(MOTOR_2, output[1]);
			gpioPWM(MOTOR_3, output[2]);
			gpioPWM(MOTOR_4, output[3]);
		}

		gettimeofday(&led_tv, NULL);

		int elapsed = utime_diff(st, led_tv) / 1000;
		if(green_led_time > 0){
			gpioWrite(LED_GREEN, 1);
			green_led_time -= elapsed;
		} else{
			gpioWrite(LED_GREEN, 0);
		}
		if(red_led_time > 0){
			gpioWrite(LED_RED, 1);
			red_led_time -= elapsed;
		} else{
			gpioWrite(LED_RED, 0);
		}

	//	if(angle_roll_output > 4 || angle_pitch_output > 4 
	//		|| angle_roll_output < -4 || angle_pitch_output < -4){
	//		red_led_time = LED_ON_TIME;
	//	}


		gettimeofday(&st, NULL);

	//	if(i++ > 20){
		//	cout << angle_roll_output << ", " << angle_pitch_output << ", " << gyro_z << endl;
	//		cout << gyro_x << ", " << gyro_y << ", " << gyro_z << endl;
	//		i = 0;
	//	}

	//	cout << "acc x,y,z : " << acc_x << ", " << acc_y << ", " << acc_z << endl;
	//	cout << "gyro r,p,y : " << setw(8) << setprecision(3) << gyro_roll << ", ";
	//	cout << setw(8) << setprecision(3) << gyro_pitch << ", ";
	//	cout << setw(8) << setprecision(3) << gyro_yaw << endl;
	//	cout << "angle roll, pitch : " << angle_roll_output << ", " << angle_pitch_output << endl;

	}

	return 0;
}

void init_mpu(){
	mpu_i2c = i2cOpen(1, 0x68, 0);
	if(i2cWriteByteData(mpu_i2c, 0x6B, 0x00) < 0){
		cout << "Failed to congifure MPU." << endl;
		exit(1);
	}
	if(i2cWriteByteData(mpu_i2c, 0x1C, 0x10) < 0){
		cout << "Failed to congifure MPU." << endl;
		exit(1);
	}
	if(i2cWriteByteData(mpu_i2c, 0x1B, 0x08) < 0){
		cout << "Failed to confiture MPU." << endl;
		exit(1);
	}
	if(i2cWriteByteData(mpu_i2c, 0x1A, 0x02) < 0){
//	if(i2cWriteByteData(mpu_i2c, 0x1A, 0x03) < 0){
		cout << "Failed to confiture MPU." << endl;
		exit(1);
	}
//	int8_t power = i2cReadByteData(mpu_i2c, 0x6B);
//	i2cWriteByteData(mpu_i2c, 0x6b, ~(1 << 6) & power);
}

void init_rf24(){
	radio.begin();
	radio.setAutoAck(false);
	radio.setRetries(0, 15);
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1, pipes[0]);
	radio.startListening();
	radio.printDetails();
}

void read_mpu_data(){
	char buffer[32];
	i2cReadI2CBlockData(mpu_i2c, 0x3B, buffer, 14);

	acc_x = buffer[0] << 8 | buffer[1];
	acc_y = buffer[2] << 8 | buffer[3];
	acc_z = buffer[4] << 8 | buffer[5];
	mpu_temperature = buffer[6] << 8 | buffer[7];
	gyro_x = buffer[8] << 8 | buffer[9];
	gyro_y = buffer[10] << 8 | buffer[11];
	gyro_z = buffer[12] << 8 | buffer[13];
}

void calibrate_gyro(){

	struct timeval start, end;

	cout << "Gyro calibration started." << endl;

	for(int i=0; i<GYRO_CAL_NUMS; i++){
		gettimeofday(&start, NULL);

		// if(i % 125 == 0){
		// 	cout << ".";
		// }
	
		read_mpu_data();
		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;

		int led_val = i % 20;
		if(led_val >=0 && led_val < 10){		
			gpioWrite(LED_GREEN, 1);
			gpioWrite(LED_RED, 1);
		} else{
			gpioWrite(LED_GREEN, 0);
			gpioWrite(LED_RED, 0);
		}

		gettimeofday(&end, NULL);

		while(utime_diff(start, end) < 4000){
			gettimeofday(&end, NULL);
		}

	}

	gyro_x_cal /= GYRO_CAL_NUMS;
	gyro_y_cal /= GYRO_CAL_NUMS;
	gyro_z_cal /= GYRO_CAL_NUMS;

	cout << "Gyro calibration done." << endl;
}

void calculate_angles(){
	gyro_x -= gyro_x_cal;
	gyro_y -= gyro_y_cal;
	gyro_z -= gyro_z_cal;

	gyro_roll = (gyro_roll * 0.7) + ((gyro_x / 65.5) * 0.3);
	gyro_pitch = (gyro_pitch * 0.7) + ((gyro_y / 65.5) * 0.3);
	gyro_yaw = (gyro_yaw * 0.7) + ((gyro_z / 65.5) * 0.3);

	// angle_pitch += gyro_x * 0.0000611;
	angle_pitch += gyro_x * coeff_gyro_angle1;
	// angle_roll += gyro_y * 0.0000611;
	angle_roll += gyro_y * coeff_gyro_angle1;

	// angle_pitch += angle_roll * sin(gyro_z * 0.000000533);
	angle_pitch += angle_roll * sin(gyro_z * coeff_gyro_angle2);
	// angle_roll -= angle_pitch * sin(gyro_z * 0.000000533);
	angle_roll -= angle_pitch * sin(gyro_z * coeff_gyro_angle2);

	acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
	if(abs(acc_y) < acc_total_vector){
		angle_pitch_acc = asin((double)acc_y / acc_total_vector) * 57.296;
	}
	if(abs(acc_x) < acc_total_vector){
		angle_roll_acc = asin((double)acc_x / acc_total_vector) * -57.296;
	}

	angle_pitch_acc -= 0.0;
	angle_roll_acc -= 0.0;

	if(initialized){
		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
	} else {
		angle_pitch = angle_pitch_acc;
		angle_roll = angle_roll_acc;
		initialized = true;
	}

	angle_roll_output = angle_roll_output * 0.9 + angle_pitch * 0.1;
	angle_pitch_output = angle_pitch_output * 0.9 + angle_roll * 0.1;
}

void calculate_pid(){
	switch(flight_mode){
		case FLIGHT_MODE_ANGLE:
			r_error_temp = gyro_roll - pid[ROLL_OUTER_P] * (r_target_deg - roll_trim - angle_roll_output);
			p_error_temp = gyro_pitch - pid[PITCH_OUTER_P] * (p_target_deg - pitch_trim - angle_pitch_output);
			y_error_temp = gyro_yaw - y_target_deg * yaw_sensitivity;
			break;
		case FLIGHT_MODE_ACRO:
			r_error_temp = gyro_roll - r_target_deg * roll_sensitivity;
			p_error_temp = gyro_pitch - p_target_deg * pitch_sensitivity;
			y_error_temp = gyro_yaw - y_target_deg * yaw_sensitivity;
			break;
		case FLIGHT_MODE_HORIZON:
			break;
	}

	if(target_power > 0){
		r_i_integral += pid[ROLL_I] * r_error_temp;
		p_i_integral += pid[PITCH_I] * p_error_temp;
		y_i_integral += pid[YAW_I] * y_error_temp;
	} else {
		r_i_integral = 0;
		p_i_integral = 0;
		y_i_integral = 0;
		last_r_error_temp = 0;
		last_p_error_temp = 0;
		last_y_error_temp = 0;
	}

	if(r_i_integral > I_INTEGRAL_LIMIT) r_i_integral = I_INTEGRAL_LIMIT;
	else if(r_i_integral < -I_INTEGRAL_LIMIT) r_i_integral = -I_INTEGRAL_LIMIT;
	if(p_i_integral > I_INTEGRAL_LIMIT) p_i_integral = I_INTEGRAL_LIMIT;
	else if(p_i_integral < -I_INTEGRAL_LIMIT) p_i_integral = -I_INTEGRAL_LIMIT;
	if(y_i_integral > I_INTEGRAL_LIMIT) y_i_integral = I_INTEGRAL_LIMIT;
	else if(y_i_integral < -I_INTEGRAL_LIMIT) y_i_integral = -I_INTEGRAL_LIMIT;

	r_pid_control = pid[ROLL_P] * r_error_temp + r_i_integral + pid[ROLL_D] * (r_error_temp - last_r_error_temp);
	p_pid_control = pid[PITCH_P] * p_error_temp + p_i_integral + pid[PITCH_D] * (p_error_temp - last_p_error_temp);
	y_pid_control = pid[YAW_P] * y_error_temp + y_i_integral + pid[YAW_D] * (y_error_temp - last_y_error_temp);

	last_r_error_temp = r_error_temp;
	last_p_error_temp = p_error_temp;
	last_y_error_temp = y_error_temp;

	if(r_pid_control > PID_GAIN_LIMIT) r_pid_control = PID_GAIN_LIMIT;
	else if(r_pid_control < -PID_GAIN_LIMIT) r_pid_control = -PID_GAIN_LIMIT;
	if(p_pid_control > PID_GAIN_LIMIT) p_pid_control = PID_GAIN_LIMIT;
	else if(p_pid_control < -PID_GAIN_LIMIT) p_pid_control = -PID_GAIN_LIMIT;
	if(y_pid_control > PID_GAIN_LIMIT) y_pid_control = PID_GAIN_LIMIT;
	else if(y_pid_control < -PID_GAIN_LIMIT) y_pid_control = -PID_GAIN_LIMIT;
}

void set_motors_output(){
	if(target_power > 0){
		int power = MOTOR_MIN + target_power * 9;
		
		output[0] = power + r_pid_control - p_pid_control - y_pid_control;
		output[1] = power + r_pid_control + p_pid_control + y_pid_control;
		output[2] = power - r_pid_control - p_pid_control + y_pid_control;
		output[3] = power - r_pid_control + p_pid_control - y_pid_control;

		for(int i=0; i<4; i++){
			output[i] = output[i] > MOTOR_MAX ? MOTOR_MAX : output[i];
			output[i] = output[i] < MOTOR_KEEP_RUNNING ? MOTOR_KEEP_RUNNING : output[i];
		}
	} else{
		output[0] = MOTOR_MIN;
		output[1] = MOTOR_MIN;
		output[2] = MOTOR_MIN;
		output[3] = MOTOR_MIN;
	}
}

void receive_messages(){
	if(radio.available()){
		/*
		int payload_size = radio.getDynamicPayloadSize();
		if(payload_size > 1){
			char* payload = new char[payload_size + 1];
			radio.read(payload, payload_size);
			payload[payload_size] = '\0';
		}
		*/
	//	bool done = radio.read(rfBuffer, CMD_BUFF_LEN);
	//	if(done){

		int payload_size = radio.getDynamicPayloadSize();
		if(payload_size > 0){
			radio.read(rfBuffer, CMD_BUFF_LEN);
			if(rfBuffer[0] == CMD_CONTROL){
				r_target_deg = parse_command_byte(rfBuffer[1], r_target_deg);
				p_target_deg = parse_command_byte(rfBuffer[2], p_target_deg);
				y_target_deg = parse_command_byte(rfBuffer[3], y_target_deg);
				target_power = rfBuffer[4] <= POWER_VALUE_THRESHOLD ? rfBuffer[4] : target_power;
			
			} else if(rfBuffer[0] == CMD_GET_PID){
				sending_idx_pid = 0;
			} else if(rfBuffer[0] == CMD_GET_TRIM){
				sending_idx_trim = 0;
			} else if(rfBuffer[0] == CMD_SET_PID){
				uint8_t values[4] = {rfBuffer[2], rfBuffer[3], rfBuffer[4], rfBuffer[5]};
				pid[(uint8_t)rfBuffer[1]] = make_float(values);
				write_pid();
			} else if(rfBuffer[0] == CMD_GET_TRIM){
			
			} else if(rfBuffer[0] == CMD_SET_TRIM){
				uint8_t values[4] = {rfBuffer[2], rfBuffer[3], rfBuffer[4], rfBuffer[5]};
				switch(rfBuffer[1]){
					case TRIM_ROLL:
						roll_trim = make_float(values);
						break;
					case TRIM_PITCH:
						pitch_trim = make_float(values);
						break;
					 case TRIM_YAW:
						yaw_trim = make_float(values);
						break;
				}
			} else if(rfBuffer[0] == CMD_SET_TRIM_OFFSE){
				uint8_t direction = rfBuffer[1];
				switch(direction){
					case OFFSET_DIRECTION_FORWARD:
						pitch_trim += 0.1;
						break;
					case OFFSET_DIRECTION_BACKWARD:
						pitch_trim -= 0.1;
						break;
					case OFFSET_DIRECTION_LEFT:
						roll_trim -= 0.1;
						break;
					case OFFSET_DIRECTION_RIGHT:
						roll_trim += 0.1;
						break;
				}
			}

			green_led_time = LED_ON_TIME;

		} else {
			cout << "Error to read RF data" << endl;
		}

		for(int i=0; i<CMD_BUFF_LEN; i++){
			rfBuffer[i] = 0;
		}

		/*
		while(!done){
			// Fetch the payload, and see if this was the last one.
			// done = radio.read( &got_time, sizeof(unsigned long) );
			int temp;
			done = radio.read(temp, 1);
		}
		*/
	}

	if(sending_idx_pid != -1){
		send_pid_value(pid[sending_idx_pid]);
	}else if(sending_idx_trim != -1){
		send_trim_value();
	}
}

void init_pid_values(){
	pid[ROLL_OUTER_P] = 0;
	pid[ROLL_P] = 1.7;
	pid[ROLL_I] = 0.03;
	pid[ROLL_D] = 13;
	pid[PITCH_OUTER_P] = 0;
	pid[PITCH_P] = 1.7;
	pid[PITCH_I] = 0.03;
	pid[PITCH_D] = 13;
	pid[YAW_OUTER_P] = 0;
	pid[YAW_P] = 0;
	pid[YAW_I] = 0;
	pid[YAW_D] = 0;
}

int utime_diff(timeval start, timeval end){
	if(end.tv_sec - start.tv_sec > 0){
		return (1000000 + end.tv_usec) - start.tv_usec;
	} else{
		return end.tv_usec - start.tv_usec;
	}
}

int parse_command_byte(char cmd, int default_val){
	int rs = cmd;
	if(cmd > 127){
		rs = cmd - 255;
	}
	return abs(rs) > CONTROL_VALUE_THRESHOLD ? default_val : rs;
}

float make_float(uint8_t buf[]){
	uint32_t f_temp = (((uint32_t)buf[0]&0xFF)<<24);
	f_temp += (((uint32_t)buf[1]&0xFF)<<16);
	f_temp += (((uint32_t)buf[2]&0xFF)<<8);
	f_temp += (((uint32_t)buf[3]&0xFF));
	float f = *((float*)&f_temp);
	return f;
}

bool is_enough_sending_delay(){
	struct timeval now;
	gettimeofday(&now, NULL);

	if(now.tv_sec - sending_timer.tv_sec > 0){
		gettimeofday(&sending_timer, NULL);
		return true;
	} else if(now.tv_usec - sending_timer.tv_usec > 10000){
		gettimeofday(&sending_timer, NULL);
		return true;
	} else{
		return false;
	}
}

void send_pid_value(float value){
	if(!is_enough_sending_delay()){
		return;
	}
  
	if(sending_idx_pid >= PID_IDX_END){
		sending_idx_pid = -1;
		return;
	}
  
	union Scomp_double {
		float temp;
		uint8_t byte_s[4];
	} s_double;
  
	uint8_t temp[CMD_BUFF_LEN];
//	uint32_t u_temp = (uint32_t)value;

	s_double.temp = value;
  
	temp[0] = CMD_GET_PID;
	temp[1] = sending_idx_pid++;
	temp[2] = s_double.byte_s[3];
	temp[3] = s_double.byte_s[2];
	temp[4] = s_double.byte_s[1];
	temp[5] = s_double.byte_s[0];
	temp[6] = 0;
	temp[7] = 0;

	radio.stopListening();
	if(!radio.write(&temp, CMD_BUFF_LEN)){
		cout << "Error to send PID value" << endl;
	}
	radio.startListening();
}

void send_trim_value(){
	if(!is_enough_sending_delay()){
		return;
	}

	if(sending_idx_trim >= TRIM_IDX_END){
		sending_idx_trim = -1;
		return;
	}

	union Scomp_double {
		float temp;
		uint8_t byte_s[4];
	} s_double;

	uint8_t temp[CMD_BUFF_LEN];

	switch(sending_idx_trim){
		case TRIM_ROLL:
			s_double.temp = roll_trim; 
			break;
		case TRIM_PITCH: 
			s_double.temp = pitch_trim; 
			break;
		case TRIM_YAW: 
			s_double.temp = yaw_trim; 
			break;
	}

	temp[0] = CMD_GET_TRIM;
	temp[1] = sending_idx_trim++;
	temp[2] = s_double.byte_s[3];
	temp[3] = s_double.byte_s[2];
	temp[4] = s_double.byte_s[1];
	temp[5] = s_double.byte_s[0];
	temp[6] = 0;
	temp[7] = 0;

	radio.stopListening();
	if(!radio.write(&temp, CMD_BUFF_LEN)){
		cout << "Error to send Trim value" << endl;
	}
	radio.startListening();
}

void write_pid(){
	string pid_str = "";
	for(int i=0; i<12; i++){
		if(i != 0){
			pid_str = pid_str + ",";
		}
		pid_str = pid_str + to_string(pid[i]);
	}

	char temp[200];
	strcpy(temp, pid_str.c_str());
	FILE *fp;
	if((fp = fopen("config", "w")) != NULL){
		fprintf(fp, "%s", temp);
		fclose(fp);
	}
}











