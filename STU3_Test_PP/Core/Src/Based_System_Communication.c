/*
 * Based_System_Communication.c
 *
 *  Created on: Apr 21, 2025
 *      Author: User
 */

#include "Based_System_Communication.h"

void modbus_heartbeat_init(ModbusHandleTypedef* hmodbus) {
	hmodbus->RegisterAddress[0x00].U16 = 22881;
}

void modbus_heartbeat(ModbusHandleTypedef *hmodbus) {
	if (hmodbus->RegisterAddress[0x00].U16 == 18537) {
		hmodbus->RegisterAddress[0x00].U16 = 22881;
	}
}

uint8_t modbus_Base_System_Status(ModbusHandleTypedef *hmodbus) {
	uint16_t status = hmodbus->RegisterAddress[0x01].U16;
	if (status == 1) {

	}
	return status;
}

//void modbus_servo_Status(ModbusHandleTypedef *hmodbus) {
//	if () { // servo has reached its lowest position
//		hmodbus.RegisterAddress[0x03].U16 = 1;
//	} else if (){ // servo has reached its highest position
//		hmodbus.RegisterAddress[0x03].U16 = 2;
//	} else { // servo still moving
//		hmodbus.RegisterAddress[0x03].U16 = 0;
//	}
//}
//
//void modbus_write_servo_up(ModbusHandleTypedef *hmodbus, uint8_t Servo_PWM) {
//	if (hmodbus->RegisterAddress[0x04].U16 == 1) {
//		// Activates servo upward motion
//	}
//}
//
//void modbus_write_servo_down(ModbusHandleTypedef *hmodbus, uint8_t Servo_PWM) {
//	if (hmodbus->RegisterAddress[0x05].U16 == 1) {
//		// Activates servo downward motion
//	}
//}

uint8_t R_Theta_moving_Status(ModbusHandleTypedef *hmodbus, uint8_t Moving_Status) {
	hmodbus->RegisterAddress[0x10].U16 = Moving_Status;
	return Moving_Status;
}

void modbus_r_position(ModbusHandleTypedef *hmodbus, float r_pos) {
	hmodbus->RegisterAddress[0x11].U16 = r_pos;
}

void modbus_theta_position(ModbusHandleTypedef *hmodbus, float theta_pos) {
	hmodbus->RegisterAddress[0x12].U16 = theta_pos;
}
void modbus_r_velocity(ModbusHandleTypedef *hmodbus, float r_Velo) {
	hmodbus->RegisterAddress[0x13].U16 = r_Velo;
}
void modbus_theta_velocity(ModbusHandleTypedef *hmodbus, float theta_Velo) {
	hmodbus->RegisterAddress[0x14].U16 = theta_Velo;
}
void modbus_r_acceleration(ModbusHandleTypedef *hmodbus, float r_accel) {
	hmodbus->RegisterAddress[0x15].U16 = r_accel;
}
void modbus_theta_acceleration(ModbusHandleTypedef *hmodbus, float theta_accel) {
	hmodbus->RegisterAddress[0x16].U16 = theta_accel;
}
void modbus_Update_All(ModbusHandleTypedef *hmodbus, float r_pos,
		float theta_pos, float r_Velo, float theta_Velo, float r_accel,
		float theta_accel) {
	hmodbus->RegisterAddress[0x11].U16 = r_pos;
	hmodbus->RegisterAddress[0x12].U16 = theta_pos;
	hmodbus->RegisterAddress[0x13].U16 = r_Velo;
	hmodbus->RegisterAddress[0x14].U16 = theta_Velo;
	hmodbus->RegisterAddress[0x15].U16 = r_accel;
	hmodbus->RegisterAddress[0x16].U16 = theta_accel;
}

void set_Target_Position_ten_points(ModbusHandleTypedef *hmodbus, float point,
		uint8_t index) {
	hmodbus->RegisterAddress[0x20 + index].U16 = point;
}
uint16_t modbus_set_goal_r_position(ModbusHandleTypedef *hmodbus) {
	uint16_t goal_r_position = hmodbus->RegisterAddress[0x40].U16;
	return goal_r_position;
}
uint16_t modbus_set_goal_theta_position(ModbusHandleTypedef *hmodbus) {
	uint16_t goal_theta_position = hmodbus->RegisterAddress[0x41].U16;
	return goal_theta_position;
}

