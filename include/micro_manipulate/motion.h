#ifndef _MOTION_H
#define _MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#ifdef RM_API_EXPORTS
#define RM_API __declspec(dllexport)
#else
#define RM_API __declspec(dllimport)
#endif
#else
#define RM_API  
#endif

	typedef int8_t rm_axis_handle;

	typedef enum {
		MOTION_COMMAND_NONE = 0,
		MOTION_COMMAND_GO_HOME = 1,
		MOTION_COMMAND_DELAY = 2,
		MOTION_COMMAND_MOVE_ABSOLUTE = 3,
		MOTION_COMMAND_PUSH = 4,
		MOTION_COMMAND_MOVE_RELATIVE = 5,
		MOTION_COMMAND_CLOSE_LOOP_PUSH = 6
	} MOTION_COMMAND;

	typedef struct {
		MOTION_COMMAND type;
		float position;
		float velocity;
		float acceleration;
		float deacceleration;
		float tolerance;
		float push_force;
		float push_distance;
		int32_t delay;
		int32_t next_command_index;
	} motion_command_t;

	typedef enum {
		MOTION_IO_IN_NULL,
		MOTION_IO_IN_GO_HOME,
		MOTION_IO_IN_ERROR_RESET,
		MOTION_IO_IN_START,
		MOTION_IO_IN_SERVO,
		MOTION_IO_IN_STOP,
		MOTION_IO_IN_PAUSE,
		MOTION_IO_IN_FORCE_INPUT,
		MOTION_IO_IN_POSITION_0,
		MOTION_IO_IN_POSITION_1,
		MOTION_IO_IN_POSITION_2,
		MOTION_IO_IN_POSITION_3,
		MOTION_IO_IN_POSITION_4,
		MOTION_IO_IN_POSITION_5,
		MOTION_IO_IN_POSITION_6,
		MOTION_IO_IN_POSITION_7,
		MOTION_IO_IN_POSITION_8,
		MOTION_IO_IN_POSITION_9,
		MOTION_IO_IN_POSITION_10,
		MOTION_IO_IN_POSITION_11,
		MOTION_IO_IN_POSITION_12,
		MOTION_IO_IN_POSITION_13,
		MOTION_IO_IN_POSITION_14,
		MOTION_IO_IN_POSITION_15,
		MOTION_IO_IN_RESERVED_0,
		MOTION_IO_IN_POSITION_START,
		MOTION_IO_IN_RESERVED_1,
		MOTION_IO_IN_RESERVED_2,
		MOTION_IO_IN_RESERVED_3,
		MOTION_IO_IN_RESERVED_4,
		MOTION_IO_IN_RESERVED_5,
		MOTION_IO_IN_RESERVED_6,
		MOTION_IO_IN_RESERVED_7,
		MOTION_IO_IN_RESERVED_8,
		MOTION_IO_IN_SAVE_PARAMETERS,
		MOTION_IO_IN_LOAD_PARAMETERS,
		MOTION_IO_IN_SAVE_POSITIONS,
		MOTION_IO_IN_LOAD_POSITIONS,
		MOTION_IO_IN_RESERVED_9,
		MOTION_IO_IN_RESERVED_10,
		MOTION_IO_IN_RESERVED_11,
		MOTION_IO_IN_RESERVED_12,
		MOTION_IO_IN_SWITCH_COMMAND_PULSE_ENABLE,
		MOTION_IO_IN_RESERVED_13,
		MOTION_IO_IN_RESERVED_14,
		MOTION_IO_IN_SWITCH_HARDWARE_IO_ENABLE

	} MOTION_IO_IN;

	typedef enum {
		MOTION_IO_OUT_NULL,
		MOTION_IO_OUT_GONE_HOME,
		MOTION_IO_OUT_ALARM,
		MOTION_IO_OUT_IN_POSITION,
		MOTION_IO_OUT_REACH_POSITION_TARGET,
		MOTION_IO_OUT_MOVING,
		MOTION_IO_OUT_REACH_0,
		MOTION_IO_OUT_REACH_1,
		MOTION_IO_OUT_REACH_2,
		MOTION_IO_OUT_REACH_3,
		MOTION_IO_OUT_REACH_4,
		MOTION_IO_OUT_REACH_5,
		MOTION_IO_OUT_REACH_6,
		MOTION_IO_OUT_REACH_7,
		MOTION_IO_OUT_REACH_8,
		MOTION_IO_OUT_REACH_9,
		MOTION_IO_OUT_REACH_10,
		MOTION_IO_OUT_REACH_11,
		MOTION_IO_OUT_REACH_12,
		MOTION_IO_OUT_REACH_13,
		MOTION_IO_OUT_REACH_14,
		MOTION_IO_OUT_REACH_15,
		MOTION_IO_OUT_IN_GLOBAL_ZONE_0,
		MOTION_IO_OUT_UPPER_GLOBAL_ZONE_0,
		MOTION_IO_OUT_LOWER_GLOBAL_ZONE_0,
		MOTION_IO_OUT_IN_GLOBAL_ZONE_1,
		MOTION_IO_OUT_UPPER_GLOBAL_ZONE_1,
		MOTION_IO_OUT_LOWER_GLOBAL_ZONE_1,
	} MOTION_IO_OUT;

	RM_API void rm_init();

	#define RM_DEVICE_NAME_MAX_LEN 0xFF
	#define RM_DEVICE_MAX_COUNT 0xFF

	RM_API int rm_list_modbus_devices(char device_names[RM_DEVICE_MAX_COUNT][RM_DEVICE_NAME_MAX_LEN]);

	// constructors
	RM_API rm_axis_handle rm_open_axis_modbus_rtu(const char* device, int baudrate, uint8_t axis_no);

	RM_API rm_axis_handle rm_open_axis_canopen_socket(const char* socketcand_address, int port, const char* device, uint8_t node_id);

	RM_API rm_axis_handle rm_open_axis_canopen_device(const char* device, int bitrate, uint8_t node_id);

	// modbus configuration
	RM_API void rm_config(uint32_t timeout_retry, uint32_t switch_delay_ms);

	// destructor
	RM_API void rm_close_axis(rm_axis_handle handle);

	RM_API void rm_close_all_axis();

	RM_API void rm_unlock_all_axis();

	RM_API bool rm_has_com_error(rm_axis_handle handle);

	// standard io api functions
	RM_API void rm_set_io_in(rm_axis_handle handle, MOTION_IO_IN in, bool value);

	RM_API bool rm_get_io_out(rm_axis_handle handle, MOTION_IO_OUT out);
	
	RM_API MOTION_IO_IN rm_get_io_in_map(rm_axis_handle handle, int port);

	RM_API MOTION_IO_OUT rm_get_io_out_map(rm_axis_handle handle, int port);

	RM_API void rm_set_io_in_map(rm_axis_handle handle, int port, MOTION_IO_IN in);

	RM_API void rm_set_io_out_map(rm_axis_handle handle, int port, MOTION_IO_OUT out);
	
	// RM_API void rm_set_variable(rm_axis_handle

	// standard motion api functions
	RM_API void rm_move_absolute(rm_axis_handle handle, float position, float velocity, float acceleration, float deacceleration, float band);

	RM_API void rm_push(rm_axis_handle handle, float force, float distance, float velocity);

	RM_API void rm_go_home(rm_axis_handle handle);

	RM_API bool rm_is_moving(rm_axis_handle handle);

	RM_API bool rm_is_position_reached(rm_axis_handle handle);

	RM_API void rm_set_command(rm_axis_handle handle, int32_t index, motion_command_t command);

	RM_API motion_command_t rm_get_command(rm_axis_handle handle, int32_t index);

	RM_API void rm_trig_command(rm_axis_handle handle, int32_t index);

	RM_API void rm_execute_command(rm_axis_handle handle, motion_command_t command);

	RM_API bool rm_is_command_executed(rm_axis_handle handle, int32_t index);

	RM_API void rm_save_commands(rm_axis_handle handle);

	RM_API void rm_load_commands(rm_axis_handle handle);

	RM_API float rm_read_current_position(rm_axis_handle handle);

	RM_API float rm_read_current_velocity(rm_axis_handle handle);

	RM_API float rm_read_current_torque(rm_axis_handle handle);

	RM_API uint32_t rm_read_error_code(rm_axis_handle handle);

	RM_API void rm_reset_error(rm_axis_handle handle);

	RM_API void rm_set_servo_on_off(rm_axis_handle handle, bool enable);

	RM_API void rm_stop(rm_axis_handle handle);	

	RM_API void rm_load_parameters(rm_axis_handle handle);

	RM_API void rm_save_parameters(rm_axis_handle handle);
	
	// modbus parameters access
	RM_API int32_t rm_get_parameter_int32(rm_axis_handle handle, uint16_t address);
	RM_API float rm_get_parameter_float(rm_axis_handle handle, uint16_t address);
	RM_API bool rm_get_parameter_bool(rm_axis_handle handle, uint16_t address);

	RM_API void rm_set_parameter_int32(rm_axis_handle handle, uint16_t address, int32_t value);
	RM_API void rm_set_parameter_float(rm_axis_handle handle, uint16_t address, float value);
	RM_API void rm_set_parameter_bool(rm_axis_handle handle, uint16_t address, bool value);


	// modbus bit access
	RM_API bool rm_read_inputs(rm_axis_handle handle, uint16_t address, uint8_t* data, uint16_t quantity);
	RM_API bool rm_read_coils(rm_axis_handle handle, uint16_t address, uint8_t* data, uint16_t quantity);
	RM_API bool rm_write_coils(rm_axis_handle handle, uint16_t address, uint8_t* data, uint16_t quantity);
	RM_API bool rm_write_coil(rm_axis_handle handle, uint16_t address, uint8_t data);

	// modbus 16 bit access
	RM_API bool rm_read_input_registers(rm_axis_handle handle, uint16_t address, uint16_t* data, uint16_t quantity);
	RM_API bool rm_read_holding_registers(rm_axis_handle handle, uint16_t address, uint16_t* data, uint16_t quantity);
	RM_API bool rm_write_registers(rm_axis_handle handle, uint16_t address, uint16_t* data, uint16_t quantity);
	RM_API bool rm_write_register(rm_axis_handle handle, uint16_t address, uint16_t data);

	// canopen parameters access
	RM_API int32_t rm_read_od_int32(rm_axis_handle handle, uint16_t index, uint8_t sub_index);
	RM_API float rm_read_od_float(rm_axis_handle handle, uint16_t index, uint8_t sub_index);
	RM_API bool rm_read_od_bool(rm_axis_handle handle, uint16_t index, uint8_t sub_index);

	RM_API void rm_write_od_int32(rm_axis_handle handle, uint16_t index, uint8_t sub_index, int32_t value);
	RM_API void rm_write_od_float(rm_axis_handle handle, uint16_t index, uint8_t sub_index, float value);
	RM_API void rm_write_od_bool(rm_axis_handle handle, uint16_t index, uint8_t sub_index, bool value);

	// canopen sdo access
	RM_API int rm_read_od(rm_axis_handle handle, uint16_t od_index, uint8_t od_sub_index, uint8_t data[4]);
	RM_API bool rm_write_od(rm_axis_handle handle, uint16_t od_index, uint8_t od_sub_index, uint8_t* data, int size);

#ifdef __cplusplus
}
#endif

#endif