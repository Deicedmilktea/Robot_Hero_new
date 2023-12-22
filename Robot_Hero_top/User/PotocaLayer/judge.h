#ifndef JUDGE_H
#define JUDGE_H

#include "drv_usart.h"

typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	 
}FrameHeader;   

typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;



typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;




typedef __packed struct
{
     uint8_t winner;
} ext_game_result_t;


typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3;
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3;
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3;
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3;
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
uint8_t lurk_mode;
uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;



typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;



typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;



typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;


typedef __packed struct
{
    uint8_t power_rune_buff;
}ext_buff_t;


typedef __packed struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;


typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;



typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;



enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};


typedef __packed struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;


typedef __packed struct JUDGE_MODULE_DATA
{
     FrameHeader header;

     ext_game_status_t status;
     ext_game_result_t result;
     ext_game_robot_HP_t robot_hp;
     ext_ICRA_buff_debuff_zone_and_lurk_status_t zone;
     ext_event_data_t event_data;
     ext_supply_projectile_action_t supply_status;
     ext_referee_warning_t warning;
     ext_dart_remaining_time_t dart_remaining_time;


     ext_game_robot_status_t    robot_status;
     ext_power_heat_data_t power_heat;
     ext_game_robot_pos_t robot_pos;
     ext_buff_t buff;

     aerial_robot_energy_t aerial_energy;

     ext_robot_hurt_t robot_hurt;
     ext_shoot_data_t shoot_data;
     ext_bullet_remaining_t bullet_remain;

     ext_rfid_status_t rfid_status;
     ext_dart_client_cmd_t dart_client_cmd;
     ext_interact_id_t			ids;	
     uint16_t                 self_client;  


}JUDGE_MODULE_DATA;







extern JUDGE_MODULE_DATA Judge_Hero;






void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length);








#endif