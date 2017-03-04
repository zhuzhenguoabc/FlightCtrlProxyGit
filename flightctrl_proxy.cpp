/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

// system includes
#include <cmath>
#include <cstdbool>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <sys/un.h>
#include <stddef.h>
#include <sys/time.h>

// Waypoint utilities
#include "snav_waypoint_utils.hpp"
// Snapdragon Navigator
#include "snapdragon_navigator.h"

using namespace std;


#define __DEBUG

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif

#define SERVER_UDP_PORT 						14559
#define QCAM_DOMAIN_PORT						16889
#define OTA_UDP_PORT 							14888


#define MAX_BUFF_LEN 							512
#define MIN_GPS_POSITION_NUM 					2
#define MAX_GPS_POSITION_NUM 					10

#define DOMAIN_BUFF_SIZE						16

#define VERSION_NUM 							"1.1.3"
#define STR_SEPARATOR  							","


#define SNAV_CMD_CONROL							"1000"

#define SNAV_CMD_TAKE_OFF						"1001"
#define SNAV_CMD_LAND							"1002"
#define SNAV_CMD_RETURN							"1003"
#define SNAV_CMD_CIRCLE							"1004"
#define SNAV_CMD_TRAIL_NAVIGATION				"1005"
#define SNAV_CMD_GPS_FOLLOW						"1006"
#define SNAV_CMD_PANORAMA						"1007"
#define SNAV_CMD_MAG_CALIBRATE					"1008"
#define SNAV_CMD_HOR_CALIBRATE					"1009"
#define SNAV_CMD_MODIFY_SSID_PWD				"1025"
#define SNAV_CMD_CHECK_WIFI_MODE				"1026"
#define SNAV_CMD_MODIFY_WIFI_5G					"1027"
#define SNAV_CMD_MODIFY_WIFI_2G					"1028"
#define SNAV_CMD_FACE_FOLLOW  					"1100"
#define SNAV_CMD_FACE_FOLLOW_MODE  				"1110"
#define SNAV_CMD_BODY_FOLLOW  					"1101"

#define SNAV_CMD_OPTIC_FLOW_CALIB  				"1201"
#define SNAV_CMD_START_PLAN_TEST_START  		"1202"
#define SNAV_CMD_START_PLAN_TEST_END 			"1203"
#define SNAV_CMD_START_EMERG_STOP  				"1204"

#define SNAV_CMD_RETURN_OPTIC_FLOW_CALIB		"2201"

#define SNAV_INFO_OPTIC_FLOW_CALIB_RESULT		"9201"


#define SNAV_CMD_RETURN_TAKE_OFF				"2001"
#define SNAV_CMD_RETURN_LAND					"2002"
#define SNAV_CMD_RETURN_RETURN					"2003"
#define SNAV_CMD_RETURN_CIRCLE					"2004"
#define SNAV_CMD_RETURN_TRAIL_NAVIGATION		"2005"
#define SNAV_CMD_RETURN_GPS_FOLLOW 				"2006"
#define SNAV_CMD_RETURN_PANORAMA				"2007"
#define SNAV_CMD_RETURN_MAG_CALIBRATE			"2008"
#define SNAV_CMD_RETURN_HOR_CALIBRATE			"2009"
#define SNAV_CMD_RETURN_MODIFY_SSID_PWD 		"2025"
#define SNAV_CMD_RETURN_CHECK_WIFI_MODE			"2026"
#define SNAV_CMD_RETURN_MODIFY_WIFI_5G			"2027"
#define SNAV_CMD_RETURN_MODIFY_WIFI_2G			"2028"
#define SNAV_CMD_RETURN_FACE_FOLLOW 			"2100"
#define SNAV_CMD_RETURN_FACE_FOLLOW_MODE  		"2110"
#define SNAV_CMD_RETURN_BODY_FOLLOW 			"2101"


#define SNAV_TASK_GET_INFO						"8001"
#define SNAV_TASK_GET_SNAV_PROXY_VERSION		"8002"
#define SNAV_TASK_CONFIRM_LAND					"8003"
#define SNAV_TASK_SNAV_UPDATE					"8008"
#define SNAV_TASK_LINARO_UPDATE					"8009"
#define SNAV_TASK_GET_LINARO_VERSION			"8010"
#define SNAV_TASK_GET_SNAV_VERSION				"8011"
#define SNAV_TASK_GET_QCAM_VERSION				"8012"
#define SNAV_TASK_GET_STORAGE					"8013"
#define SNAV_TASK_GET_SD_STATUS					"8014"



#define SNAV_TASK_GET_INFO_RETURN				"9001"
#define SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN	"9002"
#define SNAV_TASK_CONFIRM_LAND_RETURN			"9003"
#define SNAV_TASK_SNAV_UPDATE_RETURN			"9008"
#define SNAV_TASK_LINARO_UPDATE_RETURN			"9009"
#define SNAV_TASK_GET_LINARO_VERSION_RETURN		"9010"
#define SNAV_TASK_GET_SNAV_VERSION_RETURN		"9011"
#define SNAV_TASK_GET_QCAM_VERSION_RETURN		"9012"
#define SNAV_TASK_GET_STORAGE_RETURN			"9013"
#define SNAV_TASK_GET_SD_STATUS_RETURN			"9014"

//send to client
#define SNAV_TASK_SHOW_LAND_CONFIRM				"8004"
#define SNAV_TASK_NEED_PANORAMA_SNAPSHOT		"8005"
#define SNAV_TASK_SHOW_MOTER_ERROR				"8006"
#define SNAV_TASK_SHOW_GPS_TAKE_OFF_ERROR		"8007"
#define SNAV_TASK_SHOW_GPS_RETURN_ERROR_ONE		"8008"
#define SNAV_TASK_SHOW_GPS_RETURN_ERROR_TWO		"8009"


#define SNAV_INFO_OPTIC_FLOW_MISS_SAMPLE		"9100"
#define SNAV_INFO_OVER_SAFE_HEIGHT				"9101"
#define SNAV_INFO_SNAV_NEED_UPDATE				"9102"
#define SNAV_INFO_APP_NEED_UPDATE				"9103"

#define SNAV_INFO_MAG_CALIBRATE_RESULT			"9110"
#define SNAV_INFO_HOR_CALIBRATE_RESULT			"9111"

#define SDCARD_DIR								"/media/sdcard"

//#define LOW_BATTERY_LANDING
#define GPS_MAG_SWITCH
//#define HEIGHT_LIMIT
#define SPEED_LIMIT_FLAG
#define CIRCLE_HEIGHT_LIMIT_FLAG


//cuiyc
#define FOLLOW_IMG_WIDTH				640
#define FOLLOW_IMG_HEIGHT				360
#define FOLLOW_RESERVE_AREA				5


typedef unsigned char byte;

// States used to control mission
enum class MissionState
{
  UNKNOWN,
  ON_GROUND,
  STARTING_PROPS,
  TAKEOFF,
  LOITER,
  LANDING,
  TRAJECTORY_FOLLOW
};

// States of Drone
enum class DroneState
{
  NORMAL,
  MOTOR_ERROR,
  CPU_OVER_HEAT,
  IMU_ERROR,
  BARO_ERROR,
  MAG_ERROR,
  GPS_ERROR,
  SONAR_ERROR,
  OPTIC_FLOW_ERROR,
  EMERGENCY_LANDING_MODE,
  EMERGENCY_KILL_MODE,
  MODE_ERROR,
  UNKNOWN
};

// LED COLOR
enum class LedColor
{
  UNKNOWN,
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
  LED_COLOR_WHITE
};


struct Position
{
  float x;   // m
  float y;   // m
  float z;   // m
  float yaw;
};

struct GpsPosition
{
  int latitude;   // xx.xxxxxx
  int longitude;   //xxx.xxxxx
  int altitude;   //
  float yaw;
};

struct NavigationPosition
{
  float latitude;
  float longitude;
};

//cuiyc face detect
struct body_info
{
  bool have_face;
  bool have_body;
  int  body_flag; //1000 upperbody 1001 fullbody
  bool newP;
  float distance;   // m
  float velocity;   // m/s
  float hegith_calib;   // m for height need to changed to center
  float angle;   // m
};

static LedColor led_color_status = LedColor::UNKNOWN;
static bool bNeedLedColorCtl = false;

struct body_info cur_body;
static bool face_follow_switch = false;
static bool body_follow_switch = false;
static bool face_rotate_switch = false; // false: drone will parallel;   true:drone will first rotate to face then close
static bool body_follow_prallel = true; //prallel fly
const float safe_distance = 1.4f;
const float min_angle_offset = 0.087f;// about 5
const float safe_distanceB = 2.5f; //body distance
static bool adjust_people_height = true;
const float face_height_limit = 2.2f;
const float face_vel_limit = 1.0f;	 //m/sec
const float body_speed_limit = 1.0f; //m/s  10km/h   2.78*2.5 25km/h
static int recieve_pos[25];
static int select_pos[4];
static bool sendpos_back_flag = false;
static bool getpos_body_flag = false;
float speed_last =0;

typedef struct
{
 char  head[4]={'T','R','C','K'};
 unsigned short x = 0;
 unsigned short y = 0;
 unsigned short width = 0;
 unsigned short height = 0;
 unsigned char trackStatus = 0;//0:stopping, 1:tracking, 2:lost
 unsigned char reserved[3];
} S_TRACK_RESULT;

S_TRACK_RESULT track_result;
static int bd_start_counter;
//cuiyc  face detect


static bool send_panorama_flag = false;
static char panorama_buff[DOMAIN_BUFF_SIZE];

static bool send_face_follow_swither_flag = false;
static char face_follow_swither_buff[DOMAIN_BUFF_SIZE];

static bool send_body_follow_swither_flag = false;
static char body_follow_swither_buff[DOMAIN_BUFF_SIZE];



static bool send_ota_linaro_flag = false;
static char ota_linaro_path_buff[DOMAIN_BUFF_SIZE];

static bool send_ota_snav_flag = false;
static char ota_snav_path_buff[DOMAIN_BUFF_SIZE];


typedef struct
{
	int client_sockfd;
}client_arg;

struct prodcons {
    char data[MAX_BUFF_LEN];
	bool bflag;
	bool bSockExit;
	int fd_socket;
    pthread_mutex_t lock;
    pthread_cond_t flag_circle;
	pthread_cond_t flag_handler;
};


struct prodcons pro_udp_receive;

struct timeval timeout_udp = {0,300000};			//300ms


// *****************tool functions start******************************
vector<string> split(const string& s, const string& delim)
{
    vector<string> elems;

    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();

    if (delim_len == 0)
	{
		return elems;
    }

    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }

    return elems;
}

//angle transfrom to radian
float rad(double d)
{
	const float PI = 3.1415926;
	return d*PI/180.0;
}

float CalcDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
	const float EARTH_RADIUS = 6378137;	//m

	double radLat1 = rad(fLati1);
	double radLat2 = rad(fLati2);
	double lati_diff = radLat1 - radLat2;
	double long_diff = rad(fLong1) - rad(fLong2);
	double s = 2*asin(sqrt(pow(sin(lati_diff/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(long_diff/2),2)));
	s = s*EARTH_RADIUS;
	return s;
}

float CalcAxisDistance(float f1,  float f2)
{
	const float EARTH_RADIUS = 6378137.0; //r =6378.137km earth radius
	const float PI = 3.1415926;

	double s =(f1-f2)*PI*EARTH_RADIUS/180.0; //  n*pi*r/180
	return s;
}

void get_ip_address(unsigned long address,char* ip)
{
	sprintf(ip,"%d.%d.%d.%d",(int)(address>>24),(int)((address&0xFF0000)>>24),(int)((address&0xFF00)>>24),(int)(address&0xFF));
}

//high byte first
int bytesToInt(byte src[], int offset)
{
    int value;

    value = (int) ( ((src[offset] & 0xFF)<<24)
            |((src[offset+1] & 0xFF)<<16)
            |((src[offset+2] & 0xFF)<<8)
            |(src[offset+3] & 0xFF));

    return value;
}
// *****************tool functions end******************************



//cyc people detect begin
void* getVideoFaceFollowParam(void*)
{
	//video follow socket begin
	const char* UNIX_DOMAIN ="/tmp/vedio.domain";
	static int recv_php_buf[16];
	static int recv_php_num=0;

	const float window_degree = 101.9747f; //117  degree   101;  83 degree 72
	const float camera_rad =window_degree*M_PI/180; //camera 83.5 degree
	const float camera_vert_rad =(9.0f/16)*window_degree*M_PI/180; //camera 83.5 degree
	const float face_winth =0.150; //camera 83.5 degree
	const float angle_camera =0;//25*M_PI/180;//angle of inclination for y , 0 or 25
	int listen_fd;
	int ret=0;
	int i;

	struct sockaddr_un clt_addr;
	struct sockaddr_un srv_addr;
 	socklen_t len =sizeof(clt_addr);

	DEBUG("getVideoFaceFollowParam start\n");

	while(1)
	{
		//listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
		listen_fd=socket(AF_UNIX,SOCK_DGRAM,0);
		if(listen_fd<0)
		{
			DEBUG("cannot create listening socket");
			continue;
		}
		else
		{
			while(1)
			{
				srv_addr.sun_family=AF_UNIX;
				strncpy(srv_addr.sun_path,UNIX_DOMAIN,sizeof(srv_addr.sun_path)-1);
				unlink(UNIX_DOMAIN);
				ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));

				if(ret==-1)
				{
					DEBUG("cannot bind server socket");
					//close(listen_fd);
					unlink(UNIX_DOMAIN);
					break;
				}

				while(true)
				{
					recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
											0, (struct sockaddr *)&clt_addr, &len);
					DEBUG("\n=====face info=====\n");
					//0,flag 1,center-x 2,center-y,3,face_winth,4,_face_height,5,video_winth,6,video_height
					for(i=0;i<16;i++)
					{
						DEBUG("%d ",recv_php_buf[i]);
					}
					DEBUG("\n");

					if(recv_php_buf[0] == 1)
					{
						// normal  face 18cm && camera 83.5 degree
						float distance,angle,height_cal,vert_offset_angle;
						cur_body.have_face=true;

						if(recv_php_buf[6] == 720) //720p
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/1280));
						}
						else if(recv_php_buf[6] == 1080) //1080p
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/1920));
						}
						else if(recv_php_buf[5] != 0)
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/recv_php_buf[5]));
						}
						//face y center ,height need add to calibration to 1/3 height for image
						vert_offset_angle = angle_camera -
							camera_vert_rad*(recv_php_buf[6]/2 -recv_php_buf[2])/recv_php_buf[6];

						height_cal =distance*tan(angle_camera) - distance * tan(vert_offset_angle);

						if(!adjust_people_height)height_cal = 0;

						//printf("tan(angle_camera):%f ,tan offset vertical:%f \n",tan(angle_camera),
						//	tan(vert_offset_angle));

						printf("angle_camera %f,vert_offset_angle:%f \n",angle_camera,vert_offset_angle);

						//dgree for window 72.7768
						angle = window_degree*((recv_php_buf[5]/2 - recv_php_buf[1])*1.0)/recv_php_buf[5];
						DEBUG("face distance :%f angle:%f \n",distance,angle);

						if((cur_body.angle != angle) || (cur_body.distance != distance))
						{
							cur_body.distance = distance;
							cur_body.angle 	= angle;
							cur_body.newP = true;
							cur_body.hegith_calib = height_cal;
						}
						else
						{
							cur_body.newP = false;
						}

					}
					else
					{
						cur_body.have_face=false;
					}
				}
			}
		}
	}
	//video follow socket end
}

void* getVideoBodyFollowParam(void*)
{
	//video follow socket begin
	const char* UNIX_SERVER ="/tmp/vedio.body.domain";
	const char* UNIX_CLIENT ="/tmp/select.body.domain";
	const float window_degree = 101.9747f;
	float speed_factor = 0.05f;
	struct timeval tv;

	static int recv_php_buf[25];
	static int recv_php_num=0;
	static float velocity_forward =0; //ralative speed to drone

	int listen_fd,feedback_fd;
	int ret=0;
	int i;

	struct sockaddr_un clt_addr,feedback_cliaddr;
	struct sockaddr_un srv_addr;
	socklen_t len=sizeof(clt_addr);

	while(1)
	{
		 //listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
		 listen_fd =socket(AF_UNIX,SOCK_DGRAM,0);
		 feedback_fd =socket(AF_UNIX,SOCK_DGRAM,0);
		 if(listen_fd<0)
		 {
			DEBUG("cannot create listening socket");
			continue;
		 }
		 else
		 {
			  while(1)
			  {
			   srv_addr.sun_family=AF_UNIX;
			   strncpy(srv_addr.sun_path,UNIX_SERVER,sizeof(srv_addr.sun_path)-1);

			   feedback_cliaddr.sun_family=AF_UNIX;
			   strncpy(feedback_cliaddr.sun_path,UNIX_CLIENT,sizeof(feedback_cliaddr.sun_path)-1);

			   unlink(UNIX_SERVER);
			   ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));

			   if(ret==-1)
			   {
				DEBUG("cannot bind server socket");
				//close(listen_fd);
				unlink(UNIX_SERVER);
				break;
			   }

			   while(true)
			   {
			   		memset(recv_php_buf,0,sizeof(recv_php_buf));

					//recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
					//	0, (struct sockaddr *)&clt_addr, &len);
					recv_php_num = recvfrom(listen_fd, (S_TRACK_RESULT *)&track_result, sizeof(track_result),
						0, (struct sockaddr *)&clt_addr, &len);
					DEBUG("\n=====follow info===== %d\n" ,recv_php_num);

					if((strncmp(track_result.head,"TRCK",4) == 0)
						&& body_follow_switch)
					{
						gettimeofday(&tv,NULL);
						printf("follow time:%ld.%ld\n",tv.tv_sec,tv.tv_usec/1000);

						if(track_result.trackStatus == 1) //
						{
						    int center_x, center_y;

							cur_body.have_body=true;
							center_x= track_result.x + track_result.width/2;
							center_y= track_result.y + track_result.height/2;

							DEBUG("follow track_result x:%d y:%d,width:%d,height:%d\n",
								track_result.x,track_result.y,track_result.width,track_result.height);
							DEBUG("follow track_result center_x:%d center_y:%d\n",center_x,center_y);

							velocity_forward = (-1.0f*center_y+FOLLOW_IMG_HEIGHT*0.5f)*speed_factor;
							if(velocity_forward > body_speed_limit) velocity_forward = body_speed_limit;
							if(velocity_forward < -1*body_speed_limit) velocity_forward = -1*body_speed_limit;

							if(abs(center_y - FOLLOW_IMG_HEIGHT*0.5f)< FOLLOW_RESERVE_AREA)
								velocity_forward = 0;

							//velocity_forward = velocity_forward*0.1f + speed_last*0.9f;
							if(fabs(cur_body.velocity - velocity_forward) >0.5f)
								cur_body.velocity += 0.1f*velocity_forward;
							else
								cur_body.velocity = velocity_forward;

							cur_body.angle= (FOLLOW_IMG_WIDTH*0.5 -center_x)*window_degree/FOLLOW_IMG_WIDTH;

						}
						else
						{
							cur_body.have_body=false;
							cur_body.velocity= 0;
						}

						DEBUG("follow velocity:%f angle:%f \n",cur_body.velocity,cur_body.angle);
					}
					else if(recv_php_buf[0] >= 100 && body_follow_switch)
					{
					  //get peoples position for customer select which one follow
						memset(recieve_pos,0,sizeof(recieve_pos));
						int people_count = recv_php_buf[0]%100;

						recieve_pos[0] = people_count;
						for(int i = 0; i< people_count ;i++)
						{
							recieve_pos[4*i+1]=recv_php_buf[4*i+1];
							recieve_pos[4*i+2]=recv_php_buf[4*i+2];
							recieve_pos[4*i+3]=recv_php_buf[4*i+3];
							recieve_pos[4*i+4]=recv_php_buf[4*i+4];
						}
						getpos_body_flag = true;

						//for test
						sendpos_back_flag = true;
					}
					else if(sendpos_back_flag)
					{
						int result ;

						//for test
						memset(select_pos,0,sizeof(select_pos));
					    select_pos[0] = 1111;

						result = sendto(feedback_fd, select_pos, sizeof(select_pos), 0,
						   (struct sockaddr*)&feedback_cliaddr, sizeof(feedback_cliaddr));

						printf("body slecet info send result:%d\n", result);
						sendpos_back_flag = false;
					}
					else //if we have face ,set no body
						cur_body.have_body=false;

				//close(com_fd);
			   }
			  }
		 }
	}
	//video follow socket end
}
//cyc people detect end


/*******************interact with qcamvid************************************/
void* interactWithQcamvid(void*)
{
	DEBUG("interactWithQcamvid start\n");

	int socket_cli;

	struct sockaddr_in address;
	bzero(&address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr("127.0.0.1");
	address.sin_port = htons(QCAM_DOMAIN_PORT);

	socket_cli = socket(AF_INET, SOCK_DGRAM, 0);
	int send_num = 0;

	while (true)
	{
		/*
		if (send_panorama_flag)
		{
			send_num = sendto(socket_cli, panorama_buff, strlen(panorama_buff), 0, (struct sockaddr*)&address, sizeof(address));
			DEBUG("panorama_buff=%s send_num=%d\n", panorama_buff,send_num);
			send_panorama_flag = false;
		}
		*/

		if (send_face_follow_swither_flag)
		{
			send_num = sendto(socket_cli, face_follow_swither_buff, strlen(face_follow_swither_buff), 0, (struct sockaddr*)&address, sizeof(address));
			DEBUG("face_follow_swither_buff=%s send_num=%d\n", face_follow_swither_buff,send_num);
			send_face_follow_swither_flag = false;
		}

		if (send_body_follow_swither_flag)
		{
			send_num = sendto(socket_cli, body_follow_swither_buff, strlen(body_follow_swither_buff), 0, (struct sockaddr*)&address, sizeof(address));
			DEBUG("body_follow_swither_buff=%s send_num=%d\n", body_follow_swither_buff,send_num);
			send_body_follow_swither_flag = false;
		}

		usleep(2000);		//2ms
	}
}

void* interactWithOta(void*)
{
	DEBUG("interactWithOta start\n");

	int socket_cli;

	struct sockaddr_in address;
	bzero(&address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr("127.0.0.1");
	address.sin_port = htons(OTA_UDP_PORT);

	socket_cli = socket(AF_INET, SOCK_DGRAM, 0);
	int send_num = 0;

	while (true)
	{
		if (send_ota_linaro_flag)
		{
			send_num = sendto(socket_cli, ota_linaro_path_buff, strlen(ota_linaro_path_buff), 0, (struct sockaddr*)&address, sizeof(address));
			DEBUG("ota_linaro_path_buff=%s send_num=%d\n", ota_linaro_path_buff,send_num);
			send_ota_linaro_flag = false;
		}

		if (send_ota_snav_flag)
		{
			send_num = sendto(socket_cli, ota_snav_path_buff, strlen(ota_snav_path_buff), 0, (struct sockaddr*)&address, sizeof(address));
			DEBUG("ota_snav_path_buff=%s send_num=%d\n", ota_snav_path_buff,send_num);
			send_ota_snav_flag = false;
		}

		usleep(200000);		//200ms
	}
}




/*******************led control************************************/
void* ledControl(void*)
{
	DEBUG("ledControl thread start\n");

	uint8_t led_colors[3] = {0,0,0};  //R, G, B
	int32_t timeout = 1000000; // 1S, timeout for flight controller to take over LED control after API commands stop

	int continue_red_count = 0;
	int continue_green_count = 0;
	int continue_blue_count = 0;
	int continue_white_count = 0;

	while (true)
	{
		switch (led_color_status)
		{
			case LedColor::LED_COLOR_RED:
			{
				continue_red_count++;
				continue_green_count = 0;
				continue_blue_count = 0;
				continue_white_count = 0;

				led_colors[0] = 0;	//255;
				led_colors[1] = 255;	//0;
				led_colors[2] = 0;

				break;
			}
			case LedColor::LED_COLOR_GREEN:
			{
				continue_red_count = 0;
				continue_green_count++;
				continue_blue_count = 0;
				continue_white_count = 0;

				led_colors[0] = 255;	//0;
				led_colors[1] = 0;	//255;
				led_colors[2] = 0;

				break;
			}
			case LedColor::LED_COLOR_BLUE:
			{
				continue_red_count = 0;
				continue_green_count = 0;
				continue_blue_count++;
				continue_white_count = 0;

				led_colors[0] = 0;
				led_colors[1] = 0;
				led_colors[2] = 255;

				break;
			}
			case LedColor::LED_COLOR_WHITE:
			{
				continue_red_count = 0;
				continue_green_count = 0;
				continue_blue_count = 0;
				continue_white_count++;

				led_colors[0] = 255;
				led_colors[1] = 255;
				led_colors[2] = 255;

				break;
			}
			default:
			{
				break;
			}
		}

		// for red/blue color twinkle with black color
		if (((continue_red_count%100 >= 50) && (continue_red_count%100 < 100))
			|| ((continue_blue_count%100 >= 50) && (continue_blue_count%100 < 100)))
		{
			led_colors[0] = 0;
			led_colors[1] = 0;
			led_colors[2] = 0;
		}

		DEBUG("sn_set_led_colors bNeedLedColorCtl:%d,led_color_status:%d, Color:%d,%d,%d\n",
				bNeedLedColorCtl, led_color_status, led_colors[0], led_colors[1], led_colors[2]);

		if (bNeedLedColorCtl)
		{
			int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout);

			if (ret != 0)
			{
				DEBUG("sn_set_led_colors returned %d\n",ret);
			}
		}

		usleep(10000);  // note that commands should only be sent as often as needed (minimize message traffic)
	}
}

int main(int argc, char* argv[])
{
	int  udpOverTimeCount = 0;
	bool bHaveUdpClient = false;
	char current_udp_client_addr[MAX_BUFF_LEN];
	bool bReceivedUdpMsg = false;

	char result_to_client[MAX_BUFF_LEN];

	float speed_coefficient = 1.0f;
	float height_limit = 20.0f; // m

	const float fMaxVel = 2;	// m/s

	const float fTakeOffHeight = 1.2; // m
	const float fTrarilHeight = 1.8;	// m
	const float kTakeoffSpeed = 0.9;	// m/s	*2/3 = 0.6 cmd2

	const float kLandingSpeed = -0.75;	// m/s	*2/3 = 0.5 cmd2
	bool confirm_land = false;

	float distance_to_home = 0;
	float vel_target = 0.75;	 //m/sec

	// Circle mission
	int circle_cam_point_direct = 1;	// point to inside by default 1(inside) -1(outside)
	bool circle_mission = false;
	bool calcCirclePoint = false;
	vector<Position> circle_positions;
	float radius = 2.5f;				// m
	int point_count = 36;
	float angle_per = 2*M_PI/point_count;
	int clockwise = 1;					// anticlockwise = -1

	// Panorama mission
	bool panorama_mission = false;
	static bool calcPanoramaPoint = false;
	vector<Position> panorama_positions;

	// Trail follow with gps array
	bool trail_navigation_mission = false;

	// Return home mission
	bool return_mission = false;
	bool fly_home = false;
	float gohome_x_vel_des = 0;
	float gohome_y_vel_des = 0;
	float gohome_z_vel_des = 0;
	float gohome_yaw_vel_des = 0;
	uint8_t wp_goal_ret = 0b11111111;
	uint8_t wp_goal_mask = 0b11111111;
	float yaw_target_home, distance_home_squared;
	float yaw_gps_target_home, distance_gps_home_squared;
	float distance_home_squared_threshold = 1;
	FlatVars output_vel;
	bool take_off_with_gps_valid = false;

	// People detect cuiyc begin
	bool face_mission=false;
	bool body_mission=false;

	// Position at startup
	static float x_est_startup = 0;
	static float y_est_startup = 0;
	static float z_est_startup = 0;
	static float yaw_est_startup = 0;

	static float x_est_gps_startup = 0;
	static float y_est_gps_startup = 0;
	static float z_est_gps_startup = 0;
	static float yaw_est_gps_startup = 0;

	// Mission State Machine
	static size_t current_position = 0;

	// Time to loiter
	const float kLoiterTime = 3;	// second

	// Gps params
	GpsPosition posLast;
	GpsPosition posGpsCurrent;
	GpsPosition posGpsDestination;
	float destyaw = 0;
	float speed = 0;
	float distance_to_dest = 0;
	SnRcCommandType curSendMode = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

	// Gps postion fly
	vector<GpsPosition> gps_positions;
	vector<NavigationPosition> trail_navigation_positions;

	char drone_state_error[MAX_BUFF_LEN];

	DroneState drone_state = DroneState::NORMAL;
	MissionState state = MissionState::ON_GROUND;	// UNKNOWN;
	int loop_counter = 0;

	static bool landing_near_ground = false;		// for ignore the sonar wrong data below 0.3m

	// Add by wlh
	float estimated_xy_sqrt = 0;
	float desired_xy_sqrt = 0;
	int baro_tenflag = 0;
	float baro_ten_heigh = 0;
	int reverse_ctrl_count = 0;
	float old_cmd0 = 0;
	float old_cmd1 = 0;
	float old_cmd00 = 0;
	float old_cmd11 = 0;
	int sample_size_count = 0;
	int sample_size_missing_count = 0;
	int reverse_ctrl_step = 0;
	int linacc_sample_count = 0;
	float linacc_total_value = 0;
	float imu_lin_acc = 0;
	float old_pitch = 0;
	float old_roll = 0;
	float last_pitch = 0;
	float last_roll = 0;
	const float stop_control_num = 199;
	// Add end

	// For revise height with barometer and sonar
	int gps_mag_flag = 1;

	int use_revise_height = 1;

	int revise_rule = 1;
	int revise_type_sample_size = 1;
	int revise_type_desire = 1;
	int revise_type_linacc = 1;

	int revise_type = 1;
	int revise_full_flag = 1;
	int revise_ctrl_flag = 1;

	float revise_height = 0;
	float baro_groud = 0;

	// Confirm logfile name start
	FILE *fp_count_read, *fp_count_write;
	int log_count=0;

	if ((fp_count_read = fopen("/home/linaro/flightctrl_proxy_count", "a+")) != NULL)
	{
		if (fscanf(fp_count_read, "%d", &log_count) != EOF)
		{
			DEBUG("flightctrl_proxy_count=%d\n", log_count);

			if (log_count >= 0)
			{
				log_count++;
			}

			if (log_count >= 8000)
			{
				log_count = 0;
			}
		}
		else
		{
			DEBUG("flightctrl_proxy_count first time\n");
			log_count = 1;
		}
		fclose(fp_count_read);

		char str[16];
		if ((fp_count_write = fopen("/home/linaro/flightctrl_proxy_count", "w+")) != NULL)
		{
			sprintf(str, "%d", log_count);
			fwrite(&str, strlen(str), 1, fp_count_write);
			fclose(fp_count_write);
		}
	}

	// Only keep the last 5 log files
	system("find /home/linaro/ -type f -name 'log_flightctrl*'|xargs -r ls -l|head -n -5|awk '{print $9}'| xargs rm -rf");
	char log_filename[256];
 	sprintf(log_filename, "/home/linaro/log_flightctrl_%04d", log_count);
	DEBUG("log_filename=%s\n", log_filename);
	// Confirm logfile name end

	freopen(log_filename, "a", stdout);
	setbuf(stdout, NULL);		//needn't cache and fflush, output immediately
	freopen(log_filename, "a", stderr);
	setbuf(stderr, NULL);		//needn't cache and fflush, output immediately

	// Create the face_body_follow process of snav
	bool face_body_follow_flag = false;
	while (!face_body_follow_flag)
	{
		pthread_t face_follow_thread, body_follow_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if (result != 0)
	    {
	        perror("face_body_follow Attribute init failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
	    if (result != 0)
	    {
	        perror("face_body_follow Setting detached attribute failed");
	        pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		result = pthread_create(&face_follow_thread, &thread_attr, getVideoFaceFollowParam, NULL);
		if (result != 0)
	    {
	    	perror("face_follow_thread create failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		result = pthread_create(&body_follow_thread, &thread_attr, getVideoBodyFollowParam, NULL);
	    if (result != 0)
	    {
	    	pthread_cancel(face_follow_thread);

	    	perror("body_follow_thread create failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		face_body_follow_flag = true;
		pthread_attr_destroy(&thread_attr);
	}

	// Create the interact with qcamvid process
	bool interact_with_qcamvid_flag = false;
	while (!interact_with_qcamvid_flag)
	{
		pthread_t interact_with_qcamvid_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if (result != 0)
	    {
	        perror("interact_with_qcamvid_thread Attribute init failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
	    if (result != 0)
	    {
	        perror("interact_with_qcamvid_thread Setting detached attribute failed");
	        pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		result = pthread_create(&interact_with_qcamvid_thread, &thread_attr, interactWithQcamvid, NULL);
	    if (result != 0)
	    {
	    	perror("interact_with_qcamvid_thread create failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		interact_with_qcamvid_flag = true;
		pthread_attr_destroy(&thread_attr);
	}

	// Create the interact with ota process
	bool interact_with_ota_flag = false;
	while (!interact_with_ota_flag)
	{
		pthread_t interact_with_ota_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if (result != 0)
	    {
	        perror("interact_with_ota_thread Attribute init failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
	    if (result != 0)
	    {
	        perror("interact_with_ota_thread Setting detached attribute failed");
	        pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		result = pthread_create(&interact_with_ota_thread, &thread_attr, interactWithOta, NULL);
	    if (result != 0)
	    {
	    	perror("interact_with_ota_thread create failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		interact_with_ota_flag = true;
		pthread_attr_destroy(&thread_attr);
	}

	// Create the led control process
	bool led_ctl_flag = false;
	while (!led_ctl_flag)
	{
		pthread_t led_ctl_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if (result != 0)
	    {
	        perror("led_ctl_thread Attribute init failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
	    if (result != 0)
	    {
	        perror("led_ctl_thread Setting detached attribute failed");
	        pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		result = pthread_create(&led_ctl_thread, &thread_attr, ledControl, NULL);
	    if (result != 0)
	    {
	    	perror("Thread led_ctl_thread create failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		led_ctl_flag = true;
		pthread_attr_destroy(&thread_attr);
	}

	SnavCachedData* snav_data = NULL;
	if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
	{
		DEBUG("\nFailed to get flight data pointer!\n");
		return -1;
	}

	// Udp
	int server_udp_sockfd;
	int server_udp_len;
    struct sockaddr_in server_udp_address;

    server_udp_address.sin_family		= AF_INET;
    server_udp_address.sin_addr.s_addr	= htonl(INADDR_ANY);
    server_udp_address.sin_port			= htons(SERVER_UDP_PORT);
    server_udp_len						= sizeof(server_udp_address);

	server_udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    int bind_result = bind(server_udp_sockfd, (struct sockaddr*)&server_udp_address, server_udp_len);

	// 300MS avoid of udp missing data
	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_udp, sizeof(struct timeval));
	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_udp, sizeof(struct timeval));


	// The Udp Circle
	while (true)
	{
		int length = 0;
		struct sockaddr_in remote_addr;
		int sin_size = sizeof(struct sockaddr_in);
		char udp_buff_data[MAX_BUFF_LEN];

		//receive the udp data
		length = recvfrom(server_udp_sockfd, udp_buff_data, MAX_BUFF_LEN-1, 0,
							(struct sockaddr *)&remote_addr, (socklen_t*)&sin_size);

		if (length > 0)
		{
			struct timeval time_val;
			gettimeofday(&time_val, NULL);
			double time_now = time_val.tv_sec + time_val.tv_usec * 1e-6;

			udp_buff_data[length]='\0';
			DEBUG("\nudp recvfrom received data from %s, %d:\n", inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));
			DEBUG("udp recvfrom get data udp_buff_data=%s, time_now=%lf\n", udp_buff_data, time_now);

			bReceivedUdpMsg = true;

			//********************ignore the other udp connect******************************
			if (bHaveUdpClient)
			{
				DEBUG("udp recvfrom current client addr=%s\n", current_udp_client_addr);

				//ignore the other udp client when have one
				if (strcmp(current_udp_client_addr, inet_ntoa(remote_addr.sin_addr)) != 0)
				{
					DEBUG("udp recvfrom ignore the other udp addr=%s\n", inet_ntoa(remote_addr.sin_addr));
					continue;
				}
				else
				{
					udpOverTimeCount = 0;
				}
			}
			//lock the udp client ip when the first udp connect
			else
			{
				bHaveUdpClient = true;
				udpOverTimeCount = 0;

				memset(current_udp_client_addr, 0, MAX_BUFF_LEN);
				memcpy(current_udp_client_addr, inet_ntoa(remote_addr.sin_addr), MAX_BUFF_LEN);
				DEBUG("udp recvfrom the first client addr=%s\n", inet_ntoa(remote_addr.sin_addr));
			}
		}
		else
		{
			struct timeval time_val;
			gettimeofday(&time_val, NULL);
			double time_now = time_val.tv_sec + time_val.tv_usec*1e-6;

			DEBUG("\nudp overtime or issue: length=%d, errno=%d, time_now=%lf\n", length, errno, time_now);

			bReceivedUdpMsg = false;

			//************************************************************
			udpOverTimeCount++;

			DEBUG("udpOverTimeCount=%d\n", udpOverTimeCount);

			if (udpOverTimeCount >= 10)		//10*300ms
			{
				DEBUG("the first udp client overtime and discard\n");
				bHaveUdpClient = false;
				udpOverTimeCount = 0;
			}
		}

		// Always need to call this
		if (sn_update_data() != 0)
		{
			DEBUG("sn_update_data failed\n");
		}
		else
		{
			// Get the current mode
			SnMode mode;
			mode = (SnMode)snav_data->general_status.current_mode;

			// Get the current state of the propellers
			SnPropsState props_state;
			props_state = (SnPropsState) snav_data->general_status.props_state;

			// Get the source of the RC input (spektrum vs API) here
      		SnRcCommandSource rc_cmd_source = (SnRcCommandSource)(snav_data->rc_active.source);

			// Get the gps status
			int gps_enabled;
    		sn_is_gps_enabled(&gps_enabled);

			if ((int)snav_data->gps_0_raw.fix_type == 3)
			{
				posGpsCurrent.latitude	= (int)snav_data->gps_0_raw.latitude;
				posGpsCurrent.longitude	= (int)snav_data->gps_0_raw.longitude;
				posGpsCurrent.altitude	= (int)snav_data->gps_0_raw.altitude;

				//posGpsCurrent.yaw		= (float)snav_data->high_level_control_data.yaw_estimated;
				posGpsCurrent.yaw		= (float)snav_data->gps_pos_vel.yaw_estimated;
			}

			// Get the current battery voltage
			float voltage;
			voltage = (float)snav_data->general_status.voltage;

			// Get the current sample size
			int sample_size = 0;
			sample_size = snav_data->optic_flow_0_raw.sample_size;

			// Get the sensors status
			SnDataStatus imu_status = (SnDataStatus)snav_data->data_status.imu_0_status;
			SnDataStatus baro_status = (SnDataStatus)snav_data->data_status.baro_0_status;
			SnDataStatus mag_status = (SnDataStatus)snav_data->data_status.mag_0_status;
			SnDataStatus gps_status = (SnDataStatus)snav_data->data_status.gps_0_status;
			SnDataStatus sonar_status = (SnDataStatus)snav_data->data_status.sonar_0_status;
			SnDataStatus optic_flow_status = (SnDataStatus)snav_data->data_status.optic_flow_0_status;

			// Get the "on ground" flag
			int on_ground_flag;
			on_ground_flag = (int)snav_data->general_status.on_ground;

			// Add by wlh
			// Revise the z position start
			if (on_ground_flag == 1)
			{
				baro_groud = snav_data->barometer_0_raw.pressure;
			}

			float baro_x = snav_data->barometer_0_raw.pressure - baro_groud;
			float sonar_y = -0.000276* baro_x*baro_x  - 0.086519*baro_x - 0.213962;

			if (on_ground_flag == 1 || snav_data->sonar_0_raw.range  < 0.29)
			{
				sonar_y = 0;
			}

			if (on_ground_flag == 0 && snav_data->sonar_0_raw.range > 0.30 && baro_groud != 0)
			{
				if (snav_data->sonar_0_raw.range < 2.3)
				{
					if (fabs(revise_height - snav_data->sonar_0_raw.range) < 0.5)
					{
						if (fabs(sonar_y - snav_data->sonar_0_raw.range) < 1.2)
						{
							// Get the baro data
							revise_height = snav_data->sonar_0_raw.range;
						}
					}
				}
				else
				{
					if(baro_tenflag == 0)
					{
						baro_ten_heigh = 0;
						baro_tenflag++;
					}
					else
					{
						baro_ten_heigh += 0.000007*baro_x*baro_x  - 0.081320*baro_x - 0.027264;
						baro_tenflag++;
						if(baro_tenflag > 10)
						{
							baro_tenflag = 0;
							revise_height = baro_ten_heigh/10;
						}
					}

					if(revise_height < 0)
					{
						revise_height = 0;
					}
				}
			}

			DEBUG("revise_height=%f, baro_x=%f\n",revise_height, baro_x);
			// revise the z position end
			// Add end

			float x_vel, y_vel;
#ifdef	GPS_MAG_SWITCH
			/*if (gps_enabled && (gps_status == SN_DATA_VALID) && (gps_mag_flag == 1))*/
			if (gps_enabled && (gps_status == SN_DATA_VALID))
			{
				x_vel = (float)snav_data->gps_pos_vel.velocity_estimated[0];
				y_vel = (float)snav_data->gps_pos_vel.velocity_estimated[1];
			}
			else
#endif
			{
				x_vel = (float)snav_data->optic_flow_pos_vel.velocity_estimated[0];
				y_vel = (float)snav_data->optic_flow_pos_vel.velocity_estimated[1];
			}

			// Get the current estimated position and yaw
			float x_est, y_est, z_est, yaw_est;
			float x_est_gps, y_est_gps, z_est_gps, yaw_est_gps;
			float x_des_gps, y_des_gps, z_des_gps, yaw_des_gps;
#ifdef	GPS_MAG_SWITCH
			/*if (gps_enabled && (gps_status == SN_DATA_VALID) && (gps_mag_flag == 1))*/
			if (gps_enabled)
			{
				/*
				x_est_gps = (float)snav_data->high_level_control_data.position_estimated[0];
				y_est_gps = (float)snav_data->high_level_control_data.position_estimated[1];
				z_est_gps = (float)snav_data->high_level_control_data.position_estimated[2];
				yaw_est_gps = (float)snav_data->high_level_control_data.yaw_estimated;
				*/
				x_est_gps = (float)snav_data->gps_pos_vel.position_estimated[0];
				y_est_gps = (float)snav_data->gps_pos_vel.position_estimated[1];
				z_est_gps = (float)snav_data->gps_pos_vel.position_estimated[2];
				yaw_est_gps = (float)snav_data->gps_pos_vel.yaw_estimated;

				x_des_gps = (float)snav_data->gps_pos_vel.position_desired[0];
				y_des_gps = (float)snav_data->gps_pos_vel.position_desired[1];
				z_des_gps = (float)snav_data->gps_pos_vel.position_desired[2];
				yaw_des_gps = (float)snav_data->gps_pos_vel.yaw_desired;
			}
			else
#endif
			{
				x_est_gps = 0;
				y_est_gps = 0;
				z_est_gps = 0;
				yaw_est_gps = 0;

				x_des_gps = 0;
				y_des_gps = 0;
				z_des_gps = 0;
				yaw_des_gps = 0;
			}

			x_est = (float)snav_data->optic_flow_pos_vel.position_estimated[0];
			y_est = (float)snav_data->optic_flow_pos_vel.position_estimated[1];
			z_est = (float)snav_data->optic_flow_pos_vel.position_estimated[2];
			yaw_est = (float)snav_data->optic_flow_pos_vel.yaw_estimated;


			// Get the current desired position and yaw
			// NOTE this is the setpoint that will be controlled by sending
			// velocity commands
			float x_des, y_des, z_des, yaw_des;
			x_des = (float)snav_data->optic_flow_pos_vel.position_desired[0];
			y_des = (float)snav_data->optic_flow_pos_vel.position_desired[1];
			z_des = (float)snav_data->optic_flow_pos_vel.position_desired[2];
			yaw_des = (float)snav_data->optic_flow_pos_vel.yaw_desired;

			struct timeval tv_for_des;
			gettimeofday(&tv_for_des, NULL);
			double t_des_now = tv_for_des.tv_sec + tv_for_des.tv_usec * 1e-6;

			DEBUG("[%d] t_des_now:x_est,x_des,y_est,y_des=%lf,%f,%f,%f,%f\n",
							loop_counter, t_des_now, x_est, x_des, y_est, y_des);

			// Revise print message and whether to use it
			DEBUG("use_revise_height=%d, revise_height=%f, baro_x=%f, z_est-z_est_startup=%f\n"
						,use_revise_height, revise_height, baro_x, (z_est-z_est_startup));

			if (use_revise_height != 1)
			{
				revise_height = (z_est-z_est_startup);
			}

			//Add by wlh
			const float cmd_offset = 10;
			float speed_xy = fabs(sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0] + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]));

			//Samplesize error handle start
			if (revise_type_sample_size == 1)
			{
				if (sample_size >= 0)
	      		{
					if ((revise_height > 0.3) && (reverse_ctrl_count == 0) && (on_ground_flag == 0))
					{
						sample_size_count++;

						if (sample_size_count > 0)
						{
							if (sample_size < 5)
							{
								sample_size_missing_count++;
							}
						}

						if (sample_size_count > 20)
						{
							if (sample_size_missing_count > 9)
							{
								if (reverse_ctrl_step == 0)
								{
									reverse_ctrl_count++;
									if (fabs(old_cmd0) < 0.3 && fabs(old_cmd1) < 0.3)
									{
										old_cmd0 = snav_data->attitude_estimate.pitch*6;
										old_cmd1 = -snav_data->attitude_estimate.roll*6;
										DEBUG("[optic zero 1]---old_cmd0[%f]old_cmd1[%f]\n", old_cmd0, old_cmd1);
									}
									else
									{
										DEBUG("[optic zero 2]---old_cmd0[%f]old_cmd1[%f]\n", old_cmd0, old_cmd1);
									}

									old_pitch = snav_data->attitude_estimate.pitch;
									old_roll  = -snav_data->attitude_estimate.roll;
								}
							}

							sample_size_count = 0;
							sample_size_missing_count = 0;
						}

						if(speed_xy > 3)
						{
							if(reverse_ctrl_step == 0)
							{
								reverse_ctrl_count++;
								DEBUG("[speed]------speed_xy[%f]\n", speed_xy);
								old_pitch = snav_data->attitude_estimate.pitch;
								old_roll  = -snav_data->attitude_estimate.roll;
							}

						}

					}
	      		}
			}
			//Samplesize error handle end

			/*
			if (revise_type_desire == 1)
			{
				//des diff with est is abnormal handle start
				estimated_xy_sqrt = sqrt(snav_data->high_level_control_data.position_estimated[0]*snav_data->high_level_control_data.position_estimated[0] + snav_data->high_level_control_data.position_estimated[1]*snav_data->high_level_control_data.position_estimated[1]);
				desired_xy_sqrt   = sqrt(snav_data->high_level_control_data.position_desired[0]* snav_data->high_level_control_data.position_desired[0] + snav_data->high_level_control_data.position_desired[1]*snav_data->high_level_control_data.position_desired[1]);

				if(fabs(estimated_xy_sqrt)> 1
					&& fabs(desired_xy_sqrt) > 1
					&&(fabs(estimated_xy_sqrt - desired_xy_sqrt) > revise_height/2)
					&& (fabs(estimated_xy_sqrt - desired_xy_sqrt) > 0.5)
					&& (revise_height > 0.3)
					&& (snav_data->general_status.on_ground == 0)
					&& (reverse_ctrl_count == 0))
				{
					if(reverse_ctrl_step == 0)
					{
						reverse_ctrl_count++;
						DEBUG("[desired]--estimated_xy_sqrt[%f] desired_xy_sqrt[%f]--sample_size[%d]-air_heigh/2=[%f]---%d \n",estimated_xy_sqrt,desired_xy_sqrt,sample_size,revise_height/2,reverse_ctrl_count );
						old_pitch = snav_data->attitude_estimate.pitch;
						old_roll  = -snav_data->attitude_estimate.roll;
					}
				}
			}
			*/
			//des diff with est is abnormal handle start

			//lin_acc overlimit handle start
			if (revise_type_linacc == 1)
			{
				if (imu_status == SN_DATA_VALID)
				{
					imu_lin_acc = sqrt(snav_data->imu_0_compensated.lin_acc[0]*snav_data->imu_0_compensated.lin_acc[0]
										+ snav_data->imu_0_compensated.lin_acc[1]*snav_data->imu_0_compensated.lin_acc[1]);

					if (linacc_sample_count < 30)
					{
						linacc_sample_count++;
						linacc_total_value += fabs(imu_lin_acc);

					}
					else
					{
						if((linacc_total_value/30)/1.44 > 2
							&& ((revise_height> 0.3)
								  && snav_data->general_status.on_ground == 0
								  && reverse_ctrl_count == 0))
						{
							reverse_ctrl_count++;
							DEBUG("[imu_lin_acc]--linacc_total_value/30[%f]\n", linacc_total_value/30);
							old_pitch = snav_data->attitude_estimate.pitch;
							old_roll  = -snav_data->attitude_estimate.roll;
						}
						linacc_sample_count = 0;
						linacc_total_value = 0;
					}
				}
				else
				{
					imu_lin_acc = -108;
				}
			}
			//lin_acc overlimit handle end
			//Add end

			// Prepare current information to send to Client
			drone_state = DroneState::NORMAL;				/*Reinit the drone status every cicle*/

			memset(drone_state_error, 0, MAX_BUFF_LEN);		/*Reinit the drone error state*/
			sprintf(drone_state_error, "%s", "drone_state_error:0");

			if (props_state == SN_PROPS_STATE_UNKNOWN)
			{
				drone_state = DroneState::MOTOR_ERROR;
				strcat(drone_state_error, ":1");
			}

			CpuStats cpu_status = snav_data->cpu_stats;		/*Get current cpu temp*/
			for (int j = 0; j < 10; j++)
			{
				if (cpu_status.temp[j] >= 80)
				{
					drone_state = DroneState::CPU_OVER_HEAT;
					strcat(drone_state_error, ":2");
					break;
				}
			}

			if (imu_status != SN_DATA_VALID)
			{
				drone_state = DroneState::IMU_ERROR;
				strcat(drone_state_error, ":3");
			}

			if (baro_status != SN_DATA_VALID)
			{
				drone_state = DroneState::BARO_ERROR;
				strcat(drone_state_error, ":4");
			}

#ifdef	GPS_MAG_SWITCH
			if (gps_mag_flag == 1)
			{
				/*
				if ((mag_status != SN_DATA_VALID) && (mag_status != SN_DATA_WARNING))
				{
					drone_state = DroneState::MAG_ERROR;
					strcat(drone_state_error, ":5");
				}
				*/

				//if ((gps_status != SN_DATA_VALID) && (gps_status != SN_DATA_NOT_INITIALIZED) && (gps_status != SN_DATA_NO_LOCK))
				if (gps_status != SN_DATA_VALID)
				{
					drone_state = DroneState::GPS_ERROR;
					strcat(drone_state_error, ":6");
				}
			}
#endif

			if (sonar_status != SN_DATA_VALID)
			{
				drone_state = DroneState::SONAR_ERROR;
				strcat(drone_state_error, ":7");
			}

			if (optic_flow_status != SN_DATA_VALID)
			{
				drone_state = DroneState::OPTIC_FLOW_ERROR;
				strcat(drone_state_error, ":8");
			}

			if (mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
			{
				if (mode == SN_EMERGENCY_LANDING_MODE)
				{
					drone_state = DroneState::EMERGENCY_LANDING_MODE;
					strcat(drone_state_error, ":9");
				}
				else if (mode == SN_EMERGENCY_KILL_MODE)
				{
					drone_state = DroneState::EMERGENCY_KILL_MODE;
					strcat(drone_state_error, ":10");
				}
				else
				{
					drone_state = DroneState::MODE_ERROR;
					strcat(drone_state_error, ":11");
				}
			}
			DEBUG("drone_state_error=%s\n", drone_state_error);

			// Led light control
			if ((mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
				|| (drone_state == DroneState::IMU_ERROR)
				|| (drone_state == DroneState::OPTIC_FLOW_ERROR)
				|| (drone_state == DroneState::SONAR_ERROR)
				|| (drone_state == DroneState::BARO_ERROR)
				|| (drone_state == DroneState::GPS_ERROR)
				|| (drone_state == DroneState::MAG_ERROR)
				|| (drone_state == DroneState::MOTOR_ERROR)
				|| (drone_state == DroneState::CPU_OVER_HEAT))
			{
				bNeedLedColorCtl = true;
				led_color_status = LedColor::LED_COLOR_RED;
			}
#if 0
			else if (((on_ground_flag == 1) && (voltage < 7.40f))
					|| (((/*z_est - z_est_startup*/revise_height) <= 5.0f) && (voltage < 6.90f))
					|| (((/*z_est - z_est_startup*/revise_height) > 5.0f && (/*z_est - z_est_startup*/revise_height) <= 10.0f) && (voltage < 7.0f))
					|| (((/*z_est - z_est_startup*/revise_height) > 10.0f && (/*z_est - z_est_startup*/revise_height) <= 15.0f) && (voltage < 7.1f))
					|| (((/*z_est - z_est_startup*/revise_height) > 15.0f && (/*z_est - z_est_startup*/revise_height) <= 20.0f) && (voltage < 7.2f))
					|| (((/*z_est - z_est_startup*/revise_height) > 20.0f) && (voltage < 7.3f)))
#else
			else if (voltage < 6.75f)
#endif
			{
				bNeedLedColorCtl = true;
				led_color_status = LedColor::LED_COLOR_BLUE;
			}
			else if ((cur_body.have_face && face_follow_switch)
					|| (cur_body.have_body && body_follow_switch))
			{
				bNeedLedColorCtl = true;
				led_color_status = LedColor::LED_COLOR_GREEN;
			}
			else if (bHaveUdpClient)
			{
				bNeedLedColorCtl = true;
				if (on_ground_flag == 1)
				{
					led_color_status = LedColor::LED_COLOR_GREEN;
				}
				else
				{
					led_color_status = LedColor::LED_COLOR_WHITE;
				}
			}
			else
			{
				bNeedLedColorCtl = false;
			}
			DEBUG("bNeedLedColorCtl:%d,led_color_status:%d\n", bNeedLedColorCtl, led_color_status);

#ifdef	LOW_BATTERY_LANDING
			// Low battery force landing
			if ((props_state == SN_PROPS_STATE_SPINNING)
				&& (voltage < 6.75f)
				&& (state == MissionState::LOITER
					|| state == MissionState::TRAJECTORY_FOLLOW))
			{
				confirm_land = true;
				state = MissionState::LANDING;
			}
#endif

			if (state != MissionState::LANDING)
			{
				landing_near_ground = false;
			}

			string recv_udp_cmd;
			vector<string> udp_msg_array;

			if (bReceivedUdpMsg)
			{
				recv_udp_cmd = udp_buff_data;
				udp_msg_array = split(recv_udp_cmd, STR_SEPARATOR);
				DEBUG("udp control operation:%s\n", udp_buff_data);

				if ((udp_msg_array.size() >= 6) && (udp_msg_array[0].compare(SNAV_CMD_CONROL) == 0))
				{
					if (!((udp_msg_array[1].compare("0")==0)
						&& (udp_msg_array[2].compare("0")==0)
						&& (udp_msg_array[3].compare("0")==0)
						&& (udp_msg_array[4].compare("500")==0)
						&& (udp_msg_array[5].compare("0")==0)))
					{
						// For snav control
						int rolli = -1;
			      		int pitchi = -1;
			      		int yawi = -1;
			      		int thrusti = -1;
			      		int buttons = -1;

						bool land_cmd_flag = false;

						const float kMin = -1;
						const float kMax = 1;

						float cmd0 = 0;
						float cmd1 = 0;
						float cmd2 = 0;
						float cmd3 = 0;

						// Use type to control how the commands get interpreted.
			      		SnRcCommandType type = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

						// Reset the state when get udp control
						current_position =0;

						circle_mission = false;
						calcCirclePoint = false;

						panorama_mission = false;
						calcPanoramaPoint = false;

						trail_navigation_mission = false;

						return_mission = false;

						face_mission = false;
						body_mission = false;

						face_follow_switch = false;
						body_follow_switch = false;

						rolli = atoi(udp_msg_array[1].c_str());
						pitchi = atoi(udp_msg_array[2].c_str());
						yawi = atoi(udp_msg_array[3].c_str());
						thrusti = atoi(udp_msg_array[4].c_str());
						buttons = atoi(udp_msg_array[5].c_str());

						DEBUG("SNAV_CMD_CONROL rolli, pitchi, yawi, thrusti, buttons: %d, %d, %d, %d, %d\n",
											   rolli, pitchi, yawi, thrusti, buttons);

						if (props_state == SN_PROPS_STATE_SPINNING)
						{
							if (buttons == 1)	//landing
							{
								land_cmd_flag = true;
							}
							else
							{
								land_cmd_flag = false;
							}

							cmd0 = -((float)(pitchi+441)*(kMax-kMin)/882.+ kMin);
							cmd1 = -((float)(rolli+441)*(kMax-kMin)/882.+ kMin);
							cmd2 = (float)(thrusti)*(kMax-kMin)/1000.+ kMin;
							cmd3 = -((float)(yawi+250)*(kMax-kMin)/500.+ kMin);

							if (land_cmd_flag)
							{
								// If user touches roll/pitch stick, stop landing
								if (fabs(cmd0) > 1e-4 || fabs(cmd1) > 1e-4)
								{
									land_cmd_flag = false;
								}
								else
								{
									cmd0 = 0;
									cmd1 = 0;
									cmd2 = -0.5;	//-0.75;
									cmd3 = 0;
								}
							}

							DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
							DEBUG("UDP SNAV_SEND_CMD cmd0, cmd1, cmd2, cmd3: %f, %f, %f, %f\n", cmd0, cmd1, cmd2, cmd3);
							DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");

							//limit the speed when higher than 5m
							DEBUG("xyz_info:%f:%f:%f\n", (x_est - x_est_startup)
														,(y_est - y_est_startup)
														,(/*z_est - z_est_startup*/revise_height));

							DEBUG("udp sn_send_rc_command with speed_coefficient=%f\n", speed_coefficient);

#ifdef SPEED_LIMIT_FLAG
							if (face_mission == true || body_mission == true)
							{
								cmd0 = cmd0*speed_coefficient;
								cmd1 = cmd1*speed_coefficient;
							}
							else
							{
								if ((/*z_est - z_est_startup*/revise_height)  <= 5.0f)
								{
									//avoid the optic flow error
									if (((/*z_est - z_est_startup*/revise_height) <= 1.0f)
										|| snav_data->sonar_0_raw.range <= 1.0f)
									{
										cmd0 = cmd0*0.5f*speed_coefficient;
										cmd1 = cmd1*0.5f*speed_coefficient;
									}
									else
									{
										if (circle_mission || return_mission)
										{
											cmd0 = cmd0*speed_coefficient;
											cmd1 = cmd1*speed_coefficient;
										}
										else
										{
											cmd0 = cmd0*speed_coefficient;
											cmd1 = cmd1*speed_coefficient;
										}
									}
								}
								else
								{
									// Slow down when height grow
									cmd0 = cmd0*speed_coefficient*0.2f*(20.0/revise_height);
									cmd1 = cmd1*speed_coefficient*0.2f*(20.0/revise_height);

									if (cmd0 > 0.8)
									{
										cmd0 = 0.8;
									}
									else if (cmd0 < 0.15)
									{
										cmd0 = 0.15;
									}

									if (cmd1 > 0.8)
									{
										cmd1 = 0.8;
									}
									else if (cmd1 < 0.15)
									{
										cmd1 = 0.15;
									}
								}
							}
#endif
							// Add by wlh
							if (revise_full_flag == 1)
							{
								if ((fabs(cmd0) < 1e-4 && fabs(cmd1) < 1e-4 ) || reverse_ctrl_count != 0)
								{
									if ((old_cmd0 != 0 || old_cmd1 != 0)  && reverse_ctrl_count < stop_control_num)
									{
										if (reverse_ctrl_step == 0)
										{
											reverse_ctrl_step = 1;
										}

										if (reverse_ctrl_step == 1
											&& (fabs(old_cmd00+old_cmd0) > 0.05
												|| fabs(old_cmd11+old_cmd1) > 0.05)
											&& (fabs(old_cmd0)+0.05 > fabs(old_cmd00))
											&& (fabs(old_cmd1)+0.05 > fabs(old_cmd11)))
										{
											int current_cmd_offset = cmd_offset;

											if (revise_ctrl_flag == 1)
											{
												if(reverse_ctrl_count == 0 && (revise_height> 0.3) && snav_data->general_status.on_ground == 0 )
												{
													reverse_ctrl_count++;
													old_pitch = snav_data->attitude_estimate.pitch;
													old_roll  = -snav_data->attitude_estimate.roll;
													DEBUG("[control break]------old_pitch[%f]old_roll[%f]speed_xy[%f]\n",old_pitch,old_roll,speed_xy);
												}
											}

											if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
												&& ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
											{
												current_cmd_offset = cmd_offset;
											}
											else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
											{
												current_cmd_offset = cmd_offset/2;
											}
											else
											{
												current_cmd_offset = cmd_offset*2;
											}

											cmd0 = old_cmd00 - old_cmd0/current_cmd_offset;
											old_cmd00 = old_cmd00 - old_cmd0/current_cmd_offset;

											cmd1 = old_cmd11 - old_cmd1/current_cmd_offset;
											old_cmd11 = old_cmd11 - old_cmd1/current_cmd_offset;

											DEBUG("gozero------speed_xy[%f]cmd0[%f]cmd1[%f]old_cmd0[%f]old_cmd1[%f]pitch[%f]roll[%f]\n",speed_xy,cmd0,cmd1,old_cmd0,old_cmd1,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll);

											reverse_ctrl_step = 1;

											if ((speed_xy < 0.1) && (speed_xy > 0.01))
											{
												reverse_ctrl_count = 0;
												old_cmd0 = 0;
												old_cmd1 = 0;
												old_cmd00 = 0;
												old_cmd11 = 0;
												DEBUG("-------------gozero_end-1--- last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]speed_xy[%f]\n",last_pitch,old_pitch,last_roll,old_roll,speed_xy);
											}

										}
										else if(fabs(old_cmd00+old_cmd0) <= 0.05 || fabs(old_cmd11+old_cmd1) <= 0.05)
										{
											reverse_ctrl_step = 2;
										}
										if(reverse_ctrl_step == 2 && (fabs(old_cmd00) > 0.05 || fabs(old_cmd11) > 0.05) && fabs(old_cmd00)+fabs(old_cmd11) > 0.05 )
										{
											int keep_cmd = 0;
											int current_cmd_offset = cmd_offset;

											if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
												&& ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
											{
												current_cmd_offset = cmd_offset*2;
											}
											else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
											{
												current_cmd_offset = cmd_offset*3;
											}
											else
											{
												current_cmd_offset = cmd_offset;
											}


											if((fabs(last_pitch + old_pitch) > fabs(old_pitch) && fabs(old_pitch) > 0.1)  || (fabs(last_roll + old_roll) > fabs(old_roll) && fabs(old_roll) > 0.1))//还未拉反方向
											{
												//current_cmd_offset = current_cmd_offset*6;
												keep_cmd = 2;
											}
											else
											{
												//printf("last_pitch[%f]pitch[%f]old_pitch[%f]\n",fabs(last_pitch),fabs(snav_data->attitude_estimate.pitch),fabs(old_pitch));
												//printf("last_roll [%f]roll [%f]old_roll [%f]\n",fabs(last_roll),fabs(snav_data->attitude_estimate.roll),fabs(old_roll));
												if((fabs(last_pitch) > fabs(snav_data->attitude_estimate.pitch) && fabs(old_pitch) > 0.1  && fabs(last_pitch) + 0.1 < fabs(old_pitch)) || (fabs(last_roll) > fabs(snav_data->attitude_estimate.roll) && fabs(old_roll) > 0.1 && fabs(last_roll) + 0.1 < fabs(old_roll)))//已经拉反，但是倾角还未到位
												{
													keep_cmd = 1;
												}
												else
												{
													keep_cmd = 0;
												}
											}

											if(sample_size_missing_count > 3)
											{
												keep_cmd = 3;
												reverse_ctrl_count = 1;
												sample_size_missing_count = 0;
												DEBUG("sample_size_missing_count[%d]******************cmd0[%f]cmd1[%f]speed_xy[%f] \n",sample_size_missing_count,old_cmd00,old_cmd11,speed_xy);
											}

											if(keep_cmd > 0)
											{
												//current_cmd_offset = cmd_offset*6;
												cmd0 = old_cmd00;
												cmd1 = old_cmd11;
											}
											else
											{
												cmd0 = old_cmd00 + old_cmd0/current_cmd_offset;
												old_cmd00 = old_cmd00 + old_cmd0/current_cmd_offset;

												cmd1 = old_cmd11 + old_cmd1/current_cmd_offset;
												old_cmd11 = old_cmd11 + old_cmd1/current_cmd_offset;
											}



											DEBUG("gozero------speed_xy[%f]cmd0[%f]cmd1[%f]cmd2[%f]cmd3[%f]---reverse_ctrl_count[%d]---old_cmd0-1[%f][%f]---estimated-desired[%f]---imu_lin_acc[%f]---sample_size_missing_count[%d]---pitch[%f]roll[%f]---current_cmd_offset[%d]---keep_cmd[%d]\n",speed_xy,cmd0,cmd1,cmd2,cmd3,reverse_ctrl_count,old_cmd0,old_cmd1,fabs(estimated_xy_sqrt - desired_xy_sqrt),fabs(imu_lin_acc),sample_size_missing_count,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll,current_cmd_offset,keep_cmd);

											//if((fabs(last_pitch + old_pitch) < 0.05 && (fabs(last_roll + old_roll) < 0.05)
											if ((speed_xy < 0.1) && (speed_xy > 0.05))
											{
												reverse_ctrl_count = 0;//反拉结束
												old_cmd0 = 0;
												old_cmd1 = 0;
												old_cmd00 = 0;
												old_cmd11 = 0;
												DEBUG("-------------gozero-2_end---last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]speed_xy[%f]\n",last_pitch,old_pitch,last_roll,old_roll,speed_xy);
											}

											if(fabs(old_cmd00) <= fabs(old_cmd0/current_cmd_offset) &&	fabs(old_cmd11) <= fabs(old_cmd1/current_cmd_offset))
												DEBUG("gozero-------------------------------------------------------------end\n");

											last_pitch = snav_data->attitude_estimate.pitch;
											last_roll = -snav_data->attitude_estimate.roll;

											reverse_ctrl_step = 2;

										}
										else if(reverse_ctrl_step > 1 && reverse_ctrl_count > 0)
										{
											cmd0 = old_cmd00;
											cmd1 = old_cmd11;
											reverse_ctrl_count = 0;
										}

									}
								}
								else
								{
									if (reverse_ctrl_count == 0)
									{
										old_cmd0 = cmd0;
										old_cmd1 = cmd1;


										old_cmd00 = old_cmd0;
										old_cmd11 = old_cmd1;
										reverse_ctrl_step = 0;
										DEBUG("------speed_xy[%f]cmd0[%f]-cmd1[%f]\n",speed_xy,cmd0,cmd1);
									}
								}

								if (reverse_ctrl_count > 0)
								{
									reverse_ctrl_count++;
								}

								if (reverse_ctrl_count > stop_control_num)
								{
									reverse_ctrl_count = 0;
								}
							}
							//Add end

							cmd3 = cmd3*0.15f;

							DEBUG("[Current UDP sample_size]: [%d]\n", sample_size);

							DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
							DEBUG("UDP FINAL SEND CMD cmd0,cmd1,cmd2,cmd3:%f,%f,%f,%f\n",cmd0,cmd1,cmd2,cmd3);
							DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");

#ifdef HEIGHT_LIMIT
							// Limit the height
							if (((/*z_est - z_est_startup*/revise_height) >= height_limit) && (cmd2 > 0))
							{
								DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

								cmd2 = 0;

								memset(result_to_client,0,MAX_BUFF_LEN);
								memcpy(result_to_client, SNAV_INFO_OVER_SAFE_HEIGHT, MAX_BUFF_LEN);
								length=sendto(server_udp_sockfd,result_to_client,strlen(result_to_client),0,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr));
							}
#endif
							/*
							if ((x_vel > fMaxVel) || y_vel > fMaxVel)
							{
								sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
										cmd0, cmd1, cmd2, cmd3,
										&cmd0, &cmd1, &cmd2, &cmd3);
							}
							*/

							// Send the commands to Snapdragon Navigator with default RC options
			        		sn_send_rc_command(type, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
							state = MissionState::LOITER;
						}

						loop_counter++;
						continue;
					}
				}
				//task
				else if (udp_msg_array.size() >= 2 && (udp_msg_array[0].compare(SNAV_TASK_GET_INFO) == 0))
				{
					DEBUG("[%d]:prepare pro_tcp_send to send back to client\n",loop_counter);

					circle_cam_point_direct = atoi(udp_msg_array[1].c_str());
					if ((circle_cam_point_direct != 1) && (circle_cam_point_direct != -1))
					{
						circle_cam_point_direct = 1;
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_INFO_RETURN);

					char battery_info[MAX_BUFF_LEN];
					char rpm_info[MAX_BUFF_LEN];
					char sonar_info[MAX_BUFF_LEN];
					char gps_info[MAX_BUFF_LEN];
					char xyz_info[MAX_BUFF_LEN];
					char rpy_info[MAX_BUFF_LEN];
					char flight_state_info[MAX_BUFF_LEN];
					char drone_state_info[MAX_BUFF_LEN];

					memset(battery_info, 0, MAX_BUFF_LEN);
					memset(rpm_info, 0, MAX_BUFF_LEN);
					memset(sonar_info, 0, MAX_BUFF_LEN);
					memset(gps_info, 0, MAX_BUFF_LEN);
					memset(xyz_info, 0, MAX_BUFF_LEN);
					memset(rpy_info, 0, MAX_BUFF_LEN);
					memset(flight_state_info, 0, MAX_BUFF_LEN);
					memset(drone_state_info, 0, MAX_BUFF_LEN);

					sprintf(battery_info, "battery_info:%f", voltage);
					sprintf(rpm_info, "rpm_info:%d:%d:%d:%d", snav_data->esc_raw.rpm[0],
															  snav_data->esc_raw.rpm[1],
															  snav_data->esc_raw.rpm[2],
															  snav_data->esc_raw.rpm[3]);
					//sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
					sprintf(sonar_info,"sonar_info:%f", revise_height);
					sprintf(rpy_info, "rpy_info:%f:%f:%f", snav_data->attitude_estimate.roll,
														   snav_data->attitude_estimate.pitch,
														   snav_data->attitude_estimate.yaw);
					sprintf(flight_state_info, "flight_state_info:%d", state);
					sprintf(drone_state_info, "drone_state_info:%d", drone_state);
					if (gps_enabled != 1)
					{
						sprintf(gps_info, "gps_info:disable");
					}
					else
					{
						if (gps_status != SN_DATA_VALID)
						{
							sprintf(gps_info, "gps_info:invalid");
						}
						else
						{
							sprintf(gps_info, "gps_info:%d:%d", snav_data->gps_0_raw.longitude,
																snav_data->gps_0_raw.latitude);
						}
					}

					if (state == MissionState::ON_GROUND)
					{
						sprintf(xyz_info, "xyz_info:0:0:0");
					}
					else
					{
#ifdef GPS_MAG_SWITCH
						if (gps_enabled && (gps_status == SN_DATA_VALID) && (take_off_with_gps_valid))
						{
							sprintf(xyz_info, "xyz_info:%f:%f:%f", (x_est_gps - x_est_gps_startup),
																   (y_est_gps - y_est_gps_startup),
																   (z_est_gps - z_est_gps_startup));
						}
						else
#endif
						{
							sprintf(xyz_info, "xyz_info:%f:%f:%f", (x_est-x_est_startup),
																   (y_est-y_est_startup),
																   (/*z_est - z_est_startup*/revise_height));
						}
					}


					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, battery_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, rpm_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, sonar_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, gps_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, xyz_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, rpy_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, flight_state_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, drone_state_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, drone_state_error);

					DEBUG("SNAV_TASK_GET_INFO_RETURN result_to_client=%s\n", result_to_client);

					//sendback the udp data
					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("udp sendto SNAV_TASK_GET_INFO_RETURN length=%d\n", length);

					continue;
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SNAV_PROXY_VERSION) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, VERSION_NUM);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN result_to_client=%s, length=%d\n", result_to_client, length);

					continue;
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_LINARO_VERSION) == 0))
				{
					FILE *version_fp;
					char expected_version[64];

					if ((version_fp = fopen("/etc/systeminfo.cfg", "r")) != NULL)
					{
						int i=0;

						while (fgets(expected_version, sizeof(expected_version), version_fp) != NULL)
						{
							DEBUG("linaro version:%s\n", expected_version);
						}

						fclose(version_fp);
					}

					if (expected_version[strlen(expected_version)-1] == '\n')
					{
						expected_version[strlen(expected_version)-1] = '\0';
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_LINARO_VERSION_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, expected_version);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_LINARO_VERSION_RETURN result_to_client=%s, length=%d\n", result_to_client, length);

					continue;
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SNAV_VERSION) == 0))
				{
					//check current version	print $2,$3
					char get_version_cmd[128] = "dpkg -l snav-dev | tail -n 1 | awk '{print $3}'";
					char current_version[64];

				  	FILE *fp = popen(get_version_cmd, "r");
			        fgets(current_version, sizeof(current_version), fp);
			        pclose(fp);

					if (current_version[strlen(current_version)-1] == '\n')
					{
						current_version[strlen(current_version)-1] = '\0';
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_SNAV_VERSION_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, current_version);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_SNAV_VERSION_RETURN result_to_client=%s, length=%d\n", result_to_client, length);

					continue;
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_QCAM_VERSION) == 0))
				{
					//check current version	print $2,$3
					char get_version_cmd[128] = "dpkg -l mm-video | tail -n 1 | awk '{print $3}'";
					char current_version[64];

				  	FILE *fp = popen(get_version_cmd, "r");
			        fgets(current_version, sizeof(current_version), fp);
			        pclose(fp);

					if (current_version[strlen(current_version)-1] == '\n')
					{
						current_version[strlen(current_version)-1] = '\0';
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_QCAM_VERSION_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, current_version);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_QCAM_VERSION_RETURN result_to_client=%s, length=%d\n", result_to_client, length);

					continue;
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_STORAGE) == 0))
				{
					char get_storage_total[128] = "df -h | grep -w '/' | head -n 1 | awk '{print $2}'";
					char get_storage_free[128] = "df -h | grep -w '/' | head -n 1 | awk '{print $4}'";
					char current_storage_total[128];
					char current_storage_free[128];

					memset(current_storage_total, 0, 128);
					memset(current_storage_free, 0, 128);

				  	FILE *fp_total = popen(get_storage_total, "r");
			        fgets(current_storage_total, sizeof(current_storage_total), fp_total);
			        pclose(fp_total);

					FILE *fp_free = popen(get_storage_free, "r");
			        fgets(current_storage_free, sizeof(current_storage_free), fp_free);
			        pclose(fp_free);

					if (current_storage_total[strlen(current_storage_total)-1] == '\n')
					{
						current_storage_total[strlen(current_storage_total)-1] = '\0';
					}

					if (current_storage_free[strlen(current_storage_free)-1] == '\n')
					{
						current_storage_free[strlen(current_storage_free)-1] = '\0';
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_STORAGE_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, current_storage_total);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, current_storage_free);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_STORAGE_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SD_STATUS) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_GET_SD_STATUS_RETURN);
					if (access(SDCARD_DIR, F_OK) == 0)
					{
						strcat(result_to_client, ",1");
					}
					else
					{
						strcat(result_to_client, ",-1");
					}

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_GET_SD_STATUS_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_SSID_PWD);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_MODIFY_SSID_PWD result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CHECK_WIFI_MODE) == 0))
				{
					char get_current_wifi_mode[128] = "cat /etc/hostapd.conf | grep -w 'hw_mode=g'";

					char current_wifi_mode[128];
					memset(current_wifi_mode, 0, 128);

				  	FILE *fp_get_mode = popen(get_current_wifi_mode, "r");
			        fgets(current_wifi_mode, sizeof(current_wifi_mode), fp_get_mode);
			        pclose(fp_get_mode);

					if (current_wifi_mode[strlen(current_wifi_mode)-1] == '\n')
					{
						current_wifi_mode[strlen(current_wifi_mode)-1] = '\0';
					}

					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CHECK_WIFI_MODE);
					if (strcmp(current_wifi_mode, "#hw_mode=g") == 0)
					{
						strcat(result_to_client, STR_SEPARATOR);
						strcat(result_to_client, "5");
					}
					else if (strcmp(current_wifi_mode, "hw_mode=g") == 0)
					{
						strcat(result_to_client, STR_SEPARATOR);
						strcat(result_to_client, "2");
					}

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_CHECK_WIFI_MODE result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_WIFI_5G);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_MODIFY_WIFI_5G result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_WIFI_2G);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_MODIFY_WIFI_2G result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_TAKE_OFF) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_TAKE_OFF);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_TAKE_OFF result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_LAND) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_LAND);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_LAND result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 3) && (udp_msg_array[0].compare(SNAV_CMD_CIRCLE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CIRCLE);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_CIRCLE result_to_client=%s, length=%d\n", result_to_client, length);

#ifdef CIRCLE_HEIGHT_LIMIT_FLAG
					if ((/*z_est - z_est_startup*/revise_height) > 5.0)
					{
						continue;
					}
#endif
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_PANORAMA) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_PANORAMA);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, udp_msg_array[1].c_str());

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_PANORAMA result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MAG_CALIBRATE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MAG_CALIBRATE);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_MAG_CALIBRATE result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_OPTIC_FLOW_CALIB) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_OPTIC_FLOW_CALIB);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_OPTIC_FLOW_CALIB result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_HOR_CALIBRATE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_HOR_CALIBRATE);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_HOR_CALIBRATE result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_RETURN) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_RETURN);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_TRAIL_NAVIGATION);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_TRAIL_NAVIGATION result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FACE_FOLLOW);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, udp_msg_array[1].c_str());

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_FACE_FOLLOW result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW_MODE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FACE_FOLLOW_MODE);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, udp_msg_array[1].c_str());

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_FACE_FOLLOW_MODE result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_BODY_FOLLOW) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_CMD_RETURN_BODY_FOLLOW);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, udp_msg_array[1].c_str());

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_CMD_RETURN_BODY_FOLLOW result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_TASK_CONFIRM_LAND) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_CONFIRM_LAND_RETURN);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, udp_msg_array[1].c_str());

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_CONFIRM_LAND_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_SNAV_UPDATE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_SNAV_UPDATE_RETURN);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_SNAV_UPDATE_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_LINARO_UPDATE) == 0))
				{
					memset(result_to_client, 0, MAX_BUFF_LEN);
					sprintf(result_to_client, "%s", SNAV_TASK_LINARO_UPDATE_RETURN);

					length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
										(struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
					DEBUG("SNAV_TASK_LINARO_UPDATE_RETURN result_to_client=%s, length=%d\n", result_to_client, length);
				}
				// For test
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("speed_limit") == 0))
				{
					speed_coefficient = atof(udp_msg_array[1].c_str());
					DEBUG("udp receive  speed_coefficient=%f\n",speed_coefficient);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("revise_height") == 0))
				{
					use_revise_height = atoi(udp_msg_array[1].c_str());
					DEBUG("udp receive  use_revise_height=%d\n",use_revise_height);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("gps_mag_flag") == 0))
				{
					gps_mag_flag = atoi(udp_msg_array[1].c_str());
					DEBUG("udp receive  gps_mag_flag=%d\n",gps_mag_flag);
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("revise_rule") == 0))
				{
					revise_rule = atoi(udp_msg_array[1].c_str());
					DEBUG("udp receive  revise_rule=%d\n",revise_rule);

					if (revise_rule == 10)
					{
						revise_type_sample_size = 0;
					}
					else if (revise_rule == 11)
					{
						revise_type_sample_size = 1;
					}
					else if (revise_rule == 20)
					{
						revise_type_desire = 0;
					}
					else if (revise_rule == 21)
					{
						revise_type_desire = 1;
					}
					else if (revise_rule == 30)
					{
						revise_type_linacc = 0;
					}
					else if (revise_rule == 31)
					{
						revise_type_linacc = 1;
					}
				}
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("revise_type") == 0))
				{
					revise_type = atoi(udp_msg_array[1].c_str());
					DEBUG("udp receive  revise_type=%d\n",revise_type);

					if (revise_type == 10)
					{
						revise_full_flag = 0;
					}
					else if (revise_type == 11)
					{
						revise_full_flag = 1;
					}
					else if (revise_type == 20)
					{
						revise_ctrl_flag = 0;
					}
					else if (revise_rule == 21)
					{
						revise_ctrl_flag = 1;
					}
				}
#ifdef HEIGHT_LIMIT
				else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("height_limit") == 0))
				{
					height_limit = atof(udp_msg_array[1].c_str());
					DEBUG("udp receive  height_limit=%f\n",height_limit);
				}
#endif
			}

			struct timeval tv;
			gettimeofday(&tv, NULL);
			double t_now = tv.tv_sec + tv.tv_usec * 1e-6;

			// Desired velocity in vehicle world frame
			float x_vel_des = 0;
			float y_vel_des = 0;
			float z_vel_des = 0;
			float yaw_vel_des = 0;

			// Modify ssid and pwd
			if ((udp_msg_array.size() >= 2)
				&& (udp_msg_array[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING)
				/*&& (on_ground_flag == 1)*/)
			{
				char ssid[MAX_BUFF_LEN];
				char pwd[MAX_BUFF_LEN];
				char sed_str[MAX_BUFF_LEN];

				memset(ssid, 0, MAX_BUFF_LEN);
				memset(pwd, 0, MAX_BUFF_LEN);
				memset(sed_str, 0, MAX_BUFF_LEN);

				memcpy(ssid, udp_msg_array[1].c_str(), MAX_BUFF_LEN);

				if (udp_msg_array.size() >= 3)
				{
					memcpy(pwd, udp_msg_array[2].c_str(), MAX_BUFF_LEN);
					sprintf(sed_str,
							"sed -i 's/^ssid=.*$/ssid=%s/; s/^wpa_passphrase=.*$/wpa_passphrase=%s/'  /etc/hostapd.conf",
							ssid, pwd);
				}
				else
				{
					sprintf(sed_str,
							"sed -i 's/^ssid=.*$/ssid=%s/' /etc/hostapd.conf",
							ssid);
				}
				system(sed_str);
				system("chmod 755 /etc/hostapd.conf");
				//system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");
				system("pkill hostapd");
				sleep(10);	//10s
				system("hostapd -B /etc/hostapd.conf");
			}

			// Switch wifi mode 5G/2.4G
			if ((udp_msg_array.size() >= 1)
				&& ((udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0)
					|| (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0)))
			{
				if ((props_state == SN_PROPS_STATE_NOT_SPINNING) /*&& (on_ground_flag == 1)*/)
				{
					if (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0)
					{
						system("sed -i 's/^hw_mode=g$/#hw_mode=g/'  /etc/hostapd.conf");
						system("sed -i 's/^channel=0 # use ch0 to enable ACS$/#channel=0 # use ch0 to enable ACS/'  /etc/hostapd.conf");
						system("sed -i 's/^#hw_mode=a$/hw_mode=a/'  /etc/hostapd.conf");
						system("sed -i 's/^#channel=165 # some channel in 5Ghz band$/channel=165 # some channel in 5Ghz band/'  /etc/hostapd.conf");

						system("chmod 755 /etc/hostapd.conf");
						//system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");
						system("pkill hostapd");
						sleep(10);	//10s
						system("hostapd -B /etc/hostapd.conf");
					}
					else if (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0)
					{
						system("sed -i 's/^#hw_mode=g$/hw_mode=g/'  /etc/hostapd.conf");
						system("sed -i 's/^#channel=0 # use ch0 to enable ACS$/channel=0 # use ch0 to enable ACS/'  /etc/hostapd.conf");
						system("sed -i 's/^hw_mode=a$/#hw_mode=a/'  /etc/hostapd.conf");
						system("sed -i 's/^channel=165 # some channel in 5Ghz band$/#channel=165 # some channel in 5Ghz band/'  /etc/hostapd.conf");

						system("chmod 755 /etc/hostapd.conf");
						//system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");
						system("pkill hostapd");
						sleep(10);	//10s
						system("hostapd -B /etc/hostapd.conf");
					}
				}
			}

			/*
			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_START_PLAN_TEST_START) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{
				system("uf1_waypoint_fly_test -d");
			}

			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_START_PLAN_TEST_END) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{

			}

			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_START_EMERG_STOP) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{
				system("stop snav");
			}
			*/

			// Update
			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_TASK_SNAV_UPDATE) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING)
				/*&& (on_ground_flag == 1)*/)
			{
				// Notice the interactWithOta process to send msg to OTA app
				system("chmod 777 /tmp/update-snav.zip");
				send_ota_snav_flag = true;
				strcpy(ota_snav_path_buff, "/tmp/update-snav.zip");
			}

			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_TASK_LINARO_UPDATE) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING)
				/*&& (on_ground_flag == 1)*/)
			{
				system("chmod 777 /tmp/update-linaro.zip");
				send_ota_linaro_flag = true;
				strcpy(ota_linaro_path_buff, "/tmp/update-linaro.zip");
			}

			// MAG calibrate
			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_MAG_CALIBRATE) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{
				bool keep_going = true;
				bool calib_started = false;
				unsigned int attempt_number = 0;
				const unsigned int kMaxAttempts = 10;
				bool calib_result = false;

				while (keep_going)
				{
					static unsigned int circle_counter = 0;

					if (sn_update_data() != 0)
					{
						DEBUG("sn_update_data failed in accel_calibrate\n");
						keep_going = false;
					}
					else
					{
						if (snav_data->general_status.current_mode == SN_MAGNETOMETER_CALIBRATION_MODE)
						{
							calib_started = true;
							SnCalibStatus status;
							sn_get_magnetometer_calibration_status(&status);
							if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
							{
								DEBUG("[%u] Magnetometer calibration is in progress\n",circle_counter);
							}
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
						{
							DEBUG("[%u] Magnetometer calibration was completed successfully\n",circle_counter);
							keep_going = false;
							calib_result = true;
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
						{
							DEBUG("[%u] Magnetometer calibration failed\n", circle_counter);
							keep_going = false;
							calib_result = false;
						}
						else
						{
							if (attempt_number < kMaxAttempts)
							{
								DEBUG("[%u] Sending command (attempt %u) to start magnetometer calibration\n",
												circle_counter, attempt_number);
								sn_start_magnetometer_calibration();
								attempt_number++;
							}
							else
							{
								DEBUG("[%u] Unable to start calibration\n", circle_counter);
								keep_going = false;
							}
						}

						circle_counter++;
	    				usleep(100000);	//100ms

						if (circle_counter >= 600)
						{
							DEBUG("[%u] MAG Calibrate TimeOut calib_result=%d\n", circle_counter,calib_result);
							keep_going = false;
						}
					}
				}

				memset(result_to_client,0,MAX_BUFF_LEN);
				sprintf(result_to_client,"%s",SNAV_INFO_MAG_CALIBRATE_RESULT);
				strcat(result_to_client, STR_SEPARATOR);
				strcat(result_to_client, calib_result==true?"1":"0");

				length=sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
									, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
				DEBUG("udp sendto SNAV_INFO_MAG_CALIBRATE_RESULT length=%d\n",length);
			}

			// OPTIC_FLOW calibrate
			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_OPTIC_FLOW_CALIB) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{
				bool keep_going = true;
				bool calib_started = false;
				unsigned int attempt_number = 0;
				const unsigned int kMaxAttempts = 10;
				bool calib_result = false;

				while (keep_going)
				{
					static unsigned int circle_counter = 0;

					if (sn_update_data() != 0)
					{
						DEBUG("sn_update_data failed in accel_calibrate\n");
						keep_going = false;
					}
					else
					{
						if (snav_data->general_status.current_mode == SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE)
						{
							calib_started = true;
							SnCalibStatus status;
							sn_get_optic_flow_camera_yaw_calibration_status(&status);
							if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
							{
								DEBUG("[%u] Optic flow camera yaw calibration is in progress\n",circle_counter);
							}
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
						{
							DEBUG("[%u] Optic flow camera yaw calibration was completed successfully\n",circle_counter);
							keep_going = false;
							calib_result = true;
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
						{
							DEBUG("[%u] Optic flow camera yaw calibration failed\n", circle_counter);
							keep_going = false;
							calib_result = false;
						}
						else
						{
							if (attempt_number < kMaxAttempts)
							{
								DEBUG("[%u] Sending command (attempt %u) to start optic flow camera yaw calibration\n",
												circle_counter, attempt_number);
								sn_start_optic_flow_camera_yaw_calibration();
								attempt_number++;
							}
							else
							{
								DEBUG("[%u] Unable to start calibration\n", circle_counter);
								keep_going = false;
							}
						}

						circle_counter++;
	    				usleep(100000);	//100ms

						if (circle_counter >= 600)
						{
							DEBUG("[%u] Optic flow camera yaw Calibrate TimeOut calib_result=%d\n", circle_counter,calib_result);
							keep_going = false;
						}
					}
				}

				memset(result_to_client,0,MAX_BUFF_LEN);
				sprintf(result_to_client,"%s",SNAV_INFO_OPTIC_FLOW_CALIB_RESULT);
				strcat(result_to_client, STR_SEPARATOR);
				strcat(result_to_client, calib_result==true?"1":"0");

				length=sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
									, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
				DEBUG("udp sendto SNAV_INFO_OPTIC_FLOW_CALIB_RESULT length=%d\n",length);
			}

			// HOR calibrate
			if ((udp_msg_array.size() >= 1)
				&& (udp_msg_array[0].compare(SNAV_CMD_HOR_CALIBRATE) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING))
			{
				bool keep_going = true;
				bool calib_started = false;
				unsigned int attempt_number = 0;
				const unsigned int kMaxAttempts = 10;
				bool calib_result = false;

				while (keep_going)
				{
					static unsigned int circle_counter = 0;

					if (sn_update_data() != 0)
					{
						DEBUG("sn_update_data failed in accel_calibrate\n");
						keep_going = false;
					}
					else
					{
						if (snav_data->general_status.current_mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
						{
							calib_started = true;
							SnCalibStatus status;
							sn_get_static_accel_calibration_status(&status);

							if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
							{
								DEBUG("[%u] Static accel calibration is in progress\n",circle_counter);
							}
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
						{
							DEBUG("[%u] Static accel calibration was completed successfully\n",circle_counter);
							keep_going = false;
							calib_result = true;
						}
						else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
						{
							DEBUG("[%u] Static accel calibration failed\n", circle_counter);
							keep_going = false;
							calib_result = false;
						}
						else
						{
							if (attempt_number < kMaxAttempts)
							{
								DEBUG("[%u] Sending command (attempt %u) to start static accel calibration\n",
														circle_counter, attempt_number);
								sn_start_static_accel_calibration();
								attempt_number++;
							}
							else
							{
								DEBUG("[%u] Unable to start calibration\n", circle_counter);
								keep_going = false;
							}
						}

						circle_counter++;
	    				usleep(100000);	//100ms

						if (circle_counter >= 600)
						{
							DEBUG("[%u] HOR Calibrate TimeOut calib_result=%d\n", circle_counter,calib_result);
							keep_going = false;
						}
					}
				}

				memset(result_to_client,0,MAX_BUFF_LEN);
				sprintf(result_to_client,"%s",SNAV_INFO_HOR_CALIBRATE_RESULT);
				strcat(result_to_client, STR_SEPARATOR);
				strcat(result_to_client, calib_result==true?"1":"0");

				length=sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
									, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
				DEBUG("SNAV_INFO_MAG_CALIBRATE_RESULT result_to_client=%s, length=%d\n", result_to_client, length);
			}

			// FaceFollow switch
			if ((udp_msg_array.size() >= 2)
				&& (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW) == 0))
			{
				if (strcmp(udp_msg_array[1].c_str(), "on") == 0)
				{
					face_follow_switch = true;
					body_follow_switch = false;

					send_face_follow_swither_flag = true;
					memset(face_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
					strcpy(face_follow_swither_buff, "fdon");
				}
				else
				{
					face_follow_switch = false;

					send_face_follow_swither_flag = true;
					memset(face_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
					strcpy(face_follow_swither_buff, "fdoff");
				}

				face_mission = false;
				body_mission = false;
			}

			// BodyFollow switch
			if ((udp_msg_array.size() >= 2)
				&& (udp_msg_array[0].compare(SNAV_CMD_BODY_FOLLOW) == 0))
			{
				if (strcmp(udp_msg_array[1].c_str(), "on") == 0)
				{
					body_follow_switch = true;
					face_follow_switch = false;

					send_body_follow_swither_flag = true;
					memset(body_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
					strcpy(body_follow_swither_buff, "bdon");
				}
				else
				{
					body_follow_switch = false;

					send_body_follow_swither_flag = true;
					memset(body_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
					strcpy(body_follow_swither_buff, "bdoff");
				}

				face_mission = false;
				body_mission = false;
			}

			// FaceFollow mode
			if ((udp_msg_array.size() >= 2)
				&& (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW_MODE) == 0))
			{
				int flag = atoi(udp_msg_array[1].c_str());

				if (flag == 1)
				{
					face_rotate_switch = false;
				}
				else
				{
					face_rotate_switch = true;
				}
			}



			// Hover or Other Mission handle
			static bool mission_has_begun = false;

			if ((props_state == SN_PROPS_STATE_NOT_SPINNING)
				&& (on_ground_flag == 1)
				&& (!mission_has_begun))
			{
				state = MissionState::ON_GROUND;
			}

			if (state == MissionState::ON_GROUND)
			{
				take_off_with_gps_valid = false;
			}

			if (state == MissionState::ON_GROUND)
			{
				// Send zero velocity while vehicle sits on ground
				x_vel_des = 0;
				y_vel_des = 0;
				z_vel_des = 0;
				yaw_vel_des = 0;

				if (udp_msg_array.size() >= 1)
				{
					if (udp_msg_array[0].compare(SNAV_CMD_TAKE_OFF) == 0)
					{
						revise_height = 0.15;

						mission_has_begun = true;
						state = MissionState::STARTING_PROPS;
					}
					else if(udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0)
					{
						int position_num = 0;
						int i=0;
						int lati, longi;

						position_num = atoi(udp_msg_array[1].c_str());

						DEBUG("Trail Navigation position_num:%d\n", position_num);

						if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
						{
							continue;
						}

						for (i=0; i<2*position_num; i+=2)
						{
							lati = atoi(udp_msg_array[2+i].c_str());
							longi = atoi(udp_msg_array[2+i+1].c_str());

							DEBUG("Trail Navigation [%d]-lati,logi:%d,%d\n", i/2, lati, longi);

							NavigationPosition pos;
							pos.latitude = lati;
							pos.longitude = longi;

							if (pos.latitude !=0 && pos.longitude !=0)
							{
								trail_navigation_positions.push_back(pos);
							}
						}

						mission_has_begun = true;
						state = MissionState::STARTING_PROPS;

						trail_navigation_mission = true;
					}
					else
					{
						state = MissionState::ON_GROUND;
					}
				}
			}
			else if (state == MissionState::STARTING_PROPS)
			{
				x_vel_des = 0;
				y_vel_des = 0;
				z_vel_des = 0;
				yaw_vel_des = 0;

				if (udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				if (props_state == SN_PROPS_STATE_NOT_SPINNING)
				{
					//store current position to the boot position
					x_est_startup = x_est;
					y_est_startup = y_est;
					z_est_startup = z_est;
					yaw_est_startup = yaw_est;

#ifdef GPS_MAG_SWITCH
					if (gps_enabled && (gps_status == SN_DATA_VALID))
					{
						x_est_gps_startup = x_est_gps;
						y_est_gps_startup = y_est_gps;
						z_est_gps_startup = z_est_gps;
						yaw_est_gps_startup = yaw_est_gps;

						take_off_with_gps_valid = true;
					}
					else
					{
						take_off_with_gps_valid = false;

						memset(result_to_client, 0, MAX_BUFF_LEN);
						sprintf(result_to_client, "%s", SNAV_TASK_SHOW_GPS_TAKE_OFF_ERROR);

						length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0
											, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
						DEBUG("udp sendto SNAV_TASK_SHOW_GPS_ERROR result_to_client=%s, length=%d\n", result_to_client, length);
					}
#endif
					sn_spin_props();
				}
				else if (props_state == SN_PROPS_STATE_SPINNING)
				{
					state = MissionState::TAKEOFF;
				}
			}
			else if (state == MissionState::TAKEOFF)
			{
				if(udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				if (props_state == SN_PROPS_STATE_SPINNING)
				{
					// Avoid some propers do not spin
					if ((snav_data->esc_raw.rpm[0] == 0)
						|| (snav_data->esc_raw.rpm[1] == 0)
						|| (snav_data->esc_raw.rpm[2] == 0)
						|| (snav_data->esc_raw.rpm[3] == 0))
					{
						state = MissionState::LANDING;
						loop_counter++;

						memset(result_to_client, 0, MAX_BUFF_LEN);
						sprintf(result_to_client, "%s", SNAV_TASK_SHOW_MOTER_ERROR);

						length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0
											, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
						DEBUG("udp sendto SNAV_TASK_SHOW_MOTER_ERROR result_to_client=%s, length=%d\n", result_to_client, length);

						continue;
					}

					// Command constant positive z velocity during takeoff
					x_vel_des = 0;
					y_vel_des = 0;
					z_vel_des = kTakeoffSpeed;
					yaw_vel_des = 0;

					if (trail_navigation_mission)
					{
						if (z_des - z_est_startup >= fTrarilHeight)
						{
							state = MissionState::LOITER;
						}
						else if(z_des - z_est_startup > 0.5f*fTrarilHeight)
						{
							z_vel_des = kTakeoffSpeed*0.5f;
						}
					}
					else
					{
						if (z_des - z_est_startup >= fTakeOffHeight)
						{
							state = MissionState::LOITER;
						}
						else if(z_des - z_est_startup > 0.5f*fTakeOffHeight)
						{
							z_vel_des = kTakeoffSpeed*0.3f;
						}
					}
				}
				else if (props_state == SN_PROPS_STATE_NOT_SPINNING)
				{
					state = MissionState::STARTING_PROPS;
				}
			}
			else if (state == MissionState::LANDING)
			{
				if (props_state == SN_PROPS_STATE_SPINNING)
				{
					//reset all the mission
					current_position =0;

					circle_mission=false;
					calcCirclePoint = false;

					panorama_mission = false;
					calcPanoramaPoint = false;

					trail_navigation_mission = false;

					return_mission = false;

					face_mission = false;
					body_mission = false;

					face_follow_switch = false;
					body_follow_switch = false;

					// Command constant negative z velocity during landing
					x_vel_des = 0;
					y_vel_des = 0;
					z_vel_des = kLandingSpeed;
					yaw_vel_des = 0;


					if ((snav_data->sonar_0_raw.range <= 1.0)
						&& (snav_data->sonar_0_raw.range >= 0.2)
						&& !confirm_land
						&& (mode != SN_EMERGENCY_LANDING_MODE))
					{
						state = MissionState::LOITER;

						memset(result_to_client,0,MAX_BUFF_LEN);
						sprintf(result_to_client,"%s",SNAV_TASK_SHOW_LAND_CONFIRM);

						length=sendto(server_udp_sockfd,result_to_client,strlen(result_to_client),0,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr));
						DEBUG("udp sendto SNAV_TASK_SHOW_LAND_CONFIRM length=%d\n",length);

						continue;
					}

					//smoothly landing start
					if((snav_data->sonar_0_raw.range < 0.3)
						&&  (!landing_near_ground))
					{
						landing_near_ground = true;
					}

					if (landing_near_ground)
					{
						//z_vel_des = -1.125;		// 2/3 = 0.75 cmd3
						z_vel_des = -1.5;		// 2/3 = 1 cmd3 z_vel_des_max=1.2, cmd3 max=0.8
					}
					else
					{
						//z_vel_des = -0.45;		// 2/3 = 0.3 cmd3

						if(snav_data->sonar_0_raw.range < 2.5)
						{
							z_vel_des = -0.225;		// 2/3 = 0.15 cmd3
						}
					}
					//smoothly landing end

					if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
					{
						// Snapdragon Navigator has determined that vehicle is on ground,
						// so it is safe to kill the propellers
						sn_stop_props();
					}
				}
				else
				{
					state = MissionState::ON_GROUND;

					// reset all the mission
					current_position = 0;

					circle_mission = false;
					calcCirclePoint = false;

					panorama_mission = false;
					calcPanoramaPoint = false;

					trail_navigation_mission = false;

					return_mission = false;

					face_mission = false;
					body_mission = false;

					face_follow_switch = false;
					body_follow_switch = false;
				}
			}
			else if (state == MissionState::LOITER)
			{
				if (props_state == SN_PROPS_STATE_SPINNING)
				{
					confirm_land = false;

					// Maintain current position
					x_vel_des = 0;
					y_vel_des = 0;
					z_vel_des = 0;
					yaw_vel_des = 0;

					static bool entering_loiter = true;
					static double t_loiter_start = 0;

					if (entering_loiter)
					{
						t_loiter_start = t_now;
						entering_loiter = false;
					}

					if(udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
					{
						state = MissionState::LANDING;
						loop_counter++;
						continue;
					}
					else if (udp_msg_array.size() >= 2
							&& (udp_msg_array[0].compare(SNAV_TASK_CONFIRM_LAND) ==0))
					{
						if (udp_msg_array[1].compare("yes") ==0)
						{
							state = MissionState::LANDING;
							confirm_land = true;
							loop_counter++;
							continue;
						}
					}
					//panorama task
					else if(udp_msg_array.size() >= 2
							&& udp_msg_array[0].compare(SNAV_CMD_PANORAMA) == 0
							&& !circle_mission
							&& !return_mission
							&& !trail_navigation_mission
							&& !face_mission
							&& !body_mission)
					{
						clockwise = atoi(udp_msg_array[1].c_str());
						//clockwise = atoi(udp_msg_array[2].c_str());

						curSendMode = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

						panorama_mission = true;
						calcPanoramaPoint = true;

						point_count = 12;
						vel_target = 0.5;	 //m/sec

						angle_per = (2*M_PI)/(3*point_count);	//(120degree/point_count)

						DEBUG("Panorama_mission: clockwise,point_count,angle_per:%d,%d,%f\n",
								clockwise,point_count,angle_per);
					}
					//circel task
					else if(udp_msg_array.size() >= 3
							&& udp_msg_array[0].compare(SNAV_CMD_CIRCLE) == 0
							&& !panorama_mission
							&& !return_mission
							&& !trail_navigation_mission
							&& !face_mission
							&& !body_mission)
					{
						radius = atof(udp_msg_array[1].c_str());
						clockwise = atoi(udp_msg_array[2].c_str());

						curSendMode = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
						circle_mission = true;
						calcCirclePoint = true;

						if (radius <= 5.0)
						{
							point_count = 36;
						}
						else
						{
							point_count = 72;
						}
						vel_target = 0.75;	 //m/sec
						angle_per = 2*M_PI/point_count;

						DEBUG("LOITER circle: radius,clockwise,point_count,vel_target,angle_per:%f,%d,%d,%f,%f\n",
								radius,clockwise,point_count,vel_target,angle_per);
					}
					//return task
					else if(udp_msg_array.size() >= 1
							&& udp_msg_array[0].compare(SNAV_CMD_RETURN) == 0
							&& !panorama_mission
							&& !circle_mission
							&& !trail_navigation_mission
							&& !face_mission
							&& !body_mission)
					{
#ifdef GPS_MAG_SWITCH
						if (gps_enabled)
						{
							if ((gps_status != SN_DATA_VALID) || !take_off_with_gps_valid)
							{
								memset(result_to_client, 0, MAX_BUFF_LEN);

								if (gps_status != SN_DATA_VALID)
								{
									sprintf(result_to_client, "%s", SNAV_TASK_SHOW_GPS_RETURN_ERROR_ONE);
								}
								else
								{
									sprintf(result_to_client, "%s", SNAV_TASK_SHOW_GPS_RETURN_ERROR_TWO);
								}

								length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0
													, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
								DEBUG("udp sendto SNAV_TASK_SHOW_GPS_ERROR result_to_client=%s, length=%d\n", result_to_client, length);

								loop_counter++;
								continue;
							}
							else
							{
								curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
								return_mission = true;

								state = MissionState::TRAJECTORY_FOLLOW;
								entering_loiter = true;
							}
						}
						else
#endif
						{
							curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
							return_mission = true;

							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}

									// Every circle recalc the distance and yaw_diff from home for Return mission
#ifdef GPS_MAG_SWITCH
						if (gps_enabled)
						{
							distance_gps_home_squared = (x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
															+ (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps);
							yaw_gps_target_home = atan2(y_est_gps_startup-y_est_gps, x_est_gps_startup-x_est_gps);

							DEBUG("[%d]:return_mission distance_gps_home_squared, yaw_gps_target_home:[%f, %f]\n",
										loop_counter, distance_gps_home_squared, yaw_gps_target_home);

							if (distance_gps_home_squared > distance_home_squared_threshold)
							{
								fly_home = false;
							}
							else
							{
								fly_home = true;
							}
						}
						else
#endif
						{
							distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est) + (y_est_startup-y_est)*(y_est_startup-y_est);
							yaw_target_home = atan2(y_est_startup-y_est, x_est_startup-x_est);

							DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
										loop_counter,distance_home_squared,yaw_target_home);

							if (distance_home_squared > distance_home_squared_threshold)
							{
								fly_home = false;
							}
							else
							{
								fly_home = true;
							}
						}

						gohome_x_vel_des = 0;
				        gohome_y_vel_des = 0;
				        gohome_z_vel_des = 0;
				        gohome_yaw_vel_des = 0;

						DEBUG("LOITER return enter TRAJECTORY_FOLLOW\n");
					}
					//trail_navigation task
					else if(udp_msg_array.size() >= 1
							&& udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0
							&& !panorama_mission
							&& !circle_mission
							&& !return_mission
							&& !face_mission
							&& !body_mission)
					{
						int position_num = 0;
						int i=0;
						int lati, longi;

						position_num = atoi(udp_msg_array[1].c_str());

						DEBUG("Trail Navigation position_num:%d\n", position_num);

						if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
						{
							continue;
						}

						for (i=0; i<2*position_num; i+=2)
						{
							lati = atoi(udp_msg_array[2+i].c_str());
							longi = atoi(udp_msg_array[2+i+1].c_str());

							DEBUG("Trail Navigation [%d]-lati,logi:%d,%d\n", i/2, lati, longi);

							NavigationPosition pos;
							pos.latitude = posGpsCurrent.latitude;
							pos.longitude = posGpsCurrent.longitude;

							if(pos.latitude !=0 && pos.longitude !=0)
							{
								trail_navigation_positions.push_back(pos);
							}
						}

						trail_navigation_mission = true;

						state = MissionState::TRAJECTORY_FOLLOW;
						entering_loiter = true;

						DEBUG("LOITER return enter TRAJECTORY_FOLLOW\n");
					}
					//cuiyc add face detect begin
					else if(cur_body.have_face && face_follow_switch) //cuiyc add face detect begin
					{
						float face_offset ;
						face_offset = M_PI*cur_body.angle/180;
						DEBUG("followme have_face\n" );

						if((fabs(face_offset) >min_angle_offset
								|| fabs(cur_body.distance -safe_distance) >0.05
								|| fabs(cur_body.hegith_calib)>0.1)
							&& !panorama_mission
							&& !circle_mission
							&& !return_mission
							&& !trail_navigation_mission)
						{
							face_mission = true;
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
							DEBUG("face_offset:%f	distance:%f hegith_calib:%f\n",face_offset,
								cur_body.distance,cur_body.hegith_calib);
							DEBUG("followme have_face LOITER -> FACE_FOLLOW\n" );
						}
					}
					else if(cur_body.have_body && body_follow_switch)
					{
						float body_offset ;
						body_offset = M_PI*cur_body.angle/180;
						DEBUG("followme have_body\n" );

						if ((fabs(body_offset)>min_angle_offset
								|| fabs(cur_body.velocity)>0.05f)
							&& !panorama_mission
							&& !circle_mission
							&& !return_mission
							&& !trail_navigation_mission)
						{
							body_mission= true;
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
							DEBUG("body angle:%f	velocity:%f\n",cur_body.angle,cur_body.velocity);
							DEBUG("followme have_body LOITER -> BODY_FOLLOW\n" );
						}
					}//cuiyc add face detect end

					if (circle_mission && (t_now - t_loiter_start > kLoiterTime))
					{
						if(calcCirclePoint)
						{
							//circle center
							float circle_center_x;
							float circle_center_y;
							float yaw_t =0;

							if (circle_cam_point_direct == 1)	//camera point inside
							{
								circle_center_x = x_est-x_est_startup + radius*cos(yaw_est);
								circle_center_y = y_est-y_est_startup + radius*sin(yaw_est);

								if ((clockwise != 1) && (clockwise != -1))
								{
									clockwise = 1;
								}

								circle_positions.clear();//clear and recaculate.
								for(int k=0;k<=point_count;k++)	//the last position must be the same as the first position
								{
									Position pos;
									//pos.x = (1-cos(angle_per*k))*radius + x_est;
									//pos.y = -sin(angle_per*k)*radius + y_est;

									//yaw_t = yaw_est+angle_per*k*clockwise;
									yaw_t = yaw_est-angle_per*k*clockwise;

									if(yaw_t > M_PI)
									{
										yaw_t = yaw_t -2*M_PI;
									}
									else if (yaw_t < -M_PI)
									{
										yaw_t = yaw_t + 2*M_PI;
									}

									if(yaw_t>0)
									{
										pos.x = cos(yaw_t-M_PI)*radius + circle_center_x;
										pos.y = sin(yaw_t-M_PI)*radius + circle_center_y;
									}
									else
									{
										pos.x = cos(yaw_t+M_PI)*radius + circle_center_x;
										pos.y = sin(yaw_t+M_PI)*radius + circle_center_y;
									}
									pos.z = z_des - z_est_startup;
									pos.yaw = yaw_t;

									circle_positions.push_back(pos);
								}
							}
							else	//camera point outside
							{
								circle_center_x = x_est-x_est_startup - radius*cos(yaw_est);
								circle_center_y = y_est-y_est_startup - radius*sin(yaw_est);

								if ((clockwise != 1) && (clockwise != -1))
								{
									clockwise = 1;
								}

								circle_positions.clear();//clear and recaculate.
								for(int k=0;k<=point_count;k++)	//the last position must be the same as the first position
								{
									Position pos;
									//pos.x = (1-cos(angle_per*k))*radius + x_est;
									//pos.y = -sin(angle_per*k)*radius + y_est;

									//yaw_t = yaw_est+angle_per*k*clockwise;
									yaw_t = yaw_est-angle_per*k*clockwise;

									if(yaw_t > M_PI)
									{
										yaw_t = yaw_t -2*M_PI;
									}
									else if (yaw_t < -M_PI)
									{
										yaw_t = yaw_t + 2*M_PI;
									}

									if(yaw_t>0)
									{
										pos.x = circle_center_x - cos(yaw_t-M_PI)*radius;
										pos.y = circle_center_y - sin(yaw_t-M_PI)*radius;
									}
									else
									{
										pos.x = circle_center_x - cos(yaw_t+M_PI)*radius;
										pos.y = circle_center_y - sin(yaw_t+M_PI)*radius;
									}
									pos.z = z_des - z_est_startup;
									pos.yaw = yaw_t;

									circle_positions.push_back(pos);
								}
							}

							calcCirclePoint = false;

							DEBUG("circle_center_point [%f,%f]\n", circle_center_x,circle_center_y);

							for(int k=0;k<(int)circle_positions.size();k++)
							{
								DEBUG("[%d] position #%u: [%f,%f,%f,%f]\n",k,
									k,circle_positions[k].x,circle_positions[k].y,
									circle_positions[k].z,circle_positions[k].yaw);
							}
						}

						state = MissionState::TRAJECTORY_FOLLOW;
						entering_loiter = true;

					}
					else if(panorama_mission && (t_now - t_loiter_start > kLoiterTime))
					{
						if(calcPanoramaPoint)
						{
							float yaw_t =0;

							if ((clockwise != 1) && (clockwise != -1))
							{
								clockwise = 1;
							}

							panorama_positions.clear();//clear and recaculate.
							for(int k=0;k<=point_count;k++)
							{
								Position pos;
								pos.x = x_est-x_est_startup;
								pos.y = y_est-y_est_startup;

								yaw_t = yaw_est-angle_per*k*clockwise;

								if(yaw_t > M_PI)
								{
									yaw_t = yaw_t -2*M_PI;
								}
								else if (yaw_t < -M_PI)
								{
									yaw_t = yaw_t + 2*M_PI;
								}
								pos.z = z_des - z_est_startup;
								pos.yaw = yaw_t;

								panorama_positions.push_back(pos);
							}

							calcPanoramaPoint = false;

							for(int k=0;k<(int)panorama_positions.size();k++)
							{
								DEBUG("@@@@[%d] panorama_positions #%u: [%f,%f,%f,%f]\n",k,
									k,panorama_positions[k].x,panorama_positions[k].y,
									panorama_positions[k].z,panorama_positions[k].yaw);
							}
						}

						state = MissionState::TRAJECTORY_FOLLOW;
						entering_loiter = true;
					}
					else if (trail_navigation_mission)
					{
						if (t_now - t_loiter_start > kLoiterTime)
						{
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
					}
				}
			}
			else if(state == MissionState::TRAJECTORY_FOLLOW)
			{
				float accel_max  = 1.5;   	//m/sec
				float stopping_accel = 1.0; //m/sec

				static double t_last = 0;

				static float vel_x_des_sent = 0;
				static float vel_y_des_sent = 0;
				static float vel_z_des_sent = 0;
				static float vel_yaw_des_sent = 0;

				float command_diff_x;
				float command_diff_y;
				float command_diff_z;
				float command_diff_yaw;

				static float vel_x_target = 0;
				static float vel_y_target = 0;
				static float vel_z_target = 0;
				static float vel_yaw_target = 0;

				yaw_vel_des = 0;

				if(udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				if (panorama_mission)
				{
					command_diff_yaw = panorama_positions[current_position].yaw - yaw_des;

					DEBUG("[%d] [panorama_mission yaw_des command_diff_yaw]: [%f,%f]\n",
								loop_counter, yaw_des, command_diff_yaw);

					if (command_diff_yaw > M_PI)
					{
						command_diff_yaw = command_diff_yaw - 2*M_PI;
					}
					else if (command_diff_yaw < -M_PI)
					{
						command_diff_yaw = command_diff_yaw+ 2*M_PI;
					}

					if (fabs(command_diff_yaw)>M_PI*0.25f)
					{
						state = MissionState::LOITER;
						current_position =0;
						panorama_mission=false;
						calcPanoramaPoint = false;
						continue;
					}

					if (fabs(command_diff_yaw)<0.03)
					{
						// Close enough, move on
						current_position++;

						if ((current_position == 3) || (current_position == 6) || (current_position == 9))
						{
							send_panorama_flag = true;
							if (current_position == 3)
							{
								strcpy(panorama_buff, "snap,1");
							}
							else if (current_position == 6)
							{
								strcpy(panorama_buff, "snap,2");
							}
							else if (current_position == 9)
							{
								strcpy(panorama_buff, "snap,3");
							}
						}

						if (current_position >= panorama_positions.size())
						{
							// No more panorama_positions,
							state = MissionState::LOITER;
							current_position = 0;
							panorama_mission=false;
							calcPanoramaPoint = false;
						}
					}

					vel_x_target = 0;
					vel_y_target = 0;
					vel_z_target = 0;

					vel_yaw_target = command_diff_yaw/(angle_per*vel_target);

					if (vel_yaw_target<0)
					{
						vel_yaw_target = -0.5;
					}
					else
					{
						vel_yaw_target = 0.5;
					}

					DEBUG("[%d][panorama_mission current_position vel_yaw_target]: [%d %f]\n",
										loop_counter,current_position,vel_yaw_target);
				}
				else if(circle_mission)
				{
					bool stop_flag = false;

					FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_des-z_est_startup, yaw_des};
					FlatVars last_vel = {vel_x_target, vel_y_target, vel_z_target, vel_yaw_target};

					// Yaw waypoint is given in txt file
					FlatVars des_pos = {circle_positions[current_position].x,
										circle_positions[current_position].y,
										circle_positions[current_position].z,
										circle_positions[current_position].yaw};

					// Stop only at last waypoint. If stopping at all waypoints is desired, make always true
					if (current_position == circle_positions.size())
					{
						stop_flag = true;
					}
					else
					{
						stop_flag = false;
					}

					if (goto_waypoint_for_circle(current_state, des_pos, last_vel, stop_flag, &output_vel, &wp_goal_ret) == -1)
					{
						if (current_position == 0)
						{
							output_vel.x = 0;
							output_vel.y = 0;
							output_vel.z = 0;
							output_vel.yaw = 0;
						}
					}

					vel_x_target = output_vel.x;
			        vel_y_target = output_vel.y;
			        vel_z_target = output_vel.z;
			        vel_yaw_target = output_vel.yaw;

					// If reached waypoint is met, increment waypoint
					if ((wp_goal_ret&wp_goal_mask) == 0)
					{
						DEBUG("[%d][circle_mission reach point[%d]\n", loop_counter,current_position);

						current_position++;
						wp_goal_ret = 0b11111111;

						if (current_position >= circle_positions.size())
						{
							// No more circle_positions, so land
							state = MissionState::LOITER;
							current_position =0;
							circle_mission=false;
							calcCirclePoint = false;
						}
					}

					DEBUG("[%d][%d] [circle_mission current_position vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
									loop_counter,current_position, vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);

					/*
					command_diff_x = circle_positions[current_position].x - (x_des-x_est_startup);
					command_diff_y = circle_positions[current_position].y - (y_des-y_est_startup);
					command_diff_z = circle_positions[current_position].z - (z_des-z_est_startup);
					command_diff_yaw = circle_positions[current_position].yaw - yaw_des;


					DEBUG("[%d] [circle_mission x_des y_des z_des]: [%f %f %f]\n",
							loop_counter,x_des,y_des,z_des);

					if (command_diff_yaw > M_PI)
					{
						command_diff_yaw = command_diff_yaw - 2*M_PI;
					}
					else if (command_diff_yaw < -M_PI)
					{
						command_diff_yaw = command_diff_yaw+ 2*M_PI;
					}

					if (fabs(command_diff_yaw)>M_PI*0.25f)
					{
						state = MissionState::LOITER;
						current_position =0;
						circle_mission=false;
						calcCirclePoint = false;
						continue;
					}

					distance_to_dest = sqrt(command_diff_x*command_diff_x +
										 command_diff_y*command_diff_y +
										 command_diff_z*command_diff_z);

					if(distance_to_dest<0.01)
					{
						distance_to_dest = 0.01;
					}  //to prevent dividing by zero

					//if(distance_to_dest<0.04)	//&& fabs(command_diff_yaw) <angle_per
					if(distance_to_dest<0.06)
					{
						// Close enough, move on
						current_position++;
						if (current_position >= circle_positions.size())
						{
							// No more circle_positions, so land
							state = MissionState::LOITER;
							current_position =0;
							circle_mission=false;
							calcCirclePoint = false;
						}
					}

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;
					vel_yaw_target = command_diff_yaw/angle_per*vel_target;
					*/

					/*DEBUG("[%d] [distance_to_dest command_diff_x command_diff_y command_diff_z command_diff_yaw]: [%f %f %f %f %f]\n",
							loop_counter,distance_to_dest,command_diff_x,command_diff_y,command_diff_z,command_diff_yaw);*/

					//if (current_position >= 0.9*circle_positions.size()) //slow down
					//	vel_yaw_target =0;
					//if(vel_yaw_target>0.8f)vel_yaw_target=0.8f;
				}
				else if (trail_navigation_mission)
				{
					command_diff_x = CalcAxisDistance(trail_navigation_positions[current_position].longitude/1e7,
													  (float)posGpsCurrent.longitude/1e7);

					command_diff_y = CalcAxisDistance(trail_navigation_positions[current_position].latitude/1e7,
													  (float)posGpsCurrent.latitude/1e7);

					command_diff_z = 0;

					distance_to_dest = CalcDistance(trail_navigation_positions[current_position].latitude/1e7,
													trail_navigation_positions[current_position].longitude/1e7,
													(float)posGpsCurrent.latitude/1e7,
													(float)posGpsCurrent.longitude/1e7);


					if (distance_to_dest<0.01)
					{
						distance_to_dest = 0.01;
					}  //to prevent dividing by zero

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;

					if (distance_to_dest < 2.5 ) //about 150*150   x,y 1.5m
					{
						//if it is on the last waypoint then slow down before stopping
						float stopping_vel = sqrt(2*stopping_accel*distance_to_dest);
						if(stopping_vel<vel_target)
						{
							vel_target = stopping_vel;
						}
					}

					if (distance_to_dest<1) // 1m
					{
						// Close enough, move on
						current_position++;
						if (current_position >= trail_navigation_positions.size())
						{
						    state = MissionState::LOITER;
							trail_navigation_mission = false;
						}
					}
				}
				else if(face_mission) //cuiyc add face detect begin
				{
					//static float f_dest_yaw,f_dest_x,f_dest_y;
					static float distance_remain_x,distance_remain_y,distance_remain_z;
					static float forword_dis , parallel_dis,angle_face_offset;

					if(cur_body.newP )
					{
						angle_face_offset = cur_body.angle*M_PI/180;

						if(fabs(angle_face_offset) > min_angle_offset && face_rotate_switch)
						{
							if(cur_body.distance >(safe_distance -0.2f))
							{
								distance_remain_x = 0;
								distance_remain_y = 0;
								distance_remain_z = 0;
								vel_yaw_target = angle_face_offset*1.5;
							}
							else  // too close back first
							{
							    forword_dis = cur_body.distance-safe_distance;
								parallel_dis = 0;
								distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

								DEBUG("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
											loop_counter,forword_dis,parallel_dis,distance_to_dest);

								distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
								distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
								distance_remain_z = 0;
								vel_yaw_target = 0;
							}

						}
						else
						{
							forword_dis = cur_body.distance-safe_distance;
							parallel_dis = tan(angle_face_offset)*cur_body.distance;

							distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

							DEBUG("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
									loop_counter,forword_dis,parallel_dis,distance_to_dest);

							distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
							distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
							distance_remain_z = cur_body.hegith_calib;
							vel_yaw_target = 0;
						}
						cur_body.newP = false;
					}

					if(((fabs(angle_face_offset))< min_angle_offset && fabs(distance_to_dest) <0.05f)
						|| !cur_body.have_face)
					{
						if(fabs(distance_remain_z) > 0.1f)
						{
							vel_z_target = distance_remain_z*vel_target;
							if(vel_z_target >vel_target) vel_z_target =vel_target;
						}
						else
						{
							state = MissionState::LOITER;
							vel_z_target = 0;
							DEBUG(" face_mission follow face-> LOITER\n" );
							face_mission = false;
							continue;
						}
					}

					//for test rotate
					distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
					distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;
					distance_remain_z = distance_remain_z - vel_z_des_sent*0.02f;

					distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
											 distance_remain_y*distance_remain_y);

					DEBUG("[%d] face_mission [distance_remain_x distance_remain_y,distance_remain_z]: [%f %f %f]\n",
							loop_counter,distance_remain_x,distance_remain_y,distance_remain_z);

					if(fabs(distance_remain_x) <0.05f)
						distance_remain_x =0;
					if(fabs(distance_remain_y) <0.05f)
						distance_remain_y =0;
					if(fabs(distance_remain_z) < 0.1f)
						distance_remain_z =0;

					if(fabs(distance_to_dest) >0.05f)
					{
						if(forword_dis <0)
						{
						    vel_x_target = distance_remain_x*face_vel_limit*2.5f;
						    vel_y_target = distance_remain_y*face_vel_limit*2.5f;
						}
						else{
						    vel_x_target = distance_remain_x*face_vel_limit;
						    vel_y_target = distance_remain_y*face_vel_limit;
						}

						if(vel_x_target >face_vel_limit) vel_x_target =face_vel_limit;
						if(vel_y_target >face_vel_limit) vel_y_target =face_vel_limit;
						if(vel_x_target < -1*face_vel_limit) vel_x_target =-1*face_vel_limit;
						if(vel_y_target < -1*face_vel_limit) vel_y_target =-1*face_vel_limit;
					}
					else
					{
						vel_x_target = 0;
						vel_y_target = 0;
					}

					if(fabs(distance_remain_z) > 0.1f)
					{
						vel_z_target = distance_remain_z*vel_target;
						if(vel_z_target >vel_target) vel_z_target = vel_target;
					}
					else
						vel_z_target = 0;

					if(vel_yaw_target > M_PI*0.5f)
						vel_yaw_target = M_PI*0.5f;

					DEBUG("[%d] face_mission [vel_x vel_y vel_z distance vel_yaw]: [%f %f %f %f %f]\n",
						loop_counter,vel_x_target,vel_y_target,vel_z_target,distance_to_dest,vel_yaw_target);
				}
				else if(body_mission)
				{
					//static float f_dest_yaw,f_dest_x,f_dest_y;
					static float speed ,angle_body_offset;
					static float backup_speed ,backup_angle;

                    //keep a long time (200ms) same speed and angle ,udp lag or miss or follow app be killed
					if(backup_speed == cur_body.velocity &&
						backup_angle == cur_body.angle)
					{
						if(bd_start_counter  >10)
						{
							body_mission = false;
							state = MissionState::LOITER;
							cur_body.velocity = 0;
							cur_body.angle    = 0;
							DEBUG("body_mission follow body->LOITER because no body info update\n");
						}
						else
							bd_start_counter++;
					}
					else
					{
						backup_speed = cur_body.velocity;
						backup_angle = cur_body.angle;
						bd_start_counter = 0;
					}

					angle_body_offset = cur_body.angle*M_PI/180;

					if(body_follow_prallel)
					{
						if((fabs(angle_body_offset)< min_angle_offset)
							|| !cur_body.have_body)
						{
							state = MissionState::LOITER;
							DEBUG("follow body_mission prallel follow body-> LOITER\n");
							body_mission = false;
							continue;
						}
						else
						{
							vel_x_target =  sin(yaw_est)*cur_body.angle/51.0f*body_speed_limit;
							vel_y_target = cos(yaw_est)*cur_body.angle/51.0f*body_speed_limit;
							vel_z_target =0;

							if(vel_x_target >body_speed_limit)
								vel_x_target = body_speed_limit;

							if(vel_y_target >body_speed_limit)
								vel_y_target = body_speed_limit;

							DEBUG(" [%d] follow body_mission prallel [vel_x_target vel_y_target ]: [%f %f]\n",
							loop_counter,vel_x_target,vel_y_target);

						}
					}
					else
					{
					speed = sqrt(vel_x_des_sent*vel_x_des_sent +vel_y_des_sent*vel_y_des_sent);

					if((fabs(angle_body_offset)< min_angle_offset && cur_body.velocity <0.05f)
						|| !cur_body.have_body)
					{
						state = MissionState::LOITER;
						DEBUG("body_mission follow body-> LOITER\n");
						body_mission = false;
						continue;
					}

					if(fabs(angle_body_offset) > min_angle_offset)
					{
						vel_yaw_target = angle_body_offset*vel_target*1.5;
						vel_x_target =0;
						vel_y_target =0;
						vel_z_target =0;
						DEBUG(" [%d] follow body_mission [vel_x_target vel_y_target vel_yaw_target]: [%f %f %f]\n",
							loop_counter,vel_x_target,vel_y_target,vel_yaw_target);
					}
					else
					{
					    vel_yaw_target = 0;

						//DEBUG("[%d] body_mission angle_body_offset: [%f] \n",
						//		loop_counter,angle_body_offset);

						float curheight = z_est - z_est_startup;
						float sonarheight = snav_data->sonar_0_raw.range;
						if(sonarheight > 1.0f && sonarheight < 2.5f)
							curheight = sonarheight;

						DEBUG("[%d] body_mission angle_body_offset: [%f] curheight:%f,sonarheight:%f\n",
								loop_counter,angle_body_offset,curheight,sonarheight);

						if( true/*curheight>1.9f && curheight <2.1f*/)
						{
							vel_x_target = cos(yaw_est)*cur_body.velocity;
							vel_y_target = sin(yaw_est)*cur_body.velocity;
							vel_z_target =0;

							if(vel_x_target >body_speed_limit)
								vel_x_target = body_speed_limit;

							if(vel_y_target >body_speed_limit)
								vel_y_target = body_speed_limit;

							DEBUG(" [%d] follow body_mission [vel_x_target vel_y_target vel_yaw_target]: [%f %f %f]\n",
							loop_counter,vel_x_target,vel_y_target,vel_yaw_target);
						}
						else
						{
							vel_x_target = 0;
							vel_y_target = 0;
							vel_z_target = (2.0f - curheight)*vel_target*1.5;

							if(vel_z_target >0.75f)vel_z_target = 0.75;
							DEBUG(" [%d]  body_mission correct height [vel_z_target curheight vel_yaw_target]: [%f %f %f]\n",
							loop_counter,vel_z_target,curheight,vel_yaw_target);
						}
					}
					}
				}//cuiyc add face detect end


				//return mission
				if (return_mission)
				{
					// Two step: turn yaw to home first, then fly directly to home.
					if (fly_home)
					{
#ifdef	GPS_MAG_SWITCH
						//if (gps_enabled && (gps_status == SN_DATA_VALID) && (gps_mag_flag == 1))
						if (gps_enabled)
						{
							float distance_gps_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
															+ (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));
							yaw_gps_target_home = atan2(y_est_gps_startup-y_est_gps, x_est_gps_startup-x_est_gps);

							DEBUG("[%d] return_mission [distance_gps_home, yaw_gps_target_home]: [%f, %f]\n"
										, loop_counter,distance_gps_home, yaw_gps_target_home);

							// Go to home waypoint
							if (distance_gps_home > 2)
							{
								goto_waypoint_with_gps({x_est_gps, y_est_gps, z_est_gps, yaw_est_gps}
														, {x_est_gps_startup, y_est_gps_startup, z_est_gps, yaw_gps_target_home}
														, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
														, true, &output_vel, &wp_goal_ret);
							}
							else
							{
								goto_waypoint_with_gps({x_est_gps, y_est_gps, z_est_gps, yaw_est_gps}
					 									, {x_est_gps_startup, y_est_gps_startup, z_est_gps, yaw_est_gps}
					 									, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
					 									, true, &output_vel, &wp_goal_ret);
							}
							// Go to home waypoint
						}
						else
#endif
						{
							float distance_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)
															+ (y_est_startup-y_est)*(y_est_startup-y_est));

							yaw_target_home = atan2(y_est_startup-y_est, x_est_startup-x_est);

							DEBUG("[%d] return_mission [distance_home, yaw_target_home]: [%f, %f]\n"
										, loop_counter,distance_home, yaw_target_home);

							// Go to home waypoint
							if (distance_home > 0.3)
							{
								goto_waypoint({x_des, y_des, z_des, yaw_des}
			 									, {x_est_startup, y_est_startup, z_des, yaw_target_home}
			 									, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
			 									, true, &output_vel, &wp_goal_ret);
							}
							else
							{
								goto_waypoint({x_des, y_des, z_des, yaw_des}
			 									, {x_est_startup, y_est_startup, z_des, yaw_des}
			 									, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
			 									, true, &output_vel, &wp_goal_ret);
							}

						}

						gohome_x_vel_des = output_vel.x;
				        gohome_y_vel_des = output_vel.y;
				        gohome_z_vel_des = output_vel.z;
				        gohome_yaw_vel_des = output_vel.yaw;

						DEBUG("[%d]:return_mission direct fly_home wp_goal_ret,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
									loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);


						float distance_from_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
																					+ (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));

						if (((wp_goal_ret&wp_goal_mask) == 0) ||  (distance_from_home < 2))
				        {
							// If home, enter landing
							wp_goal_ret = 0b11111111;
							state = MissionState::LANDING;
							return_mission = false;

							gohome_x_vel_des = 0;
					        gohome_y_vel_des = 0;
					        gohome_z_vel_des = 0;
					        gohome_yaw_vel_des = 0;

							fly_home = false;
				        }
					}
					else
					{
#ifdef	GPS_MAG_SWITCH
						//if (gps_enabled && (gps_status == SN_DATA_VALID) && (gps_mag_flag == 1))
						if (gps_enabled)
						{
							goto_waypoint_with_gps({x_des_gps, y_des_gps, z_des_gps, yaw_des_gps}
													, {x_des_gps, y_des_gps, z_des_gps, yaw_gps_target_home}
													, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
													, true, &output_vel, &wp_goal_ret);
						}
						else
#endif
						{
							goto_waypoint({x_des, y_des, z_des, yaw_des}
											, {x_des, y_des, z_des, yaw_target_home}
											, {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
											, true, &output_vel, &wp_goal_ret);
						}

						gohome_x_vel_des = output_vel.x;
				        gohome_y_vel_des = output_vel.y;
				        gohome_z_vel_des = output_vel.z;
				        gohome_yaw_vel_des = output_vel.yaw;

						DEBUG("[%d]:return_mission fly_raw wp_goal_ret,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
									loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);

						if ((wp_goal_ret&wp_goal_mask) == 0)
				        {
							// If waypoint is reached (facing home), enter Fly_home
							wp_goal_ret = 0b11111111;

							fly_home = true;
						}
					}
				}
				else
				{
					float delT;

					if(t_last != 0)
					{
						delT = (t_now - t_last);
					}
					else
					{
						delT = 0.02;
					}

					t_last = t_now;

					//now converge the velocity to desired velocity

					float v_del_max = accel_max*delT;

					float vel_x_diff = (vel_x_target - vel_x_des_sent);
					float vel_y_diff = (vel_y_target - vel_y_des_sent);
					float vel_z_diff = (vel_z_target - vel_z_des_sent);
					float vel_yaw_diff = (vel_yaw_target - vel_yaw_des_sent);

					DEBUG("[%d] [vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff]: [%f,%f,%f,%f] \n",
										loop_counter,vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff);

					float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
					                  vel_y_diff*vel_y_diff +
					                  vel_z_diff*vel_z_diff);

					if(vel_diff_mag<0.01)
					{
						vel_diff_mag = 0.01;
					}

					if(vel_diff_mag<v_del_max)
					{
						//send through the target velocity
						vel_x_des_sent = vel_x_target;
						vel_y_des_sent = vel_y_target;
						vel_z_des_sent = vel_z_target;
					}
					else
					{
						//converge to the target velocity at the max acceleration rate
						vel_x_des_sent += vel_x_diff/vel_diff_mag * v_del_max;
						vel_y_des_sent += vel_y_diff/vel_diff_mag * v_del_max;
						vel_z_des_sent += vel_z_diff/vel_diff_mag * v_del_max;
					}

					//smooth accel
					if(vel_yaw_diff<v_del_max)
					{
						vel_yaw_des_sent = vel_yaw_target;
					}
					else
					{
						vel_yaw_des_sent += v_del_max;
					}

					distance_to_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)+
									(y_est_startup-y_est)*(y_est_startup-y_est)+
									(z_est_startup-z_est)*(z_est_startup-z_est));

					yaw_vel_des = vel_yaw_des_sent;

					x_vel_des = vel_x_des_sent;
					y_vel_des = vel_y_des_sent;
					z_vel_des = vel_z_des_sent;

					DEBUG("[%d] [x_vel_des,y_vel_des,z_vel_des,yaw_vel_des]: [%f,%f,%f,%f] \n",
										loop_counter,x_vel_des,y_vel_des,z_vel_des,yaw_vel_des);
				}
			}
			else
			{
				// Unknown state has been encountered
				x_vel_des = 0;
				y_vel_des = 0;
				z_vel_des = 0;
				yaw_vel_des = 0;

				if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
				{
					sn_stop_props();
				}
			}


			if ((props_state == SN_PROPS_STATE_NOT_SPINNING)
					&& (on_ground_flag == 1)
					&& ((state == MissionState::TRAJECTORY_FOLLOW)
						|| (state == MissionState::LOITER)))
			{
				state = MissionState::ON_GROUND;

				circle_mission=false;
				calcCirclePoint = false;

				panorama_mission = false;
				calcPanoramaPoint = false;

				trail_navigation_mission = false;

				return_mission = false;

				face_mission=false;
				body_mission=false;

				face_follow_switch = false;
				body_follow_switch = false;
			}


			// reset all the mission when disconnect with the phone.
			if (!bHaveUdpClient
				&& (props_state == SN_PROPS_STATE_SPINNING)
				&& (state == MissionState::TRAJECTORY_FOLLOW
					|| state == MissionState::LOITER))
			{
				current_position =0;

				circle_mission = false;
				calcCirclePoint = false;

				panorama_mission = false;
				calcPanoramaPoint = false;

				trail_navigation_mission = false;

				return_mission = false;

				face_mission=false;
				body_mission=false;

				state = MissionState::LOITER;

				face_follow_switch = false;
				body_follow_switch = false;

				face_rotate_switch = false;

				send_face_follow_swither_flag = true;
				memset(face_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
				strcpy(face_follow_swither_buff, "fdoff");

				send_body_follow_swither_flag = true;
				memset(body_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
				strcpy(body_follow_swither_buff, "bdoff");
			}

			// Rotate velocity by estimated yaw angle before sending
			// This puts velocity in body-relative Z-up frame
			float x_vel_des_yawed = x_vel_des*cos(-yaw_est) - y_vel_des*sin(-yaw_est);
			float y_vel_des_yawed = x_vel_des*sin(-yaw_est) + y_vel_des*cos(-yaw_est);

			float gohome_x_vel_des_yawed = gohome_x_vel_des*cos(-yaw_est) - gohome_y_vel_des*sin(-yaw_est);
			float gohome_y_vel_des_yawed = gohome_x_vel_des*sin(-yaw_est) + gohome_y_vel_des*cos(-yaw_est);

			float cmd0 = 0;
			float cmd1 = 0;
			float cmd2 = 0;
			float cmd3 = 0;

			// Go from the commands in real units computed above to the
			// dimensionless commands that the interface is expecting using a
			// linear mapping
			//sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
			//sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,//curSendMode

			if (return_mission)
			{
				DEBUG("[sn_apply_cmd_mapping gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des]: [%f,%f,%f,%f]\n",
	  											gohome_x_vel_des_yawed,gohome_y_vel_des_yawed,gohome_z_vel_des,gohome_yaw_vel_des);
				sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
						gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des,
						&cmd0, &cmd1, &cmd2, &cmd3);
			}
			else
			{
				DEBUG("[sn_apply_cmd_mapping x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des]: [%f,%f,%f,%f]\n",
		  											x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
				sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
						x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
						&cmd0, &cmd1, &cmd2, &cmd3);
			}

			// Send the commands if in the right mode.
			//sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
			//sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,

			if(body_follow_switch && body_follow_prallel && cmd1 != 0)cmd0 =0; //cuiyc add for people follow
			if(face_follow_switch && (cmd0 != 0 || cmd1 != 0))cmd2 = 0;

			DEBUG("[sn_send_rc_command cmd0 cmd1 cmd2 cmd3]: [%f,%f,%f,%f]\n",cmd0,cmd1,cmd2,cmd3);

#ifdef SPEED_LIMIT_FLAG
			if (face_mission == true || body_mission == true)
			{
				cmd0 = cmd0*speed_coefficient;
				cmd1 = cmd1*speed_coefficient;
			}
			else
			{
				if ((/*z_est - z_est_startup*/revise_height)  <= 5.0f)
				{
					//avoid the optic flow error
					if (((/*z_est - z_est_startup*/revise_height) <= 1.0f)
						|| snav_data->sonar_0_raw.range <= 1.0f)
					{
						cmd0 = cmd0*0.5f*speed_coefficient;
						cmd1 = cmd1*0.5f*speed_coefficient;
					}
					else
					{
						if (circle_mission || return_mission)
						{
							cmd0 = cmd0*speed_coefficient;
							cmd1 = cmd1*speed_coefficient;
						}
						else
						{
							cmd0 = cmd0*speed_coefficient;
							cmd1 = cmd1*speed_coefficient;
						}
					}
				}
				else
				{
					cmd0 = cmd0*speed_coefficient*0.2f*(20.0/revise_height);
					cmd1 = cmd1*speed_coefficient*0.2f*(20.0/revise_height);

					if (cmd0 > 0.8)
					{
						cmd0 = 0.8;
					}
					else if (cmd0 < 0.15)
					{
						cmd0 = 0.15;
					}

					if (cmd1 > 0.8)
					{
						cmd1 = 0.8;
					}
					else if (cmd1 < 0.15)
					{
						cmd1 = 0.15;
					}
				}
			}
#endif
			DEBUG("[Current sample_size]: [%d]\n", sample_size);

			DEBUG("[final send_rc_command cmd0 cmd1 cmd2 cmd3]: [%f,%f,%f,%f]\n",cmd0,cmd1,cmd2,cmd3);

#ifdef HEIGHT_LIMIT
			//limit the height
			if (((z_est - z_est_startup)>=height_limit) && (cmd2 > 0))
			{
				cmd2 = 0;

				DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

				memset(result_to_client,0,MAX_BUFF_LEN);
				memcpy(result_to_client, SNAV_INFO_OVER_SAFE_HEIGHT, MAX_BUFF_LEN);
				length=sendto(server_udp_sockfd,result_to_client,strlen(result_to_client),0,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr));
			}
#endif

			//Add by wlh
			if (revise_full_flag == 1)
			{
				if ((fabs(cmd0) < 1e-4 && fabs(cmd1) < 1e-4 ) || reverse_ctrl_count != 0)
				{
					if ((old_cmd0 != 0 || old_cmd1 != 0)  && reverse_ctrl_count < stop_control_num)
					{
						if (reverse_ctrl_step == 0)
						{
							reverse_ctrl_step = 1;
						}

						if (reverse_ctrl_step == 1
							&& (fabs(old_cmd00+old_cmd0) > 0.05
								|| fabs(old_cmd11+old_cmd1) > 0.05)
							&& (fabs(old_cmd0)+0.05 > fabs(old_cmd00))
							&& (fabs(old_cmd1)+0.05 > fabs(old_cmd11)))
						{
							int current_cmd_offset = cmd_offset;

							if (revise_ctrl_flag == 1)
							{
								if(reverse_ctrl_count == 0 && (revise_height> 0.3) && snav_data->general_status.on_ground == 0 )
								{
									reverse_ctrl_count++;
									old_pitch = snav_data->attitude_estimate.pitch;
									old_roll  = -snav_data->attitude_estimate.roll;
									DEBUG("*******************************[control break]------old_pitch[%f]old_roll[%f]speed_xy[%f]\n",old_pitch,old_roll,speed_xy);
								}
							}

							if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
								&& ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
							{
								current_cmd_offset = cmd_offset;
							}
							else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
							{
								current_cmd_offset = cmd_offset/2;
							}
							else
							{
								current_cmd_offset = cmd_offset*2;
							}

							cmd0 = old_cmd00 - old_cmd0/current_cmd_offset;
							old_cmd00 = old_cmd00 - old_cmd0/current_cmd_offset;

							cmd1 = old_cmd11 - old_cmd1/current_cmd_offset;
							old_cmd11 = old_cmd11 - old_cmd1/current_cmd_offset;

							DEBUG("gozero------speed_xy[%f]cmd0[%f]cmd1[%f]old_cmd0[%f]old_cmd1[%f]pitch[%f]roll[%f]\n",speed_xy,cmd0,cmd1,old_cmd0,old_cmd1,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll);

							reverse_ctrl_step = 1;

							if ((speed_xy < 0.1) && (speed_xy > 0.01))
							{
								reverse_ctrl_count = 0;
								old_cmd0 = 0;
								old_cmd1 = 0;
								old_cmd00 = 0;
								old_cmd11 = 0;
								DEBUG("-------------gozero_end-1--- last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]speed_xy[%f]\n",last_pitch,old_pitch,last_roll,old_roll,speed_xy);
							}

						}
						else if(fabs(old_cmd00+old_cmd0) <= 0.05 || fabs(old_cmd11+old_cmd1) <= 0.05)
						  reverse_ctrl_step = 2;

						if(reverse_ctrl_step == 2 && (fabs(old_cmd00) > 0.05 || fabs(old_cmd11) > 0.05) && fabs(old_cmd00)+fabs(old_cmd11) > 0.05 )
						{
							int keep_cmd = 0;
							int current_cmd_offset = cmd_offset;

							if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
								&& ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
							{
								current_cmd_offset = cmd_offset*2;
							}
							else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
							{
								current_cmd_offset = cmd_offset*3;
							}
							else
							{
								current_cmd_offset = cmd_offset;
							}


							if((fabs(last_pitch + old_pitch) > fabs(old_pitch) && fabs(old_pitch) > 0.1)  || (fabs(last_roll + old_roll) > fabs(old_roll) && fabs(old_roll) > 0.1))//还未拉反方向
							{
								//current_cmd_offset = current_cmd_offset*6;
								keep_cmd = 2;
							}
							else
							{
								//printf("last_pitch[%f]pitch[%f]old_pitch[%f]\n",fabs(last_pitch),fabs(snav_data->attitude_estimate.pitch),fabs(old_pitch));
								//printf("last_roll [%f]roll [%f]old_roll [%f]\n",fabs(last_roll),fabs(snav_data->attitude_estimate.roll),fabs(old_roll));
								if((fabs(last_pitch) > fabs(snav_data->attitude_estimate.pitch) && fabs(old_pitch) > 0.1  && fabs(last_pitch) + 0.1 < fabs(old_pitch)) || (fabs(last_roll) > fabs(snav_data->attitude_estimate.roll) && fabs(old_roll) > 0.1 && fabs(last_roll) + 0.1 < fabs(old_roll)))//已经拉反，但是倾角还未到位
								{
									keep_cmd = 1;
								}
								else
								{
									keep_cmd = 0;
								}
							}


							if(sample_size < 5)
							{
								keep_cmd = 3;
								reverse_ctrl_count = 1;
								DEBUG("*****************************************sample_size < 5***************************************");
							}

							if(keep_cmd > 0)
							{
								//current_cmd_offset = cmd_offset*6;
								cmd0 = old_cmd00;
								cmd1 = old_cmd11;
							}
							else
							{
								cmd0 = old_cmd00 + old_cmd0/current_cmd_offset;
								old_cmd00 = old_cmd00 + old_cmd0/current_cmd_offset;

								cmd1 = old_cmd11 + old_cmd1/current_cmd_offset;
								old_cmd11 = old_cmd11 + old_cmd1/current_cmd_offset;
							}



							DEBUG("gozero------speed_xy[%f]cmd0[%f]cmd1[%f]cmd2[%f]cmd3[%f]---reverse_ctrl_count[%d]---old_cmd0-1[%f][%f]---estimated-desired[%f]---imu_lin_acc[%f]---sample_size_missing_count[%d]---pitch[%f]roll[%f]---current_cmd_offset[%d]---keep_cmd[%d]\n",speed_xy,cmd0,cmd1,cmd2,cmd3,reverse_ctrl_count,old_cmd0,old_cmd1,fabs(estimated_xy_sqrt - desired_xy_sqrt),fabs(imu_lin_acc),sample_size_missing_count,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll,current_cmd_offset,keep_cmd);

							//if((fabs(last_pitch + old_pitch) < 0.05 && (fabs(last_roll + old_roll) < 0.05)
							if ((speed_xy < 0.1) && (speed_xy > 0.05))
							{
								reverse_ctrl_count = 0;//反拉结束
								old_cmd0 = 0;
								old_cmd1 = 0;
								old_cmd00 = 0;
								old_cmd11 = 0;
								DEBUG("-------------gozero-2_end---last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]speed_xy[%f]\n",last_pitch,old_pitch,last_roll,old_roll,speed_xy);
							}

							if(fabs(old_cmd00) <= fabs(old_cmd0/current_cmd_offset) &&	fabs(old_cmd11) <= fabs(old_cmd1/current_cmd_offset))
								DEBUG("gozero-------------------------------------------------------------end\n");

							last_pitch = snav_data->attitude_estimate.pitch;
							last_roll = -snav_data->attitude_estimate.roll;

							reverse_ctrl_step = 2;
						}
						else if(reverse_ctrl_step > 1 && reverse_ctrl_count > 0)
						{
							cmd0 = old_cmd00;
							cmd1 = old_cmd11;
							reverse_ctrl_count = 0;
						}

					}
				}
				else
				{
					if (reverse_ctrl_count == 0)
					{
						old_cmd0 = cmd0;
						old_cmd1 = cmd1;


						old_cmd00 = old_cmd0;
						old_cmd11 = old_cmd1;
						reverse_ctrl_step = 0;
						DEBUG("------speed_xy[%f]cmd0[%f]-cmd1[%f]\n",speed_xy,cmd0,cmd1);
					}
				}

				if (reverse_ctrl_count > 0)
				{
					reverse_ctrl_count++;
				}

				if (reverse_ctrl_count > stop_control_num)
				{
					reverse_ctrl_count = 0;
				}
			}
			//Add end

			/*
			if ((x_vel > fMaxVel) || y_vel > fMaxVel)
			{
				sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
						cmd0, cmd1, cmd2, cmd3,
						&cmd0, &cmd1, &cmd2, &cmd3);
			}
			*/

			if (return_mission)
			{
				if (gps_enabled)
				{
					if (fly_home)
					{
						if ((revise_height) > 5)
						{
							cmd0 = 0.10*speed_coefficient*0.2f*(20.0/revise_height);

							if (fabs((float)snav_data->gps_pos_vel.velocity_estimated[0]) > 1.5)
							{
								cmd0 = cmd0*0.2f;
							}
						}
						else
						{
							cmd0 = 0.15;

							if (fabs((float)snav_data->gps_pos_vel.velocity_estimated[0]) > 1.5)
							{
								cmd0 = cmd0*0.2f;
							}
						}

						float distance_gps_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
															+ (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));

						if (distance_gps_home <= 5)
						{
							cmd0 = cmd0*0.5f;
						}

						cmd1 = 0;
						cmd2 = 0;
					}
					else
					{
						cmd3 = cmd3*0.5;
					}

					DEBUG("\n[%d] gps_return cmd0:cmd1:cmd2:cmd3:%f, %f, %f, %f", loop_counter, cmd0, cmd1, cmd2, cmd3);
				}
			}

			sn_send_rc_command(curSendMode, RC_OPT_LINEAR_MAPPING, cmd0, cmd1, cmd2, cmd3);


			// Print some information
			if (mode == SN_GPS_POS_HOLD_MODE)
			{
				DEBUG("\n[%d] SN_GPS_POS_HOLD_MODE. ", loop_counter);
			}
			else if (mode == SN_SENSOR_ERROR_MODE)
			{
				DEBUG("\n[%d] SENSOR ERROR MODE. ", loop_counter);
			}
			else if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
			{
				DEBUG("\n[%d] OPTIC FLOW VELOCITY MODE MODE. ", loop_counter);
			}
			else
			{
				DEBUG("\n[%d] UNDEFINED MODE :%d \n", loop_counter,mode);
			}


			if (props_state == SN_PROPS_STATE_NOT_SPINNING)
			{
				DEBUG("Propellers NOT spinning\n");
			}
			else if (props_state == SN_PROPS_STATE_STARTING)
			{
				DEBUG("Propellers attempting to spin\n");
			}
			else if (props_state == SN_PROPS_STATE_SPINNING)
			{
				DEBUG("Propellers spinning\n");
			}
			else
			{
				DEBUG("Unknown propeller state\n");
			}

			DEBUG("[%d] battery_voltage: %f\n", loop_counter, voltage);
			DEBUG("[%d] Current Sonar range: [%f]\n", loop_counter, snav_data->sonar_0_raw.range);
			DEBUG("[%d] commanded rates: [%f, %f, %f, %f]\n",
						loop_counter, x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des);

			DEBUG("[%d] [Origin Estimated x_est, y_est, z_est, yaw_est]: [%f, %f, %f, %f]\n",
						loop_counter, x_est, y_est, z_est, yaw_est);
			DEBUG("[%d] [Origin Desired x_des, y_des, z_des, yaw_des]: [%f, %f, %f, %f]\n",
						loop_counter, x_des, y_des, z_des, yaw_des);
			DEBUG("[%d] [x_est_startup, y_est_startup, z_est_startup, yaw_est_startup]: [%f, %f, %f, %f]\n",
						loop_counter, x_est_startup, y_est_startup, z_est_startup, yaw_est_startup);
			DEBUG("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
			DEBUG("[%d] [Reduced x_est, y_est, z_est, yaw_est]: [%f, %f, %f, %f]\n",
						loop_counter, x_est-x_est_startup, y_est-y_est_startup, z_est-z_est_startup, yaw_est);
			DEBUG("[%d] [Reduced x_des, y_des, z_des, yaw_des]: [%f, %f, %f, %f]\n",
						loop_counter, x_des-x_est_startup, y_des-y_est_startup, z_des-z_est_startup, yaw_des);

			if (state == MissionState::ON_GROUND)
			{
				DEBUG("[%d] ON_GROUND\n",loop_counter);
			}
			else if (state == MissionState::STARTING_PROPS)
			{
				DEBUG("[%d] STARTING_PROPS\n",loop_counter);
			}
			else if (state == MissionState::TAKEOFF)
			{
				DEBUG("[%d] TAKEOFF\n",loop_counter);
				DEBUG("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
			}
			else if (state == MissionState::TRAJECTORY_FOLLOW)
			{
				DEBUG("[%d] TRAJECTORY_FOLLOW\n",loop_counter);

				if (circle_mission)
				{
					DEBUG("[%d] circle_mission position #%u: [%f,%f,%f,%f]\n",
								loop_counter,
								current_position,
								circle_positions[current_position].x,
								circle_positions[current_position].y,
								circle_positions[current_position].z,
								circle_positions[current_position].yaw);
				}
			}
			else if (state == MissionState::LOITER)
			{
				DEBUG("[%d] LOITER\n",loop_counter);
			}
			else if (state == MissionState::LANDING)
			{
				DEBUG("[%d] LANDING\n",loop_counter);
			}
			else
			{
				DEBUG("[%d] STATE UNKNOWN\n",loop_counter);
			}
		}
		loop_counter++;
	}

	fclose(stdout);
	fclose(stderr);

	return 0;
}
