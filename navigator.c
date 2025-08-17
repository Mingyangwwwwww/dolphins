
#include "./navigator.h"

double navi_velocity[50] = {0};  // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾惧綊鏌ｉ幋锝呅撻柛銈呭閺屻倝宕妷锔芥瘎婵炲濮靛銊ф閹捐纾兼繛鍡樺姉閵堟澘顪冮妶鍡樿偁闁搞儯鍔屾禍閬嶆⒑鐟欏嫬鍔
int navi_velocity_i = 0;        
double navi_velocity_sum = 0.0;  // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾剧懓顪冪€ｎ亝鎹ｉ柣顓炴閵嗘帒顫濋敐鍛婵°倗濮烽崑娑⑺囬悽绋挎瀬鐎广儱顦粈瀣亜閹扳晛鐏ù婊€鍗抽弻锝嗘償閳ュ啿杈
double navi_velocity_speed = 0.0; // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾剧懓顪冪€ｎ亝鎹ｉ柣顓炴閵嗘帒顫濋敐鍛婵°倗濮烽崑娑⑺囬悽绋挎瀬鐎广儱顦粈瀣亜閹哄秶鍔嶆い鏂挎处缁绘繂顕ラ柨瀣
double navi_angular[50] = {0};   // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾惧綊鏌ｉ幋锝呅撻柛銈呭閺屻倝宕妷锔芥瘎婵炲濮靛銊ф閹捐纾兼繛鍡樺姉閵堟澘顪冮妶鍡樿偁闁搞儯鍔屾禍閬嶆⒑鐟欏嫬鍔
int navi_angular_i = 0;          // 缂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾惧綊鏌熼梻瀵割槮缁炬儳婀遍埀顒傛嚀鐎氫即宕戞繝鍌樷偓鎺撶節濮橆厾鍘甸梺缁樺灦钃遍柍閿嬪浮閺岋綁顢橀悢宄版懙闂佸搫鏈
double navi_angular_sum = 0.0;   // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾剧懓顪冪€ｎ亝鎹ｉ柣顓炴閵嗘帒顫濋敐鍛婵°倗濮烽崑娑⑺囬悽绋挎瀬鐎广儱顦粈瀣亜閹扳晛鐏ù婊€鍗抽弻锝嗘償閳ュ啿杈
double navi_angular_speed = 0.0; // 闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾剧懓顪冪€ｎ亝鎹ｉ柣顓炴閵嗘帒顫濋敐鍛婵°倗濮烽崑娑⑺囬悽绋挎瀬鐎广儱顦粈瀣亜閹哄秶鍔嶆い鏂挎处缁绘繂顕ラ柨瀣

static void string_analysis(char* buff, double* data, int number_start, int number_total)
{
	//	printf("into string_analysis\n");
	int i = 0;
	int j = 0;
	int k = 0;
	char string_transverse[30];
	//	int *position=(int *)calloc(number_total+1,sizeof(int));
	int position[30];
	while (i < number_total + number_start)
	{
		if ((buff[j] == ',') || (buff[j] == '*'))
		{
			position[i] = j;
			//	printf("position[%d]=%d\n",i,j);
			i++;
		}
		j++;
	}

	for (k = 0; k < number_total; k++)
	{
		if ((position[k + number_start] - position[k + number_start - 1]) == 1)
		{
		}
		else
		{
			strncpy(string_transverse, buff + position[k + number_start - 1] + 1, position[k + number_start] - position[k + number_start - 1] - 1);
			string_transverse[position[k + number_start] - position[k + number_start - 1] - 1] = '\0';
			data[k] = strtod(string_transverse, NULL);
		}
	}
	//	printf("tuichu string_analysis\n");
	return;
}


static void navigator_analysis()
{
	char* str_start;
	char* str_end;
	double navi_data[15];

	if ((str_start = strstr(buff_navigator, "$GPFPD")) != NULL)
	{
		if ((str_end = strstr(str_start, "*")) != NULL)
		{
			string_analysis(str_start, navi_data, 1, 15);
			navigator_data1.GPSWeek = navi_data[0];
			navigator_data1.socond_of_week = navi_data[1];
			navigator_data1.yaw = navi_data[2]-180;
			navigator_data1.pitch = navi_data[3];
			navigator_data1.roll = navi_data[4];
			navigator_data1.latitude = navi_data[5];
			navigator_data1.longitude = navi_data[6];
			navigator_data1.altitude = navi_data[7];
			navigator_data1.East_velocity = navi_data[8];
			navigator_data1.North_velocity = navi_data[9];
			navigator_data1.Virtical_velocity = navi_data[10];
			navigator_data1.Baseline_length = navi_data[11];
			navigator_data1.NSV1 = navi_data[12];
			navigator_data1.NSV2 = navi_data[13];
			navigator_data1.satellite_state = navi_data[14];
			feedback_guandao_data1.GPSV = sqrt(navigator_data1.North_velocity * navigator_data1.North_velocity + navigator_data1.East_velocity * navigator_data1.East_velocity );



			if (fabs(CTRL_DATA_PSI.yd_1[0]-CTRL_DATA_PSI.y_1[0])>=10){
				cheat_flag=false;
			} else if((CTRL_DATA_PSI.yd_1[0]-CTRL_DATA_PSI.y_1[0])*(CTRL_DATA_PSI.yd_1[1]-CTRL_DATA_PSI.y_1[1])<=0){
				cheat_flag=true;
			}
			if ((fabs(CTRL_DATA_PSI.yd_1[0]-CTRL_DATA_PSI.y_1[0])<=10) && HEADING_MODE==0 && CTRL_DATA_PSI.mode==1 && cheat_flag==true){
				navigator_data1.yaw = navigator_data1.yaw+0.2*angle_to_pi(angle_to_pi(CTRL_DATA_PSI.yd_1[0])-angle_to_pi(navigator_data1.yaw));
			}



			if (navigator_data1.yaw > 180 && navigator_data1.yaw <= 360)
			{
				navigator_data1.yaw = navigator_data1.yaw - 360;
			}
			if (first_analysis_flag == 0)
			{
				first_analysis_flag = 1;
				navi_yaw_old = navigator_data1.yaw;
				navi_North_velocity_old = feedback_guandao_data1.GPSV;
				gettimeofday(&tv_navigator, NULL);
				navi_time_old = tv_navigator.tv_sec * 1000 + tv_navigator.tv_usec / 1000;//闂傚倸鍊搁崐鎼佸磹閹间礁纾归柟闂寸绾惧綊鏌熼梻瀵割槮缁惧墽鎳撻—鍐偓锝庝簼閹癸綁鏌ｉ鐐搭棞闁靛棙甯掗～婵嬫晲閸涱剙顥氭繝鐢靛Х椤ｄ粙鍩€椤掆偓绾绢參宕洪敐鍡╂闁绘劕顕晶顒勬煙瀹勭増鍣介柛鐘诧工铻ｉ弶鐐存緲椤ユ岸姊绘担鍛靛綊鎯夋總绋跨；闁绘劕鎼悿鐐節闂堟侗鍎愰柣鎾存礋閹﹢鎮欓幓鎺嗘寖闂佸疇妫勯ˇ鎵崲濞戙垹閱囬柣鏃堫棑娴煎矂姊洪崫鍕効缂傚秳绶氶獮鍐煛閸涱喖娈ラ梺闈涚墕濞层劑鎮伴妷鈺傗拻濞撴埃鍋撴繛浣冲吘娑樼暆閸曨偆锛涢梺鐟板⒔缁垶鍩涢幋锔界厱婵犻潧妫楅鎾煕鎼淬垹濮堝ǎ鍥э躬椤㈡洟濮€閻樿櫕顔掗柣搴㈩問閸犳牠鈥﹂悜钘夌畺闁靛繈鍊曞婵嗏攽閻樻彃顏懖鏍⒒閸屾瑧顦﹂柟璇х節楠炴劙宕卞☉妯虹獩濡炪倖姊婚悺鏃堝疮閸涱垳纾介柛灞捐壘閳ь剚鎮傚畷鎰槹鎼淬埄鍋ㄩ梺璺ㄥ枔婵澹曟繝姘厱闁哄洢鍔岄獮姗€鏌ｉ鐔稿暗闁瑰弶鎮傞幃褔宕煎┑鍫㈡噯闂備浇妫勯崯浼村窗閹邦喗宕叉繛鎴欏灩缁狅絾绻涢崱妤冪闁告梹鎸冲鐑樻姜閹殿噮妲梺鍝ュ枑閹稿啿顕ｆ繝姘╅柕澶堝灪閺傗偓闂備胶纭堕崜婵嬫晝閳轰絼娑㈠Χ閸滀焦瀵岄梺闈涚墕缁绘绮堢€ｎ喗鐓曢柡鍐ｅ亾闁搞劎鏁婚敐鐐剁疀閹句焦妞介、鏃堝礋椤撗冩暪闂備胶顢婇崑鎰偘閵夆晛鐒垫い鎺戝暞閻濐亪鏌ｈ箛鎾搭棦婵﹦绮幏鍛存倻濡儤鐣梻浣规偠閸婃牠鎮ч悩鑽ゅ祦闊洦绋掗弲鎼佹煥閻曞倹瀚�
			}
			else
			{
				gettimeofday(&tv_navigator, NULL);
				navi_time_new = tv_navigator.tv_sec * 1000 + tv_navigator.tv_usec / 1000;
				navigator_data1.angular_velocity = (navigator_data1.yaw - navi_yaw_old) * 1000 / (navi_time_new - navi_time_old);
				navigator_data1.ax = (feedback_guandao_data1.GPSV - navi_North_velocity_old) * 1000 / (navi_time_new - navi_time_old);
				navi_yaw_old = navigator_data1.yaw;
				navi_North_velocity_old = feedback_guandao_data1.GPSV;
				navi_time_old = navi_time_new;
			}

		}
		else
		{
			//printf("Data about the navigator is not complete\n");
		}
	}
	else
	{
		//printf("There is no data about the navigator\n");
	}

	int length_window = 3;  
	if (feedback_guandao_data1.GPSV>2.0){ 
		feedback_guandao_data1.GPSV=navi_velocity[length_window - 1];
	}
	if (navi_velocity_i < length_window)  
	{
		navi_velocity[navi_velocity_i] = feedback_guandao_data1.GPSV;
		navi_velocity_sum += feedback_guandao_data1.GPSV;
		navi_velocity_speed = navi_velocity_sum / (navi_velocity_i + 1);
	}else {
		navi_velocity_sum -= navi_velocity[0];
		int i;
		for (i = 0; i < length_window - 1; i++) {
			navi_velocity[i] = navi_velocity[i + 1];
		}
		navi_velocity[length_window - 1] = feedback_guandao_data1.GPSV;
		navi_velocity_sum += navi_velocity[length_window - 1];
		navi_velocity_speed = navi_velocity_sum / length_window;
	}
	navi_velocity_i++;  


	length_window = 3;  
	if (fabs(navigator_data1.angular_velocity)>35.0){ 
		navigator_data1.angular_velocity=navi_angular[length_window - 1];
	}
	if (navi_angular_i < length_window)  
	{
		navi_angular[navi_angular_i] = navigator_data1.angular_velocity;
		navi_angular_sum += navigator_data1.angular_velocity;
		navi_angular_speed = navi_angular_sum / (navi_angular_i + 1);
	}else { 
		navi_angular_sum -= navi_angular[0];
		int i;
		for (i = 0; i < length_window - 1; i++) {
			navi_angular[i] = navi_angular[i + 1];
		}
		navi_angular[length_window - 1] =navigator_data1.angular_velocity;
		navi_angular_sum += navi_angular[length_window - 1];
		navi_angular_speed = navi_angular_sum / length_window;
	}
	navi_angular_i++;  








	feedback_guandao_data1.GPSV = navi_velocity_speed;
	feedback_guandao_data1.Roll = navigator_data1.roll;
	feedback_guandao_data1.Pitch = navigator_data1.pitch;
	// printf("feedback_guandao_data1.Yaw=%f\n",feedback_guandao_data1.Yaw);
	feedback_guandao_data1.Gyro_z = navi_angular_speed;
	feedback_guandao_data1.longitude = navigator_data1.longitude;
	// printf("feedback_guandao_data1.longitude=%f\n", feedback_guandao_data1.longitude);
	feedback_guandao_data1.Lattitude = navigator_data1.latitude;
	feedback_guandao_data1.Height = navigator_data1.altitude;
	//if(navigator_data1.North_velocity<0)
	//feedback_guandao_data1.GPSV=-navigator_data1.North_velocity;
	feedback_guandao_data1.ax = navigator_data1.ax;
	//feedback_guandao_data1.ax=navigator_data1.Y_ACCEL_OUTPUT;
	feedback_guandao_data1.Yaw = navigator_data1.yaw;
	//feedback_guandao_data1.Yaw=navigator_data1.Z_GYRO_OUTPUT;

}



void navigator()
{
	static struct timeval tv_NV = {0 ,1000};
	switch (select(fd_NV + 1, &rd_NV, NULL, NULL,&tv_NV))//
	{

	case -1:
		perror("select3:");
		printf("NV case -1");
		break;
	case 0:
		printf("NV case 0\n");

		break;
	default:
		//printf("NV case default\n");
		if (FD_ISSET(fd_NV, &rd_NV))
		{
			int temp=0;
			memset(buff_navigator, 0, 1024 * sizeof(char));
			if ((temp=read(fd_NV, buff_navigator, 1024)) < 0)
				perror("read fd_NV:");
			else
			{
				buff_navigator[temp] = '\0';
				//printf("buff_navigator is %s\n",buff_navigator);
				navigator_analysis();
				int first_data_position = 28;
				int nav_a = 1073741824;
				if ((buff_navigator[0] == 0xAA) && (buff_navigator[1] == 0x44) && (buff_navigator[2] == 0x13) && (buff_navigator[3] == 0x28)) 
				{
					//printf("buff_navigator[0] is %x\n",buff_navigator[0]);
					navigator_data1.Z_ACCEL_OUTPUT = (float)((buff_navigator[first_data_position + 3]) * 65536 * 256 + (buff_navigator[first_data_position + 2]) * 65536 + (buff_navigator[first_data_position + 1]) * 256 + (buff_navigator[first_data_position])) * (100) / nav_a * 200;
					navigator_data1.Y_ACCEL_OUTPUT = (float)((buff_navigator[first_data_position + 7]) * 65536 * 256 + (buff_navigator[first_data_position + 6]) * 65536 + (buff_navigator[first_data_position + 5]) * 256 + (buff_navigator[first_data_position + 4])) * (-100) / nav_a * 200;
					navigator_data1.X_ACCEL_OUTPUT = (float)((buff_navigator[first_data_position + 11]) * 65536 * 256 + (buff_navigator[first_data_position + 10]) * 65536 + (buff_navigator[first_data_position + 9]) * 256 + (buff_navigator[first_data_position + 8])) * 100 / nav_a * 200;

					navigator_data1.Z_GYRO_OUTPUT = (float)((buff_navigator[first_data_position + 15]) * 65536 * 256 + (buff_navigator[first_data_position + 14]) * 65536 + (buff_navigator[first_data_position + 13]) * 256 + (buff_navigator[first_data_position + 12])) * 360 / nav_a * 200;
					navigator_data1.Y_GYRO_OUTPUT = (float)((buff_navigator[first_data_position + 19]) * 65536 * 256 + (buff_navigator[first_data_position + 18]) * 65536 + (buff_navigator[first_data_position + 17]) * 256 + (buff_navigator[first_data_position + 16])) * (-360) / nav_a * 200;
					navigator_data1.X_GYRO_OUTPUT = (float)((buff_navigator[first_data_position + 23]) * 65536 * 256 + (buff_navigator[first_data_position + 22]) * 65536 + (buff_navigator[first_data_position + 21]) * 256 + (buff_navigator[first_data_position + 20])) * (360) / nav_a * 200;

					printf("navigator_data1.Z_ACCEL_OUTPUT = %f\n", navigator_data1.Z_ACCEL_OUTPUT);
					printf("navigator_data1.Y_ACCEL_OUTPUT = %f\n", navigator_data1.Y_ACCEL_OUTPUT);
					printf("navigator_data1.X_ACCEL_OUTPUT = %f\n", navigator_data1.X_ACCEL_OUTPUT);
					printf("navigator_data1.Z_GYRO_OUTPUT = %f\n", navigator_data1.Z_GYRO_OUTPUT);
					printf("navigator_data1.Y_GYRO_OUTPUT = %f\n", navigator_data1.Y_GYRO_OUTPUT);
					printf("navigator_data1.X_GYRO_OUTPUT = %f\n", navigator_data1.X_GYRO_OUTPUT);
				}
				//navigator_analysis();
				tcflush(fd_NV, TCIFLUSH);

			}
		}
	}

}

