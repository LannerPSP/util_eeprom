/* standard include file */

//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <sys/io.h>
//#include <time.h>
//#include <stdint.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <dirent.h>
//#include "../include/lmbdrv_ioctl.h"
//#include "../include/iolib.h"
//#include "../include/config.h"
//#include "../include/smbuspath.h"
#define PCH_SMBUS_PORT		0
#define PCH_I2C_PORT0		1
#define PCH_I2C_PORT1		2
#define PCH_I2C_PORT2		3
#define PCH_I2C_PORT3		4
#define PCH_I2C_PORT4		5
#define PCH_I2C_PORT5		6
/*** Return Value ***/
#define RET_Success		0
#define RET_Error		1 	/*(-1)*/
#define RET_NotExist		2	/*(-2)*/
#define RET_NotOpened		3	/*(-3)*/
#define RET_Invalid		4	/*(-4)*/
#define RET_NotSupport		5	/*(-5)*/
#define RET_BusyInUses		6	/*(-6)*/
#define RET_BoardNotMatch	7	/*(-7)*/
#define RET_DriverNoLoad	8	/*(-8)*/

/**** LAN bypass ****/
#define SLOT_SWITCH		(0x73 << 1)	//switch Slot I2C device address
#define SLOT_SWITCHL2		(0x72 << 1)
#define ONBOARD_CONTROLLER	(0x37 << 1)	//OnBoard LanBypass Controller I2C address	
#define SLOT_CONTROLLER		(0x30 << 1)	//Slot LanBypass Controller I2C address

/**** Serial EEPROM I2C Addr ***/
#define ONBOARD_EEPROM		(0x57 << 1)	//OnBoard EEPROM I2C address
#define SLOT_EEPROM	 	(0x56 << 1)	//SLOT EEPROM I2C address
#define EEPROM_MAXSIZE		256
#define EEPROM_SIZE_128KB	0x20000

#define EEPROM_256B		0
#define EEPROM_128KB		1


#define SLOT_OFF	9	//for close SMBus path

#define EEP_SMBUS_PATH	1
#define LBP_SMBUS_PATH	2
#define PSU_SMBUS_PATH	3
	

typedef struct DEF_SMBUS_HWPATH {
	uint8_t	ubLvl1_Addr; 	//0xFF will ignore
	uint8_t ubLvl1_CH;
	uint8_t	ubLvl2_Addr; 	//0xFF will ignore
	uint8_t ubLvl2_CH;
	uint8_t	ubLvl3_Addr; 	//0xFF will ignore
	uint8_t ubLvl3_CH;
}SMBUS_HWPATH;

/*************************************/
/*** Software Reset Button Define  ***/
/*************************************/
#define _NONE		0xFF

#define _OFF	0x00
#define _CH0	0x01
#define _CH1	0x02
#define _CH2	0x04
#define _CH3	0x08
#define _CH4	0x10
#define _CH5	0x20
#define _CH6	0x40
#define _CH7	0x80


SMBUS_HWPATH EepSmbPath[10]= {
{_NONE, _CH2, _NONE, _OFF, _NONE, _OFF }, 	//On-Board
{ 0xE6, _CH0, _NONE, _CH1, _NONE, _OFF },	//Slot-1	
{ 0xE6, _CH1, _NONE, _CH2, _NONE, _OFF },	//Slot-2
{ 0xE6, _CH2, _NONE, _OFF, _NONE, _OFF },	//Slot-3	
{ 0xE6, _CH3, _NONE, _OFF, _NONE, _OFF },	//Slot-4 
{ 0xE6, _CH4, _NONE, _OFF, _NONE, _OFF },	//Slot-5	
{ 0xE6, _CH5, _NONE, _OFF, _NONE, _OFF },	//Slot-6
{ 0xE6, _CH6, _NONE, _OFF, _NONE, _OFF },	//Slot-7	
{ 0xE6, _CH7, _NONE, _OFF, _NONE, _OFF },	//Slot-8
{ 0xE6, _OFF, _NONE, _OFF, _NONE, _OFF }	//Slot-9, OFF uses
};

SMBUS_HWPATH LbpSmbPath[10]= {
{_NONE, _CH2, _NONE, _OFF, _NONE, _OFF }, 	//On-Board
{ 0xE6, _CH0, _NONE, _CH1, _NONE, _OFF },	//Slot-1	
{ 0xE6, _CH1, _NONE, _CH2, _NONE, _OFF },	//Slot-2
{ 0xE6, _CH2, _NONE, _OFF, _NONE, _OFF },	//Slot-3	
{ 0xE6, _CH3, _NONE, _OFF, _NONE, _OFF },	//Slot-4 
{ 0xE6, _CH4, _NONE, _OFF, _NONE, _OFF },	//Slot-5	
{ 0xE6, _CH5, _NONE, _OFF, _NONE, _OFF },	//Slot-6
{ 0xE6, _CH6, _NONE, _OFF, _NONE, _OFF },	//Slot-7	
{ 0xE6, _CH7, _NONE, _OFF, _NONE, _OFF },	//Slot-8
{ 0xE6, _OFF, _NONE, _OFF, _NONE, _OFF }	//Slot-9, OFF uses
};



int fSmbusDevExist = 0;
int fd_smbus=0;
int smbus_adapter_number=-1;
unsigned char buf[1024];

typedef struct DEF_EEPROM_WRINFO {
	uint32_t	udwWrCnt;
	uint32_t	udwStartLoc;
	uint32_t	udwLength;
}EEPROM_WRINFO;


/*** add for PCH I2C controller ***/
#define	I2C_PORT_MAX	6
int fI2cDevExist[I2C_PORT_MAX] = {0, 0,0,0, 0, 0 };
int fd_i2c[I2C_PORT_MAX]={0, 0, 0, 0, 0, 0};
int i2c_adapter_number[I2C_PORT_MAX]={-1, -1, -1, -1, -1, -1};

//#define ONBOARD_EEP_ADDR	0x53
//#define MAX_FILE_SIZE 1024*2
//#define MAX_EEP_SIZE 256

int32_t __attribute__ ((constructor)) _lmbsmb_initial(void);
void __attribute__ ((destructor)) _lmbsmb_exit(void);
/**********************************************/
/****   Detect Intel PCH I2C conroller ********/
/**********************************************/
int detect_i2c_busses(void)
{

    char sysfs[1024], n[1024], s[120], s_best[120];
    DIR *dir;
    struct dirent *de, *de_best=NULL;
    FILE *f;
    int bus_count=0, guess_bus=0, index=0;

    sprintf(sysfs,"/sys/class/i2c-adapter");
    if (!(dir = opendir(sysfs))) return guess_bus;
    while ((de = readdir(dir)) !=NULL) {
        if (!strcmp(de->d_name, ".")) continue;
        if (!strcmp(de->d_name, ".."))continue;
        //snprintf(n, sizeof(n),"%s/%s/name", sysfs, de->d_name); ubuntu build failure
	strcpy(n, sysfs);
	strcat(n , "/");
	strcat(n, de->d_name);
	strcat(n , "/name");
        f = fopen(n, "r");
        if (f == NULL) continue;
        fgets(s, 120, f);
        fclose(f);

        if ( strstr(s, "Synopsys")){
            de_best = de;
            strcpy(s_best, s);
            sscanf(de_best->d_name, "i2c-%d", &guess_bus);
            i2c_adapter_number[index++]=guess_bus;
//printf("s=%s, de_best->d_name=%s, guess_bus=%d\n",s, de_best->d_name, guess_bus);
        }
        bus_count++;
    }

    return guess_bus;
}
/***/
int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

/********************/
void open_i2c_adapter()
{
	int fd;
	char str[20];
	int xi;

	/* try to probe how many i2c bus on system
         * Show warning message if multi adapter presented and no "-a" argument input
         */
	if (i2c_adapter_number[0] == -1) detect_i2c_busses();
	//    i2c_adapter_number[index] = detect_i2c_busses();

	qsort(i2c_adapter_number, 4, sizeof(int), cmpfunc);


	/* 1. check smbus adapter available or not */
//	sprintf(str,"/dev/i2c-%d", i2c_adapter_number[0]);
//printf("I2C-0 path = %s\n", str);
//	sprintf(str,"/dev/i2c-%d", i2c_adapter_number[1]);
//printf("I2C-1 path = %s\n", str);

	for ( xi=0 ; xi<I2C_PORT_MAX ; xi++ ) {
		if ( i2c_adapter_number[xi] != -1 ) {
			sprintf(str,"/dev/i2c-%d", i2c_adapter_number[xi]);
			fd = open(str, O_RDWR);
			if (fd < 0 && (errno == ENOENT || errno == ENOTDIR)) {
				sprintf(str,"/dev/i2c/%d", i2c_adapter_number[xi]);
				fd = open(str, O_RDWR);
			}
			if(fd > 0) {
				fI2cDevExist[xi] = 1;
				fd_i2c[xi]=fd;
//printf("open i2c-%d, path=%s OK\n", xi, str);
			}
		}	
	}
	return ;
}

/***********************************************************/
/**** I2C-DEV functions  ***********************************/
/***********************************************************/
int detect_smbus_busses(void)
{

    char sysfs[1024], n[1024], s[120], s_best[120];
    DIR *dir;
    struct dirent *de, *de_best=NULL;
    FILE *f;
    int bus_count=0, guess_bus=0;

    sprintf(sysfs,"/sys/class/i2c-adapter");
    if (!(dir = opendir(sysfs))) return guess_bus;
    while ((de = readdir(dir)) !=NULL) {
        if (!strcmp(de->d_name, ".")) continue;
        if (!strcmp(de->d_name, ".."))continue;
        //snprintf(n, sizeof(n),"%s/%s/name", sysfs, de->d_name); ubuntu build failure
	strcpy(n, sysfs);
	strcat(n , "/");
	strcat(n, de->d_name);
	strcat(n , "/name");
        f = fopen(n, "r");
        if (f == NULL) continue;
        fgets(s, 120, f);
        fclose(f);

        if ( strstr(s, "801") ){
            de_best = de;
            strcpy(s_best, s);
            sscanf(de_best->d_name, "i2c-%d", &guess_bus);
        }
	else if  (strstr(s, "PIIX") ){
            de_best = de;
            strcpy(s_best, s);
            sscanf(de_best->d_name, "i2c-%d", &guess_bus);
	    break; //only get 1st smbus device
        }
        bus_count++;
    }

    return guess_bus;
}
/********************/
int open_smbus_adapter(void)
{
	int fd;
	char str[20];

	/* try to probe how many i2c bus on system
         * Show warning message if multi adapter presented and no "-a" argument input
         */
	if (smbus_adapter_number < 0)
	    smbus_adapter_number = detect_smbus_busses();

	/* 1. check smbus adapter available or not */
	sprintf(str,"/dev/i2c-%d", smbus_adapter_number);
	fd = open(str, O_RDWR);
	if (fd < 0 && (errno == ENOENT || errno == ENOTDIR)) {
		sprintf(str,"/dev/i2c/%d", smbus_adapter_number);
		fd = open(str, O_RDWR);
	}
	if(fd < 0)return -1;
	return fd;
}

__s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
	int i, ret;
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

         //To implement failure_limitation counter in order to retry when smbus is busy
	for(i=0 ; i<5; i++){
	    if( (ret=ioctl(file,I2C_SMBUS,&args)) ){
	        if(i != 4 && ret==-1){
                    // delay between 1 and 30ms.
	            srand(getpid());
	            ret = rand()%30000;
                    usleep(ret + 1000);
                }
	    }
	    else break;
	}
	return ret;
}

static inline __s32 i2c_smbus_write_quick(int file, __u8 value)
{
	return i2c_smbus_access(file,value,0,I2C_SMBUS_QUICK,NULL);
}
static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command,
                                              __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BYTE_DATA, &data);
}
static inline __s32 i2c_smbus_write_word_data(int file, __u8 command,
                                              __u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_WORD_DATA, &data);
}

inline __s32 i2c_smbus_write_byte(int file, __u8 value)
{
	return i2c_smbus_access(file,I2C_SMBUS_WRITE,value,
	                        I2C_SMBUS_BYTE,NULL);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file,
                         I2C_SMBUS_READ,
                         command,
                         I2C_SMBUS_BYTE_DATA,
                         &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_read_byte(int file)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file,
                         I2C_SMBUS_READ,
                         0,
                         I2C_SMBUS_BYTE,
                         &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file,
                         I2C_SMBUS_READ,
                         command,
                         I2C_SMBUS_WORD_DATA,
                         &data))
        return -1;
    else
        return 0x0FFFF & data.word;
}


static inline __s32 i2c_smbus_read_block_data(int file, __u8 command,
                                              __u8 *values)
{
    union i2c_smbus_data data;
    int i;
    if (i2c_smbus_access(file,
                         I2C_SMBUS_READ,
                         command,
                         I2C_SMBUS_BLOCK_DATA,
                         &data))
        return -1;
    else {
        for (i = 1; i <= data.block[0]; i++)
            values[i-1] = data.block[i];
        return data.block[0];
    }
}

/************************************/
static inline __s32 i2c_smbus_write_i2c_block_data(int file, __u8 command, 
                                                   __u8 length, 
                                                   const __u8 *values) 
{ 
    union i2c_smbus_data data; 
    int i; 
    if (length > 32) 
        length = 32; 
    for (i = 1; i <= length; i++) 
        data.block[i] = values[i-1]; 
    data.block[0] = length; 
    return i2c_smbus_access(file,I2C_SMBUS_WRITE,command, 
                            I2C_SMBUS_I2C_BLOCK_BROKEN, &data); 
}


/************************************************/
/**** Library Load Initial Here *****************/
/************************************************/
/***/
int32_t _lmbsmb_initial(void)
{
int32_t iret = RET_Success;
	fd_smbus = open_smbus_adapter();
	if ( fd_smbus > 0 ) {
		fSmbusDevExist = 1;
		ioctl(fd_smbus, I2C_TIMEOUT,1);
	}
	else 	{
		fSmbusDevExist = 0;
		iret = -RET_DriverNoLoad;
	}

	open_i2c_adapter();
	return iret;
}
/****/
void _lmbsmb_exit(void)
{
int xi;
	if(fd_smbus > 0 ) close(fd_smbus);
	for (xi=0 ; xi<I2C_PORT_MAX ; xi++) {
		if ( fd_i2c[xi] > 0 ) close(fd_i2c[xi]);
	}

}
/*******************/
int32_t _lmbsmb_i2c_read_byte(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubReg, uint8_t* pubData)
{
int32_t iret;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}

	//set slave address
	if(ioctl(fd_temp, I2C_SLAVE, ubAddr>>1) < 0) return -RET_Error;
	//get byte data
	iret =i2c_smbus_read_byte_data(fd_temp, ubReg);
	if ( iret < 0 ) return -RET_Error;
	*pubData =(uint8_t) (iret & 0xFF);
	
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_i2c_read_word(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubReg, uint16_t* puwData)
{
int32_t iret;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	//set slave address
	if(ioctl(fd_temp, I2C_SLAVE, ubAddr>>1) < 0) return -RET_Error;
	//get word data
	iret = i2c_smbus_read_word_data(fd_temp, ubReg);
	if ( iret < 0 ) return -RET_Error;
	*puwData = (uint16_t)(iret & 0xFFFF);
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_i2c_read_block(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubReg, uint8_t* pubBlock)
{
int32_t iret,xi;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	//set slave address
	if(ioctl(fd_temp, I2C_SLAVE, ubAddr>>1) < 0) return -RET_Error;
	//get block data
	iret = i2c_smbus_read_block_data(fd_temp, ubReg, buf);
	if ( iret < 0 ) return -RET_Error;
	for ( xi=0 ; xi<iret ; xi++) 	*pubBlock++ = buf[xi]; 
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_i2c_write_byte(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubReg, uint8_t ubData)
{
int32_t iret;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	//set slave address
	if(ioctl(fd_temp, I2C_SLAVE, ubAddr>>1) < 0) return -RET_Error;
	//write byte data
	iret =i2c_smbus_write_byte_data(fd_temp, ubReg, ubData);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_i2c_write_word(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubReg, uint16_t uwData)
{
int32_t iret;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	//set slave address
	if(ioctl(fd_temp, I2C_SLAVE, ubAddr>>1) < 0) return -RET_Error;
	//write byte data
	iret = i2c_smbus_write_word_data(fd_temp, ubReg, uwData);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_switch_slot(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubSlot)
{
uint8_t ubData=0;
int32_t iret;
uint8_t ubReg;

	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	if ( ubSlot != 0 )	ubReg = 0x01 << (ubSlot-1);
	else 			ubReg = 0;
	iret = _lmbsmb_i2c_read_byte(bPortIndex, ubAddr, ubReg, &ubData);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_switch_ch(int8_t bPortIndex, uint8_t ubAddr, uint8_t ubChannel)
{
uint8_t ubData=0;
int32_t iret;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	iret = _lmbsmb_i2c_read_byte(bPortIndex, ubAddr, ubChannel, &ubData);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
}
/*******************/
int32_t _lmbsmb_switch_path(int8_t bPortIndex, uint8_t ubType, uint8_t ubSlot)
{
SMBUS_HWPATH *bptr;
int32_t iret;

	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		break;
	default:
		return -RET_Invalid; 
		break;
	}

	if 	( ubType == EEP_SMBUS_PATH ) bptr = &EepSmbPath[ubSlot];
	else if ( ubType == LBP_SMBUS_PATH ) bptr = &LbpSmbPath[ubSlot];
	else 				     return -RET_Error;

	//check Level-1
	if ( bptr->ubLvl1_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl1_Addr, bptr->ubLvl1_CH);
	if ( iret < 0 ) return -RET_Error;

	//check Level-2
	if ( bptr->ubLvl2_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl2_Addr, bptr->ubLvl2_CH);
	if ( iret < 0 ) return -RET_Error;

	//check Level-3
	if ( bptr->ubLvl3_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl3_Addr, bptr->ubLvl3_CH);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
	
}
/*******************/
int32_t _lmbsmb_switch_pathOFF(int8_t bPortIndex, uint8_t ubType, uint8_t ubSlot)
{
SMBUS_HWPATH *bptr;
int32_t iret;

	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		break;
	default:
		return -RET_Invalid; 
		break;
	}

	if 	( ubType == EEP_SMBUS_PATH ) bptr = &EepSmbPath[ubSlot];
	else if ( ubType == LBP_SMBUS_PATH ) bptr = &LbpSmbPath[ubSlot];
	else 				     return -RET_Error;

	//check Level-1
	if ( bptr->ubLvl1_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl1_Addr, _OFF);
	if ( iret < 0 ) return -RET_Error;

	//check Level-2
	if ( bptr->ubLvl2_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl2_Addr, _OFF);
	if ( iret < 0 ) return -RET_Error;

	//check Level-3
	if ( bptr->ubLvl3_Addr == _NONE ) return RET_Success; 
	iret = _lmbsmb_switch_ch(bPortIndex, bptr->ubLvl3_Addr, _OFF);
	if ( iret < 0 ) return -RET_Error;
	return RET_Success;
}
/************************/
int32_t _lmbsmb_onboard_eeprom_type(int8_t bPortIndex, int8_t* pbType)
{
uint8_t ubDevAddr;
int32_t iret;
int fd_temp = 0;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}

	_lmbsmb_switch_path(bPortIndex, EEP_SMBUS_PATH, 0);
	ubDevAddr = (uint8_t)(ONBOARD_EEPROM >>1) & 0xFB;
	iret = ioctl(fd_temp, I2C_SLAVE, ubDevAddr) ;
	iret = i2c_smbus_read_byte(fd_temp);
	if(iret < 0) 	*pbType = EEPROM_256B;
	else 		*pbType = EEPROM_128KB;
	_lmbsmb_switch_pathOFF(bPortIndex, EEP_SMBUS_PATH, 0);
	return RET_Success;
}
/************************/
int32_t _lmbsmb_write_eepEX(int8_t bPortIndex, uint32_t udwEepAddr, uint32_t uwLength, uint8_t* pubData)
{
int32_t ret, xi, xloop;
EEPROM_WRINFO eep_wrinfo[6];
uint32_t xdwAddr = udwEepAddr, udwTmpAddr ;
//uint32_t xLength = uwLength;
uint32_t xLen1=0, xLen2=0, xLen3=0,xTempLen;
uint8_t xbAddr;
uint8_t xCmd, aryData[32];
uint8_t *bptr;
int fd_temp = 0;
	
	if ( dwEEP_type == 0 )  return 	-RET_Invalid;

	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}
	if  ( (udwEepAddr + (uint32_t)uwLength ) > aryEEPROM_SIZE[dwEEP_type] ) {
		//printf("\e[1;31mWrire address out of range !!!\n\e[m");
		return -RET_Invalid;
	} 

	if ( ((xdwAddr & 0xF ) + uwLength ) > 0x10 ) {
		xLen1= 0x10 - (xdwAddr & 0xF) ;
		xTempLen = uwLength - xLen1;
	}
	else  {
		xLen1 = uwLength;
		xLen2 =0;
		xLen3 =0;
		xTempLen =0;
	}
	
	if ( xTempLen >= 16 ) {
		xLen2 = xTempLen & 0xFFFF0 ;
		if ( xLen2 > 0 ) {
			xLen3 =  xTempLen & 0xF ;
		}
	}
	else if (xTempLen>0 )  {
		xLen2 = xTempLen ; 
		xLen3 =0;
	}
	else  xLen2=0;
	
	if ( xLen1 != 0 ) {
		eep_wrinfo[0].udwWrCnt = 1 ;
		eep_wrinfo[0].udwStartLoc = xdwAddr ;
		eep_wrinfo[0].udwLength = xLen1 ;
	}

	if ( xLen2 > 0x10 ) {
		eep_wrinfo[1].udwWrCnt = xLen2 / 0x10 ;
		eep_wrinfo[1].udwStartLoc = xdwAddr + xLen1 ;
		eep_wrinfo[1].udwLength = 0x10 ;
		
	}	
	else if ( xLen2 > 0 ) {
		eep_wrinfo[1].udwWrCnt = 1;
		eep_wrinfo[1].udwStartLoc = xdwAddr + xLen1 ;
		eep_wrinfo[1].udwLength = xLen2 ;
		
	}	
	else 	eep_wrinfo[1].udwWrCnt = 0 ;

	
	if ( xLen3 != 0 ) {
		eep_wrinfo[2].udwWrCnt = 1;
		eep_wrinfo[2].udwStartLoc = xdwAddr + xLen1 + xLen2 ;
		eep_wrinfo[2].udwLength = xLen3 ;
		
	}	
	else eep_wrinfo[2].udwWrCnt = 0 ;

	
	bptr = pubData;
	if ( eep_wrinfo[0].udwWrCnt != 0 ) {
		//check DevAdr 
		if 	( eep_wrinfo[0].udwStartLoc >= 0x10000 ) xbAddr = (uint8_t)(ubDevAddr >>1) ; 		//High Bank
		else						 xbAddr = (uint8_t)(ubDevAddr >>1) & 0xFB;	//Low Bank
		
		//read data to write array
		xCmd = (uint8_t)((eep_wrinfo[0].udwStartLoc >> 8 ) & 0xFF );
		aryData[0] = (uint8_t)(eep_wrinfo[0].udwStartLoc & 0xFF );
		for (xi=1 ; xi<=eep_wrinfo[0].udwLength ;xi++ ) {
			aryData[xi] = *bptr++;
		}

		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //      ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}
		ret = i2c_smbus_write_i2c_block_data(fd_temp, xCmd, eep_wrinfo[0].udwLength+1, aryData) ;
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_i2c_block_data  \e[m\n"); 
			return -RET_Error;
		}
	
	}

	udwTmpAddr = eep_wrinfo[1].udwStartLoc;
	if ( eep_wrinfo[1].udwWrCnt > 0 ) {
	    for ( xloop=0 ; xloop<eep_wrinfo[1].udwWrCnt ; xloop++) {
		//check DevAdr 
		if 	( udwTmpAddr >= 0x10000 ) xbAddr = (uint8_t)(ubDevAddr >>1) ; 		//High Bank
		else				  xbAddr = (uint8_t)(ubDevAddr >>1) & 0xFB;	//Low Bank
		
		//read data to write array
		xCmd = (uint8_t)((udwTmpAddr >> 8 ) & 0xFF );
		aryData[0] = (uint8_t)(udwTmpAddr & 0xFF );
		for (xi=1 ; xi<=eep_wrinfo[1].udwLength ;xi++ ) {
			aryData[xi] = *bptr++;
		}

		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //      ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}
		ret = i2c_smbus_write_i2c_block_data(fd_temp, xCmd, eep_wrinfo[1].udwLength+1, aryData) ;
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_i2c_block_data  \e[m\n"); 
			return -RET_Error;
		}
		udwTmpAddr += eep_wrinfo[1].udwLength;
	   }
	
	}
	
	if ( eep_wrinfo[2].udwWrCnt != 0 ) {
		//check DevAdr 
		if 	( eep_wrinfo[0].udwStartLoc >= 0x10000 ) xbAddr = (uint8_t)(ubDevAddr >>1) ; 		//High Bank
		else						 xbAddr = (uint8_t)(ubDevAddr >>1) & 0xFB;	//Low Bank
		
		//read data to write array
		xCmd = (uint8_t)((eep_wrinfo[2].udwStartLoc >> 8 ) & 0xFF );
		aryData[0] = (uint8_t)(eep_wrinfo[2].udwStartLoc & 0xFF );
		for (xi=1 ; xi<=eep_wrinfo[2].udwLength ;xi++ ) {
			aryData[xi] = *bptr++;
		}

		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //    ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}
		ret = i2c_smbus_write_i2c_block_data(fd_temp, xCmd, eep_wrinfo[2].udwLength+1, aryData) ;
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_i2c_block_data  \e[m\n"); 
			return -RET_Error;
		}
	
	}

	return RET_Success;
} 

int32_t _lmbsmb_read_eepEX(int8_t bPortIndex, uint32_t udwEepAddr, uint32_t udwLength, uint8_t* pubData)
{
int32_t ret, xi, xloop;
EEPROM_WRINFO eep_rdinfo[6];
uint32_t xdwAddr = udwEepAddr, udwTmpAddr ;
//uint32_t xLength = uwLength;
uint32_t xLen1=0, xLen2=0, xLen3=0,xTempLen;
uint8_t xbAddr;
uint8_t xCmd, xReg;// aryData[32];
uint8_t *bptr;
int fd_temp = 0;

	if ( dwEEP_type == 0 )  return 	-RET_Invalid;
	switch (bPortIndex) {
	case PCH_SMBUS_PORT:
		if ( !fSmbusDevExist ) return -RET_DriverNoLoad;
		fd_temp = fd_smbus;
		break;
	case PCH_I2C_PORT0:
	case PCH_I2C_PORT1:
	case PCH_I2C_PORT2:
	case PCH_I2C_PORT3:
	case PCH_I2C_PORT4:
	case PCH_I2C_PORT5:
		if ( !fI2cDevExist[bPortIndex-1] ) return -RET_DriverNoLoad;
		fd_temp = fd_i2c[bPortIndex-1];
		break;
	default:
		return -RET_Invalid; 
		break;
	}

	if  ( (udwEepAddr + (uint32_t)udwLength ) > aryEEPROM_SIZE[dwEEP_type] ) {
		//printf("\e[1;31mWrire address out of range !!!\n\e[m");
		return -RET_Invalid;
	} 

	if ( ((xdwAddr & 0xF ) + udwLength ) > 0x10 ) {
		xLen1= 0x10 - (xdwAddr & 0xF) ;
		xTempLen = udwLength - xLen1;
	}
	else  {
		xLen1 = udwLength;
		xLen2 =0;
		xLen3 =0;
		xTempLen =0;
	}
	
	if ( xTempLen >= 16 ) {
		xLen2 = xTempLen & 0xFFFF0 ;
		if ( xLen2 > 0 ) {
			xLen3 =  xTempLen & 0xF ;
		}
	}
	else if (xTempLen>0 )  {
		xLen2 = xTempLen ; 
		xLen3 =0;
	}
	else  xLen2=0;
	
	
	if ( xLen1 != 0 ) {
		eep_rdinfo[0].udwWrCnt = 1 ;
		eep_rdinfo[0].udwStartLoc = xdwAddr ;
		eep_rdinfo[0].udwLength = xLen1 ;
	}

	if ( xLen2 > 0x10 ) {
		eep_rdinfo[1].udwWrCnt = xLen2 / 0x10 ;
		eep_rdinfo[1].udwStartLoc = xdwAddr + xLen1 ;
		eep_rdinfo[1].udwLength = 0x10 ;
		
	}	
	else if ( xLen2 > 0 ) {
		eep_rdinfo[1].udwWrCnt = 1;
		eep_rdinfo[1].udwStartLoc = xdwAddr + xLen1 ;
		eep_rdinfo[1].udwLength = xLen2 ;
		
	}	
	else 	eep_rdinfo[1].udwWrCnt = 0 ;

	
	if ( xLen3 != 0 ) {
		eep_rdinfo[2].udwWrCnt = 1;
		eep_rdinfo[2].udwStartLoc = xdwAddr + xLen1 + xLen2 ;
		eep_rdinfo[2].udwLength = xLen3 ;
		
	}	
	else eep_rdinfo[2].udwWrCnt = 0 ;

	bptr = pubData;
	if ( eep_rdinfo[0].udwWrCnt != 0 ) {
		//check DevAdr 
		if 	( eep_rdinfo[0].udwStartLoc >= 0x10000 ) xbAddr = (uint8_t)ubDevAddr >>1 ; 		//High Bank
		else						 xbAddr = ((uint8_t)ubDevAddr >>1) & 0xFB;	//Low Bank

		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //      ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}
		//set Location address
		xCmd = (uint8_t)((eep_rdinfo[0].udwStartLoc >> 8 ) & 0xFF );
		xReg = (uint8_t)(eep_rdinfo[0].udwStartLoc & 0xFF );
		ret = i2c_smbus_write_byte_data(fd_temp, xCmd, xReg);	
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_byte_data ret=%d \e[m\n", ret); 
			return -RET_Error;
		}
		//read data 
		for ( xi=0 ; xi<eep_rdinfo[0].udwLength ; xi++ ) {
			ret = i2c_smbus_read_byte(fd_temp);
			if ( ret < 0 ) { 
				//printf("\e[0;31m<ALARM> i2c_smbus_read_byte ret=%d \e[m\n", ret); 
				return -RET_Error;
			}
			else 		*bptr++ = (uint8_t)(ret & 0xFF);
		}

	}

	udwTmpAddr = eep_rdinfo[1].udwStartLoc;
	if ( eep_rdinfo[1].udwWrCnt > 0 ) {
	    for ( xloop=0 ; xloop<eep_rdinfo[1].udwWrCnt ; xloop++) {
		//check DevAdr 
		if 	( udwTmpAddr >= 0x10000 ) xbAddr = (uint8_t)(ubDevAddr >>1) ; 		//High Bank
		else						 xbAddr = (uint8_t)(ubDevAddr >>1) & 0xFB;	//Low Bank
		
		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //      ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}

		//set Location address
		xCmd = (uint8_t)((udwTmpAddr >> 8 ) & 0xFF );
		xReg = (uint8_t)(udwTmpAddr & 0xFF );
		ret = i2c_smbus_write_byte_data(fd_temp, xCmd, xReg);	
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_byte_data ret=%d \e[m\n", ret); 
			return -RET_Error;
		}
		//read data 
		for ( xi=0 ; xi<eep_rdinfo[1].udwLength ; xi++ ) {	
			ret = i2c_smbus_read_byte(fd_temp);
			if ( ret < 0 ) { 
				//printf("\e[0;31m<ALARM> i2c_smbus_read_byte ret=%d \e[m\n", ret); 
				return -RET_Error;
			}
			else 		*bptr++ = (uint8_t)(ret & 0xFF);
		}
		udwTmpAddr += eep_rdinfo[1].udwLength;
	   }
	
	}
	
	if ( eep_rdinfo[2].udwWrCnt != 0 ) {
		//check DevAdr 
		if 	( eep_rdinfo[0].udwStartLoc >= 0x10000 ) xbAddr = (uint8_t)(ubDevAddr >>1) ; 		//High Bank
		else						 xbAddr = (uint8_t)(ubDevAddr >>1) & 0xFB;	//Low Bank
		
		//set slave address
		if(ioctl(fd_temp, I2C_SLAVE, xbAddr) < 0){
	                //printf("\e[0;31m<ALARM> cannot set i2c address to 0x%02x. errno %d: %s\e[m\n",
         	        //      ubDevAddr, errno, strerror(errno));
			return -RET_Error;
		}
		//set Location address
		xCmd = (uint8_t)((eep_rdinfo[2].udwStartLoc >> 8 ) & 0xFF );
		xReg = (uint8_t)(eep_rdinfo[2].udwStartLoc & 0xFF );
		ret = i2c_smbus_write_byte_data(fd_temp, xCmd, xReg);	
		if ( ret < 0 ) { 
			//printf("\e[0;31m<ALARM> i2c_smbus_write_byte_data ret=%d \e[m\n", ret); 
			return -RET_Error;
		}
		//read data 
		for ( xi=0 ; xi<eep_rdinfo[2].udwLength ; xi++ ) {
			ret = i2c_smbus_read_byte(fd_temp);
			if ( ret < 0 ) { 
				//printf("\e[0;31m<ALARM> i2c_smbus_read_byte ret=%d \e[m\n", ret); 
				return -RET_Error;
			}
			else 		*bptr++ = (uint8_t)(ret & 0xFF);
		}
	
	}


	return RET_Success;
} 


