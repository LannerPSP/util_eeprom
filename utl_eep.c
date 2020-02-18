
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <ctype.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define TYPE_256B	0
#define TYPE_512B	1
#define TYPE_1KB	2
#define TYPE_2KB	3
#define TYPE_4KB	4
#define TYPE_8KB	5
#define TYPE_16KB	6
#define TYPE_32KB	7
#define TYPE_64KB	8
#define TYPE_128KB	9



uint8_t ubDevAddr=0xAE;
int dwEEP_type=0;
int32_t aryEEPROM_SIZE[]={256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072};
#include "smblib.c"
int8_t	bDevPort=PCH_SMBUS_PORT;

uint8_t aryEEPROM[131072];
#define DEF_ERROR_MAX	3 

int _eep_to_file(uint8_t ubAddr, int32_t dwLength);
int __read_string_disp(uint8_t ubAddr);
int _file_to_eep(void);

int ascii_to_hex(char ch) 
{ 
char ch_tmp; 
int hex_val = -1; 

	ch_tmp = tolower(ch); 

	if ((ch_tmp >= '0') && (ch_tmp <= '9')) { 
		hex_val = ch_tmp - '0'; 
	} 
	else if ((ch_tmp >= 'a') && (ch_tmp <= 'f')) { 
		hex_val = ch_tmp - 'a' + 10; 
	} 

	return hex_val; 
} 
/********/
int str_to_hex(char *hex_str) 
{
int i, len; 
int hex_tmp, hex_val; 
char *bptr;
int fHEX=0;
	bptr = strstr(hex_str, "0x");
	if ( bptr != NULL ) {
		bptr+=2;
		fHEX=1;
	}
	else 	bptr=hex_str;

	if (fHEX==1 ){
		len = (int)strlen(bptr); 
		hex_val = 0; 
	 	for (i=0; i<len;i++) { 
			hex_tmp = ascii_to_hex(bptr[i]); 
			if (hex_tmp == -1) { return -1; } 
			hex_val = (hex_val) * 16 + hex_tmp; 
		}
	}
	else {
		hex_val = atoi(bptr);
	} 
	return hex_val; 
} 
void __printf_usage(char *argv0)
{
	printf("Usage: %s -switch   [-i2cport #num] -addr #hex -channel #num\n", argv0);
	printf("			\e[1;34m--> setting SMBus switch channel 0~7, #num=8 is channel off\e[m\n");
	printf("       %s -eep2file [-i2cport #num] [-type #num] [-ad-dr #num] [-start #num] [-len #num] [-file fielname]\n", argv0);
	printf("			\e[1;34m--> save eeprom data to file\e[m\n");
	printf("       %s -eep2disp [-i2cport #num] [-type #num] [-addr #num] [-start #num] [-len #num]\n", argv0);
	printf("			\e[1;34m---> show eeprom data to screen\e[m\n");
	printf("       %s -file2eep [-i2cport #num] [-type #num] [-addr #num] [-file fielname]\n", argv0);
	printf("			\e[1;34m---> write EEPROM from file\e[m\n");
	printf("       %s -writeasc [-i2cport #num] [-type #num] [-addr #num] [-start #num] -string \"ASCII_string\"\n", argv0);
	printf("			\e[1;34m---> write ASCII to EEPROM with NULL\e[m\n");
	printf("       %s -writebin [-i2cport #num] [-type #num] [-addr #num] [-start #num] -string \"BCD_string\"\n", argv0);
	printf("			\e[1;34m---> write Binary to EEPROM without NULL\e[m\n");
	//printf("       %s -compare  [-i2cport #num] [-type #num] [-addr #num] [-file fielname] [-start #num] [-len #num]\n", argv0);
	//printf("			\e[1;34m---> compare EEPROM with file\e[m\n");
	printf("       %s -erase    [-i2cport #num] [-type #num] [-addr #num] [-value #num] [-start #num]\n", argv0);
	printf("			\e[1;34m---> erase EEPROM 256B/page with 0x100 boundary, default: value is 0x00\e[m\n");
	printf("       %s -getstr   [-i2cport #num] [-type #num] [-addr #num] [-start #num]\n", argv0);
	printf("			\e[1;34m---> read string from EEPROM and display to scrren\e[m\n");
	printf("	parameter:\n");
	printf("	-i2cport #num	: uess I2C device port #num (1~6), default: SMBus port (0)\n");
	printf("	-addr #num	: assign device address (8bit mode), default: 0xAE\n");
	printf("	        	: #num with \"0x\" is hex-decimal input, other is decimal\n");
	printf("	-channel #num	: assign channel value (0~8)\n");
	printf("	-start #num	: assign EEPROM strating location, default: 0x0\n");
	printf("	-len #num	: assign length, max. 65536 Bytes\n");
	printf("	-type #num	: 0:256B, 1:512B, 2:1KB, 3:2KB, 4:4KB, 5:8KB, 6:16KB, 7:32KB, 8:64KB, 9:128KB, default: type 0\n");
}


void _err_printf(char * pbStirng)
{
	printf("\e[1;31m%s\e[m\n",pbStirng);

}
void __error_exit(char *argv0)
{
	printf("\e[1;31m<Error> comamnd error !!!\e[m\n");
	__printf_usage(argv0);
	exit(-1);
}

#define ITEM_BUSSWITCH	1
#define ITEM_EEP2FILE	2
#define ITEM_FILE2EEP	3
#define ITEM_EEP2DISP	4
#define ITEM_WRITEASC	5
#define ITEM_WRITEBIN	6
#define ITEM_COMPARE	7
#define ITEM_ERASE	8
#define ITEM_GETSTRING	9
int dwStart=0;
int fActionType=0;
char CONFIG_NAME[]= "eepdata.txt";
char arybWriteData[256];


int main(int argc, char **argv) 
{
int iRet =0, xi,xj, iType=-1;
int dwData=0x00;
int32_t xCnt =3, iret;
uint8_t ubReg=0xFF, aryuData[2], ubData, tmpBuf[3], *bptr, *bptw;
int dwLength =0,iBlockLen;


	if ( getuid() != 0 ) {
		_err_printf("<Warning> Please uses root user !!!");
		return -1;
	}
	memset(arybWriteData, 0, 256); //initial null 
	for ( xi= 1; xi< argc ; xi++ ) {
		if 	( strcmp("-switch", argv[xi]) == 0 ) iType = ITEM_BUSSWITCH;
		else if ( strcmp("-eep2file", argv[xi]) == 0 ) {
			iType = ITEM_EEP2FILE;
			dwLength=aryEEPROM_SIZE[dwEEP_type];
		}
		else if ( strcmp("-eep2disp", argv[xi]) == 0 ) {
			iType = ITEM_EEP2DISP; 
			dwLength=aryEEPROM_SIZE[dwEEP_type];
		}
		else if ( strcmp("-writeasc", argv[xi]) == 0 ) iType = ITEM_WRITEASC;
		else if ( strcmp("-writebin", argv[xi]) == 0 ) iType = ITEM_WRITEBIN;
		else if ( strcmp("-file2eep", argv[xi]) == 0 ) iType = ITEM_FILE2EEP;
		else if ( strcmp("-getstr", argv[xi]) == 0 ) iType = ITEM_GETSTRING;
		else if ( strcmp("-erase", argv[xi]) == 0 ) {
			iType = ITEM_ERASE;
			dwLength = 256;
		}
		else if ( strcmp("-compare", argv[xi]) == 0 ) {
			iType = ITEM_COMPARE;
			dwStart =0;
			dwLength = aryEEPROM_SIZE[dwEEP_type];
		}
		else if ( strcmp("-type", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input EEPROM type");
				return -1;
			}
			dwEEP_type = str_to_hex(argv[xi+1]);
			if ( dwEEP_type > TYPE_128KB ) {
				_err_printf("<Error> EEPROM type out of range 0 ~ 9");
				return -1;
			}
			xi++;
		}
		else if ( strcmp("-i2cport", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input port number");
				return -1;
			}
			bDevPort = str_to_hex(argv[xi+1]);
			if ( bDevPort > 6 ) {
				_err_printf("<Error> i2C Port out of range 0 ~ 6");
				return -1;
			}
			xi++;
		}
		else if ( strcmp("-len", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input length");
				return -1;
			}
			dwLength = str_to_hex(argv[xi+1]);
			if (dwLength == 0 || dwLength>65536 ) {
				_err_printf("<Error> length out of range 1 ~ 65536");
				return -1;
			}
			xi++;
		}
		else if ( strcmp("-channel", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input channel");
				return -1;
			}
			dwData = str_to_hex(argv[xi+1]);
			if (dwData < 0 || dwData>8 ) {
				_err_printf("<Error> channel out of range 0 ~ 8");
				return -1;
			}
			xi++;
		}
		else if ( strcmp("-value", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input vaule");
				return -1;
			}
			dwData = str_to_hex(argv[xi+1]);
			if ( dwData > 0x100 ) {
				_err_printf("<Warning> erase data range 0 ~ 255");
				__error_exit(argv[0]);
			}			
			xi++;
		}
		else if ( strcmp("-start", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input start location");
				return -1;
			}
			dwStart = str_to_hex(argv[xi+1]);
			xi++;
		}
		else if ( strcmp("-addr", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input device address");
				return -1;
			}
			dwData = str_to_hex(argv[xi+1]);
			if ( dwData < 0x10 || dwData > 0xFC ) {
				_err_printf("<Warning> addr< 0x10 || addr >0xFC");
				__error_exit(argv[0]);
			}			
			ubDevAddr = dwData & 0xFF;
			xi++;
		}
		else if ( strcmp("-file", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input file");
				return -1;
			}
			strcpy(CONFIG_NAME, argv[xi+1]);
			xi++;
		}
		else if ( strcmp("-string", argv[xi]) == 0 ) {
			if ( xi == argc-1 ) {
				_err_printf("<Warning> not input string");
				return -1;
			}
			strcpy(arybWriteData, argv[xi+1]);
			xi++;
		}
		else {
			_err_printf("<Warning> invalid command or parameter input");
			__printf_usage(argv[0]);
			return -1;
		}
	}

	if ( iType==-1 ) {
		__printf_usage(argv[0]);
		return -1;
	}
	/*****************************************************************/
	switch (iType) {
	case ITEM_BUSSWITCH:
		if ( ubDevAddr < 0xe0 || ubDevAddr > 0xF0) {
			_err_printf("<Error> ITEM_BUSSWITCH Addr out of range 0xE0 ~ 0xEE");
		}
		if (dwData == 8 ) ubReg=0;
		else		  ubReg = (uint8_t)(0x01 <<  dwData);
		iret = _lmbsmb_i2c_read_byte(bDevPort, ubDevAddr, ubReg, arybWriteData);
		if ( iret !=0  ) _err_printf("<Error> ITEM_BUSSWITCH _lmbsmb_i2c_read_byte");
		break;
	case ITEM_EEP2FILE:
		if ( dwLength == 0 || ubDevAddr==0xFF || (dwLength+dwStart) > aryEEPROM_SIZE[dwEEP_type]) {
			_err_printf("<Error> ITEM_EEP2FILE start+length out of EEPROM size");
			__error_exit(argv[0]);
		}
		fActionType = 1;
		iRet =  _eep_to_file(ubDevAddr, dwLength);
		break;
	case ITEM_FILE2EEP:
		_file_to_eep();
		break;
	case ITEM_EEP2DISP:	
		if ( dwLength == 0 || ubDevAddr==0xFF || (dwLength+dwStart) > aryEEPROM_SIZE[dwEEP_type]) {
			 _err_printf("<Error> ITEM_EEP2DISP start+length out of EEPROM size");
			__error_exit(argv[0]);
		}
		fActionType =0;
		iRet =  _eep_to_file(ubDevAddr, dwLength);
		break;
	case ITEM_WRITEASC:
		dwLength = strlen((char*)arybWriteData)+1;
		if ( dwLength == 0 || ubDevAddr==0xFF || (dwLength+dwStart) > aryEEPROM_SIZE[dwEEP_type]) {
			 _err_printf("<Error>ITEM_WRITEASC start+length out of EEPROM size");
			__error_exit(argv[0]);
		}
		if (dwEEP_type!=0) {
			iret = _lmbsmb_write_eepEX(bDevPort, dwStart, dwLength, arybWriteData);
			if ( iret !=0 ) {
				 _err_printf("<Error>read from EEPROM _lmbsmb_write_eepEX");
			}
		}
		else {
			for ( xi=0 ; xi<dwLength;  xi++) {
				ubReg = (uint8_t)(dwStart & 0xFF) + xi;
				for ( xj=1 ; xj<=xCnt ; xj++ ){
					iret = _lmbsmb_i2c_write_byte(bDevPort, ubDevAddr, ubReg, arybWriteData[xi]);
					if ( iret == 0  ) xj=100;
				}
				if (  xj < 100 )  _err_printf("<Error>ITEM_WRITEASC _lmbsmb_i2c_write_byte");
			}		
		}
		break;
	case ITEM_WRITEBIN:
		dwLength = strlen((char*)arybWriteData) / 2;
		if ( dwLength == 0 || ubDevAddr==0xFF || (dwLength+dwStart) > aryEEPROM_SIZE[dwEEP_type]) {
			_err_printf("<Error>ITEM_WRITEBIN start+length out of EEPROM size");
			__error_exit(argv[0]);
		}
		
		if (dwEEP_type!=0) {
			bptr=bptw=arybWriteData;
			for ( xi=0 ; xi<dwLength;  xi++) {
				tmpBuf[0]=*bptr++;
				tmpBuf[1]=*bptr++;
				tmpBuf[2]=0;
				ubData = (uint8_t)(strtoul(tmpBuf, 0, 16) & 0xFF);
				*bptw++=ubData;
			}
			iret = _lmbsmb_write_eepEX(bDevPort, dwStart, dwLength, arybWriteData); //Legend
			if ( iret !=0 ) {
				 _err_printf("<Error>read from EEPROM _lmbsmb_write_eepEX");
			}
		}
		else {
			bptr=arybWriteData;
			for ( xi=0 ; xi<dwLength;  xi++) {
				tmpBuf[0]=*bptr++;
				tmpBuf[1]=*bptr++;
				tmpBuf[2]=0;
				ubData = (uint8_t)(strtoul(tmpBuf, 0, 16) & 0xFF);
				ubReg = (uint8_t)(dwStart & 0xFF) + xi;
				for ( xj=1 ; xj<=xCnt ; xj++ ){
					iret = _lmbsmb_i2c_write_byte(bDevPort, ubDevAddr, ubReg, ubData);
					if ( iret == 0  ) xj=100;
				}
				if (  xj < 100 )  _err_printf("<Error>ITEM_WRITEBIN _lmbsmb_i2c_write_byte");			
			}
		}
		break;
	//case ITEM_COMPARE:
	//	break;
	case ITEM_ERASE:
		dwStart &= 0xFFFFFF00 ;
		if ( dwLength == 0 || ubDevAddr==0xFF || (dwLength+dwStart) > aryEEPROM_SIZE[dwEEP_type]) {
			 _err_printf("<Error>ITEM_ERASE start+length out of EEPROM size");
			__error_exit(argv[0]);
		}
		memset(arybWriteData,(uint8_t)(dwData&0xFF),256); 
		if (dwEEP_type!=0) {		
			iret = _lmbsmb_write_eepEX(bDevPort, dwStart, dwLength, arybWriteData);
			if ( iret !=0 ) {
				 _err_printf("<Error>read from EEPROM _lmbsmb_write_eepEX");
			}
		}
		else {
			for ( xi=0 ; xi<dwLength;  xi++) {
				ubReg = (uint8_t)(dwStart & 0xFF) + xi;
				for ( xj=1 ; xj<=xCnt ; xj++ ){
					iret = _lmbsmb_i2c_write_byte(bDevPort, ubDevAddr, ubReg, arybWriteData[xi]);
					if ( iret == 0  ) xj=100;
				}
				if (  xj < 100 )  _err_printf("<Error>ITEM_WRITEASC _lmbsmb_i2c_write_byte");
			}		
		}
		break;
	case ITEM_GETSTRING:
		__read_string_disp(ubDevAddr);
		break;
	default:
		__error_exit(argv[0]);
		break;
	}

	return iRet;
}
FILE *pFile = NULL;
int32_t offset_addr=7, offset_data=45;
int _savefile(uint8_t* pbSting);
/*************************************/
/*** Save EEPROM to File *************/
/*************************************/
int _eep_to_file(uint8_t ubAddr, int32_t dwLength)
{
int iret, xi,xj, xCnt =3;
uint8_t ubReg, aryuData[2];

int32_t dwIndex=0, dwAddr=0, dwCnt=0;
uint8_t aryDisp[80], xline =0;
uint32_t *dwptr;
uint16_t *wptr;
uint8_t *bptr;
int errCnt=0;

	if (dwEEP_type!=0) {
		iret = _lmbsmb_read_eepEX(bDevPort, dwStart, dwLength, aryEEPROM);
		if ( iret !=0 ) {
			if ( iret == -RET_DriverNoLoad ) _err_printf("<Error>i2c-dev driver doesn't load !!!");
			else				 _err_printf("<Error>read from EEPROM _lmbsmb_read_eepEX");
			return -1;
		}
	}
	else {
		for (xi=0 ; xi<dwLength+dwStart ; xi++ ) {
			ubReg = (uint8_t)xi + (uint8_t)(dwStart&0xFF);
			for ( xj=1 ; xj<=xCnt ; xj++ ){
				iret = _lmbsmb_i2c_read_byte(bDevPort, ubAddr, ubReg, aryuData);
				if ( iret == 0  ) xj=100;
				else {
					if ( iret == -RET_DriverNoLoad ) _err_printf("<Error>i2c-dev driver doesn't load !!!");
					else				 _err_printf("<Error>read from EEPROM _lmbsmb_i2c_read_byte");
					errCnt++;
					if ( errCnt >=3 ) return -1;
				}
			}
			if (  xj >= 100 ) aryEEPROM[xi] = aryuData[0];
		}
 	}
	
	sprintf(aryDisp, "%s", "#            <<   Lanner EEPROM Data File   >>              \n");
	if ( fActionType == 0 )	printf("%s", aryDisp);
	else 			_savefile(aryDisp);
	sprintf(aryDisp, "%s", "# Addr |-----------   Data  ----------|      |--   ASCII  --|\n");
	if ( fActionType == 0 )	printf("%s", aryDisp);
	else 			_savefile(aryDisp);
	sprintf(aryDisp, "%s", "#============================================================\n");
	if ( fActionType == 0 )	printf("%s", aryDisp);
	else 			_savefile(aryDisp);
	
	dwIndex =0;
	xCnt= dwStart % 16;
	if ( xCnt != 0 ) {
		memset(aryDisp, 0x20, 80);
		sprintf(aryDisp, ":%05X ", dwIndex + dwStart);
		aryDisp[offset_addr] = ' ';
	}
	
	//memset(aryDisp, 0x20, 80);
	while ( dwIndex < dwLength ) {
		//if ( xCnt == 0 ) printf("%04X ", dwIndex );
		//printf("%02X", aryEEPROM[dwIndex])
		if ( xCnt == 0 ) {
			memset(aryDisp, 0x20, 80);
			sprintf(aryDisp, ":%05X ", dwIndex + dwStart);
		}
		if ( xline == 0 ) {
			bptr = &aryDisp[offset_addr+(xCnt-(dwStart%16))*2];
			sprintf(bptr, "%02X", aryEEPROM[dwIndex]);
			bptr = &aryDisp[offset_addr+(xCnt+1-(dwStart%16))*2];
			*bptr=' ';
			bptr = &aryDisp[offset_data+xCnt-(dwStart%16)];
			if ( isprint(aryEEPROM[dwIndex]) ) *bptr=aryEEPROM[dwIndex];
			else				   *bptr='.';
		}
		else {
			bptr = &aryDisp[offset_addr+xCnt*2];
			sprintf(bptr, "%02X", aryEEPROM[dwIndex]);
			//aryDisp[offset_addr+xCnt*2+1] = 0x20 ;
			bptr = &aryDisp[offset_data+xCnt];
			if ( isprint(aryEEPROM[dwIndex]) ) *bptr=aryEEPROM[dwIndex];
			else 						 *bptr='.';
		}
		//bptr = &aryDisp[offset_addr+xCnt*2];
		//*bptr = ' ';
		//bptr = &aryDisp[44+xCnt+1];
		//*bptr = 0;
		dwIndex++;
		xCnt++;
		if ( xCnt == 16 ) {
			if ( xline == 0 ) {
				bptr = &aryDisp[offset_addr+(xCnt-(dwStart%16))*2];
				*bptr = '#';
			}
			else {
				bptr = &aryDisp[offset_addr+xCnt*2];
				*bptr = '#';
			}
			bptr = &aryDisp[offset_data+xCnt];
			*bptr++ = 0x0A;
			*bptr++ = 0x0;
			if ( fActionType == 0 )	printf("%s", aryDisp);
			else 			_savefile(aryDisp);
			//memset(aryDisp, 0x20, 80);
			xCnt=0;
			xline++;
		}			
	}
	//check last line
	if ( xCnt !=0 ) {
		bptr = &aryDisp[offset_addr+xCnt*2];
		*bptr = '#';
		bptr = &aryDisp[offset_data+xCnt];
		*bptr++ = 0x0A;
		*bptr++ = 0x0;
		if ( fActionType == 0 )	printf("%s", aryDisp);
		else 			_savefile(aryDisp);
		//memset(aryDisp, 0x20, 80);
	}
	sprintf(aryDisp, "%s", "@END\n");
	if ( fActionType == 0 )	printf("%s", aryDisp);
	else 			_savefile(aryDisp);
	if ( pFile != NULL ) fclose(pFile);
	return 1;
}

/********************/
int _savefile(uint8_t* pbSting) 
{

	if (  pFile == NULL )  pFile = fopen( CONFIG_NAME,"w" );
	if( NULL == pFile ) {
	        printf( "open failure" );
	        return 1;
	}
	else {
		fputs(pbSting, pFile);
		//fputc(0x0a, pFile);
    	}
}
/*************************************/
/*** Save File to EEPROM *************/
/*************************************/
int _file_to_eep(void)
{
int8_t errMsg[256];
uint8_t xch, aryubData[16], tmpBuf[3];
char *bptr;
uint32_t dwAddr;
int dwBytes=0, xi,xj,xCnt =3, iret;
uint8_t ubReg; 
int dwErrCnt=0;
	if (  pFile == NULL )  pFile = fopen( CONFIG_NAME,"r" );
	if( NULL == pFile ) {
		sprintf(errMsg, "open %s failure !!!", CONFIG_NAME);
	        _err_printf(errMsg);
	        return 0;
	}
	fseek(pFile, 0, SEEK_SET);
	while(!feof(pFile)) {
		fgets(errMsg, 255, pFile);
		if ( feof(pFile) ) break;
		if ( errMsg[0] == ':' ) {
			bptr = strstr((char*)errMsg, "#");
			if ( bptr != NULL ) *bptr =0x00; 
			dwAddr = strtoul((char*)&errMsg[1], 0, 16);
			//printf("dwAddr=0x%05X, ", dwAddr);
			dwBytes = strlen((char*)&errMsg[offset_addr]) / 2;
			//printf("dwBytes=%d\n", dwBytes);
			bptr = &errMsg[offset_addr];
			for ( xi=0 ; xi<dwBytes;  xi++) {
				tmpBuf[0]=*bptr++;
				tmpBuf[1]=*bptr++;
				tmpBuf[2]=0;
				aryubData[xi] = (uint8_t)(strtoul(tmpBuf, 0, 16) & 0xFF);
				//printf("0x%02X,", aryubData[xi]);

				ubReg = (uint8_t)(dwAddr & 0xFF) + xi;
				if (dwEEP_type!=0) {
					iret = _lmbsmb_write_eepEX(bDevPort, dwAddr, dwBytes, aryubData);
					if ( iret !=0 ) {
						dwErrCnt++;
						if ( iret == -RET_DriverNoLoad ) _err_printf("<Error>i2c-dev driver doesn't load !!!");
						else				 _err_printf("<Error>write to EEPROM _lmbsmb_write_eepEX");
						//_err_printf("<Error>read from EEPROM _lmbsmb_write_eepEX");
						//printf("\e[1;31m<Info> dwAddr=%X, dwBytes=%X\e[m\n", dwAddr, dwBytes);
						if ( dwErrCnt >= DEF_ERROR_MAX) return -1;
					 	
					}	
				}
				else {
					for ( xj=1 ; xj<=xCnt ; xj++ ){
						iret = _lmbsmb_i2c_write_byte(bDevPort, ubDevAddr, ubReg, aryubData[xi]);
						if ( iret == 0  ) xj=100;
					}
					if (  xj < 100 )  {
						dwErrCnt++;
						if ( iret == -RET_DriverNoLoad ) _err_printf("<Error>i2c-dev driver doesn't load !!!");
						else				 _err_printf("<Error>write to EEPROM _lmbsmb_i2c_write_byte");
						//_err_printf("<Error>_lmbsmb_i2c_write_byte");	
						if ( dwErrCnt >= DEF_ERROR_MAX) return -1;
					}
				}			
			}
			//printf("\n");
			//printf("%s", errMsg);

		}
		else if (errMsg[0] == '@' ) {
			printf("-END-\n");
		}
		else {
		}
		

//		xch = fgetc(pFile);		
//		if ( feof(pFile) ) break;
//		printf("%c", xch);
	}
	


	return 1;
}
/*****************************************/
int __read_string_disp(uint8_t ubAddr)
{
int iret, xi,xj, xCnt =3;
uint8_t ubReg, aryuData[2];

int32_t dwIndex=0, dwAddr=0, dwCnt=0;
int32_t dwLength=256;


	if (dwEEP_type!=0) {
		iret = _lmbsmb_read_eepEX(bDevPort, dwStart, dwLength, aryEEPROM);
		if ( iret !=0 ) {
			_err_printf("<Error>read from EEPROM _lmbsmb_read_eepEX");
			return -1;
		}
	}
	else {
		for (xi=0 ; xi<dwLength+dwStart ; xi++ ) {
			ubReg = (uint8_t)xi + (uint8_t)(dwStart&0xFF);
			for ( xj=1 ; xj<=xCnt ; xj++ ){
				iret = _lmbsmb_i2c_read_byte(bDevPort, ubAddr, ubReg, aryuData);
				if ( iret == 0  ) xj=100;
			}
			if (  xj < 100 )  xj = 0x100000;
			else		  aryEEPROM[xi] = aryuData[0];
		}
 	}
	for (xi=0 ; xi<256 ; xi++ ) {
		if ( aryEEPROM[xi] != 0x00) printf("%c", aryEEPROM[xi]);
		else			    {printf("\n"); xi=10000; }
	}
	return 1;
}

