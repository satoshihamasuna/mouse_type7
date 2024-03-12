/*
 * myshell.cpp
 *
 *  Created on: 2024/03/09
 *      Author: sato1
 */

#include "core/ntshell.h"
#include "core/ntlibc.h"
#include "util/ntopt.h"
#include "../../Module/Inc/communicate.h"
#include "../Inc/myshell.h"
#include <stdio.h>

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int user_callback(const char *text, void *extobj);

static int usrcmd_execute(const char *text);
static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);

//コマンド一覧
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);


typedef struct {
	const char *cmd;
	const char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
};

static ntshell_t nts;


/* ---------------------------------------------------------------
	help と info
--------------------------------------------------------------- */
static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (uint i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        printf("  %s", p->cmd);
        printf("\t:");
        printf("  %s", p->desc);
        printf("\r\n");
        p++;
    }
    return 0;
}

static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
    	printf("info sys\r\n");
    	printf("info ver\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
    	printf("prototype7\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
    	printf("Version 0.0.0\r\n");
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}


/* ---------------------------------------------------------------
	送受信用ローカル関数
--------------------------------------------------------------- */
static int func_read(char *buf, int cnt, void *extobj)
{
	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  buf[i] = (char) Communicate_RxPopData();
	  }
	  return cnt;
}

static int func_write(const char *buf, int cnt, void *extobj)
{

	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  Communicate_TxPushData((int8_t)(buf[i]));
	  }
	return cnt;
}

/* ---------------------------------------------------------------
	コールバック関数
--------------------------------------------------------------- */
static int user_callback(const char *text, void *extobj)
{

	usrcmd_execute(text);
	return 0;
}

static int usrcmd_execute(const char *text)
{
	return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (uint i = 0; i < (sizeof(cmdlist) / sizeof(cmdlist[0])); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    printf("Unknown command found.\r\n");
    return 0;
}

/* ---------------------------------------------------------------
	初期設定関数
--------------------------------------------------------------- */

void Myshell_Initialize( void )
{
	void *extobj = 0;

	ntshell_init(&nts, func_read, func_write, user_callback, extobj);
}

void Myshell_Execute( void )
{

	if( (&nts)->initcode != 0x4367 ) {
		return;
	} else;

	unsigned char ch;
	func_read((char *)&ch, sizeof(ch), (&nts)->extobj);
	vtrecv_execute(&((&nts)->vtrecv), &ch, sizeof(ch));
	//ntshell_execute(&nts);
	//ntshell_execute(&nts);
}

