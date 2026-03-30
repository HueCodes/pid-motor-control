#ifndef CLI_H
#define CLI_H

#include "pid.h"

#define CLI_BUF_SIZE 128

typedef struct {
    char buf[CLI_BUF_SIZE];
    uint32_t pos;
    pid_controller_t *pid;
    volatile int step_pending;
    float step_target;
} cli_t;

void cli_init(cli_t *cli, pid_controller_t *pid);
void cli_process_byte(cli_t *cli, char c);

#endif
