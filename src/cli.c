/*
 * UART command-line interface for real-time PID tuning
 *
 * Commands:
 *   kp <val>     - Set proportional gain
 *   ki <val>     - Set integral gain
 *   kd <val>     - Set derivative gain
 *   sp <val>     - Set setpoint (degrees)
 *   step <val>   - Step to angle (triggers step response logging)
 *   reset        - Reset PID integrator
 *   status       - Print current gains and state
 *   help         - Print command list
 */

#include "cli.h"
#include "hal.h"
#include <string.h>
#include <stdlib.h>

void cli_init(cli_t *cli, pid_controller_t *pid) {
    cli->pos = 0;
    cli->pid = pid;
    cli->step_pending = 0;
    cli->step_target = 0;
}

static void cli_exec(cli_t *cli, const char *cmd) {
    pid_controller_t *pid = cli->pid;

    if (strncmp(cmd, "kp ", 3) == 0) {
        pid->kp = strtof(cmd + 3, NULL);
        uart_printf("kp = %.4f\n", pid->kp);
    } else if (strncmp(cmd, "ki ", 3) == 0) {
        pid->ki = strtof(cmd + 3, NULL);
        uart_printf("ki = %.4f\n", pid->ki);
    } else if (strncmp(cmd, "kd ", 3) == 0) {
        pid->kd = strtof(cmd + 3, NULL);
        uart_printf("kd = %.4f\n", pid->kd);
    } else if (strncmp(cmd, "sp ", 3) == 0) {
        float sp = strtof(cmd + 3, NULL);
        pid_set_setpoint(pid, sp);
        uart_printf("setpoint = %.2f\n", sp);
    } else if (strncmp(cmd, "step ", 5) == 0) {
        cli->step_target = strtof(cmd + 5, NULL);
        cli->step_pending = 1;
        uart_printf("step -> %.2f (logging 2s)\n", cli->step_target);
    } else if (strcmp(cmd, "reset") == 0) {
        pid_reset(pid);
        uart_printf("PID reset\n");
    } else if (strcmp(cmd, "status") == 0) {
        uart_printf("kp=%.4f ki=%.4f kd=%.4f sp=%.2f out=%.3f int=%.3f\n",
                    pid->kp, pid->ki, pid->kd,
                    pid->setpoint, pid->output, pid->integral);
    } else if (strcmp(cmd, "help") == 0) {
        uart_printf("commands: kp ki kd sp step reset status help\n");
    } else if (cmd[0] != '\0') {
        uart_printf("unknown: %s\n", cmd);
    }
}

void cli_process_byte(cli_t *cli, char c) {
    if (c == '\r' || c == '\n') {
        if (cli->pos > 0) {
            cli->buf[cli->pos] = '\0';
            cli_exec(cli, cli->buf);
            cli->pos = 0;
        }
    } else if (c == '\b' || c == 127) {
        if (cli->pos > 0) {
            cli->pos--;
            uart_send_string("\b \b");
        }
    } else if (cli->pos < CLI_BUF_SIZE - 1) {
        cli->buf[cli->pos++] = c;
        uart_send_byte(c);
    }
}
