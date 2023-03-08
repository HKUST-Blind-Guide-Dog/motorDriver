#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif 

#include "motor.h"
#include <sched.h>  
#include <pthread.h>  
#include <limits.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <sys/mman.h>	   // Needed for mlockall()
#include <malloc.h>
#include <sys/time.h>	   // Needed for getrusage
#include <sys/resource.h>  // Needed for getrusage

int err;
pthread_t task_handle_control;
pthread_attr_t task_attr_control;
#define SAVE_DATA_SIZE 1500 //control frequency = 
#define JOINT_SIZE 1
int control_loop_delay_us = 2500;
float save_data_pos[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
float save_data_vel[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
float save_data_tor[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
float save_data_target_value[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
//float save_cmd_tor[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
float save_data_err[SAVE_DATA_SIZE][JOINT_SIZE] = {0};
float save_data_comm[SAVE_DATA_SIZE] = {0};
float save_data_time[SAVE_DATA_SIZE] = {0};
float save_data_cost[SAVE_DATA_SIZE] = {0};
//motor target value
float target_value[JOINT_SIZE] = {0};


void *control_task(void *arg)
{  
    printf("JOINT_SIZE: %d\n", JOINT_SIZE);
    // usleep(2000000);
    struct timeval t;
    int t_max = 0, t_min = 1000000, t_sum = 0;
    int tt_max = 0, tt_min = 1000000, tt_sum = 0;
    long int t00, t0, t1, t2;
    int16_t i, j;
    float pi = 3.1415926, k[JOINT_SIZE]; 

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = control_loop_delay_us * 1000L;
    serial motorSerial0, motorSerial1;
    int motorId = 1;
    Motor motor(motorId, &motorSerial0);
    // Motor motor1(2, &motorSerial0, "/dev/ttyCH9344USB0");
    // Motor motor2(1, &motorSerial1);
    Motor* motors[JOINT_SIZE];
    motors[0] = &motor;
    // motors[0] = &motor1;
    // motors[1] = &motor2;
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    gettimeofday(&t, NULL);
    t00 = t.tv_sec * 1000000 + t.tv_usec;

    for (i = 1; i < SAVE_DATA_SIZE; ++ i) {
        //printf("num of set: %d \n",i);
        gettimeofday(&t, NULL);
        t0 = t.tv_sec * 1000000 + t.tv_usec;

        for(j = 0; j < JOINT_SIZE; j++){
            /* torque test
            target_value[j] = 0;
            motor.setTarget(target_value[j], ControlMethod::TORQUE);
            */
            //velocity test
            target_value[j] = 2 * (sin(2.0 * pi * i / SAVE_DATA_SIZE));
            motors[j]->setTarget(target_value[j], ControlMethod::VELOCITY);
            /*position test
            target_value[j] = 150 * (1.0 - cos(2.0 * pi * i / SAVE_DATA_SIZE));
            motor.setTarget(target_value[j], ControlMethod::POSITION);
            */
           save_data_target_value[i][j] = target_value[j]; 
        }

        //take the data from motors
        for(j = 0; j < JOINT_SIZE; j++){
            save_data_pos[i][j] = motors[j]->getCurPos();
            save_data_vel[i][j] = motors[j]->getCurVel();
            save_data_tor[i][j] = motors[j]->getCurTor();
            save_data_err[i][j] = target_value[j] - save_data_pos[i][j];
        }
        gettimeofday(&t, NULL);
        t1 = t.tv_sec * 1000000 + t.tv_usec;
        
        /*for (j = 0; j < JOINT_SIZE; ++ j) {
            save_data_err[i][j] = q_cmd[j] - save_data_pos[i][j];
        }*/

        save_data_comm[i] = t1 - t0;
        t_sum += t1 - t0;
        if (t_max < t1 - t0)
            t_max = t1 - t0;
        if (t_min > t1 - t0)
            t_min = t1 - t0;
        if (save_data_comm[i] > control_loop_delay_us)
            ts.tv_nsec = (control_loop_delay_us - t1 + t0 + (int)(save_data_comm[i] / control_loop_delay_us) * control_loop_delay_us) * 1000L;
        else{
            ts.tv_nsec = (control_loop_delay_us - t1 + t0) * 1000L;
            clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        }

        gettimeofday(&t, NULL);
        t2 = t.tv_sec * 1000000 + t.tv_usec;
        tt_sum += t2 - t0;
        if (tt_max < t2 - t0)
            tt_max = t2 - t0;
        if (tt_min > t2 - t0)
            tt_min = t2 - t0;
        save_data_time[i] = t2 - t00;
        if (i > 0) {
            save_data_cost[i] = save_data_time[i] - save_data_time[i-1];
        }
    }
    motor.setComSpeed(2000000);
    printf("PDO max time %d, min time %d, ave time %d us. [%d]\n", t_max, t_min, t_sum / i, i);
    printf("Task max time %d, min time %d, ave time %d us. [%d]\n", tt_max, tt_min, tt_sum / i, i);
    printf("create file for saving data \n");
    // save data
    FILE *fp = fopen("save_data_rt.txt", "wb");
    if (fp == NULL) {
        printf("File cannot open!\n" );
        exit(0);
    }else
         printf("successfully open the file and started to record the data \n");
    float rad = 180.0 / pi;
    for(i = 0; i < SAVE_DATA_SIZE; i++){
        fprintf(fp,"%d %f %f %f", i, save_data_comm[i], save_data_cost[i], save_data_time[i]);
        for(j = 0; j < JOINT_SIZE; j++)
            fprintf(fp,"%f %f %f %f %f ", save_data_err[i][j]/9, save_data_pos[i][j]-save_data_pos[0][j], save_data_vel[i][j], save_data_tor[i][j], save_data_target_value[i][j]);
        fprintf(fp,"\n");
    }
    fclose(fp);
    return NULL;
}

int creat_rt_thread(pthread_t *task_handle, pthread_attr_t *task_attr, int priority, void *(*fun)(void *))
{
    /* Initialize pthread attributes (default values) */
    int ret = pthread_attr_init(task_attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        return -1;
    }

    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(task_attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        return -2;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(task_attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        return -3;
    }

    struct sched_param param;
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(task_attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        return -4;
    }
    
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(task_attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        return -5;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(task_handle, task_attr, fun, NULL);
    if (ret) {
        printf("create pthread failed\n");
        return -6;
    }
    
    return 0;
}

int main()
{
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }
    // Turn off malloc trimming.
    mallopt(M_TRIM_THRESHOLD, -1);
    //d Turn off mmap usage. 
    mallopt(M_MMAP_MAX, 0);

    // create thread for rt control
    if (creat_rt_thread(&task_handle_control, &task_attr_control, 90, control_task)) {
        if (pthread_join(task_handle_control, NULL))
            printf("join pthread failed: %m\n");
        printf("Error: Cannot create watchdog thread.\n");
        return -7;
    } else {
        printf("RT pdo start.\n");
        while (1)
            sched_yield();
            
        if (pthread_join(task_handle_control, NULL))
            printf("join pthread failed: %m\n");
    } 

    printf("End program\n");
    return 0;
}

// int main(){
//     serial motorSerial;
//     Motor motor(1, &motorSerial);
//     motor.setComSpeed(2000000);
//     // while(1) {
//     //     motor.setTarget(10, ControlMethod::VELOCITY);
//     //     usleep(20000);
//     // }
//     return 0;
// }
