### 准备文件

我的STM32项目是经由`STM32CubeMX` v6.2.0生成的`Makefile`项目；使用[VS-Code-STM32-IDE](https://github.com/damogranlabs/VS-Code-STM32-IDE)来生成VS Code项目；使用v10.3.1的FreeRTOS系统；芯片为`STM32F407VE`；使用USART1作为输出端口，开启DMA传输。

下载[SystemView](https://www.segger.com/products/development-tools/systemview/) v3.30版本。在安装目录内的`SystemView_V330\Src`文件夹中，将`SEGGER`和`Sample`内的相关文件复制到STM32项目的文件夹`STM32_Project\Middlewares\Third_Party\SystemView`中[^1][^2]。注意根据项目的需求，选择复制`Sample`中对应的系统和`SEGGER\Syscalls`中对应的编译器:

```
SystemView
├── Config
│   ├── Global.h
│   ├── SEGGER_RTT_Conf.h
│   ├── SEGGER_SYSVIEW_Conf.h
│   └── SEGGER_SYSVIEW_Config_FreeRTOS.c
└── SEGGER
    ├── SEGGER.h
    ├── SEGGER_RTT.c
    ├── SEGGER_RTT.h
    ├── SEGGER_RTT_ASM_ARMv7M.s
    ├── SEGGER_RTT_printf.c
    ├── SEGGER_SYSVIEW.c
    ├── SEGGER_SYSVIEW.h
    ├── SEGGER_SYSVIEW_ConfDefaults.h
    ├── SEGGER_SYSVIEW_FreeRTOS.c
    ├── SEGGER_SYSVIEW_FreeRTOS.h
    ├── SEGGER_SYSVIEW_Int.h
    └── Syscalls
        └── SEGGER_RTT_Syscalls_GCC.c
```

**注意：`SEGGER_RTT_ASM_ARMv7M.S`需要改名为`SEGGER_RTT_ASM_ARMv7M.s`，不然后面编译时会出现`No rule to make target`错误。**

### FreeRTOS补丁

下面是针对`FreeRTOS` v10.3.1版本的补丁，复制下面内容，并且保存名为`patch`的文件，并且放到`STM32_Project\Middlewares\Third_Party`，内含`FreeRTOS`的文件夹下：

```
diff -rupwN FreeRTOS/Source/include/FreeRTOS.h FreeRTOS_systemview/Source/include/FreeRTOS.h
--- FreeRTOS/Source/include/FreeRTOS.h	2021-03-22 00:24:29.699956400 +0800
+++ FreeRTOS_systemview/Source/include/FreeRTOS.h	2021-03-14 03:13:48.049285300 +0800
@@ -160,10 +160,6 @@ extern "C" {
 	#define INCLUDE_uxTaskGetStackHighWaterMark2 0
 #endif
 
-#ifndef INCLUDE_pxTaskGetStackStart
-	#define INCLUDE_pxTaskGetStackStart 0
-#endif
-
 #ifndef INCLUDE_eTaskGetState
 	#define INCLUDE_eTaskGetState 0
 #endif
@@ -420,22 +416,6 @@ hold explicit before calling the code. *
 	#define tracePOST_MOVED_TASK_TO_READY_STATE( pxTCB )
 #endif
 
-#ifndef traceREADDED_TASK_TO_READY_STATE
-	#define traceREADDED_TASK_TO_READY_STATE( pxTCB )	traceMOVED_TASK_TO_READY_STATE( pxTCB )
-#endif
-
-#ifndef traceMOVED_TASK_TO_DELAYED_LIST
-	#define traceMOVED_TASK_TO_DELAYED_LIST()
-#endif
-
-#ifndef traceMOVED_TASK_TO_OVERFLOW_DELAYED_LIST
-	#define traceMOVED_TASK_TO_OVERFLOW_DELAYED_LIST()
-#endif
-
-#ifndef traceMOVED_TASK_TO_SUSPENDED_LIST
-	#define traceMOVED_TASK_TO_SUSPENDED_LIST( pxTCB )
-#endif
-
 #ifndef traceQUEUE_CREATE
 	#define traceQUEUE_CREATE( pxNewQueue )
 #endif
@@ -680,18 +660,6 @@ hold explicit before calling the code. *
 	#define traceTASK_NOTIFY_GIVE_FROM_ISR()
 #endif
 
-#ifndef traceISR_EXIT_TO_SCHEDULER
-	#define traceISR_EXIT_TO_SCHEDULER()
-#endif
-
-#ifndef traceISR_EXIT
-	#define traceISR_EXIT()
-#endif
-
-#ifndef traceISR_ENTER
-	#define traceISR_ENTER()
-#endif
-
 #ifndef traceSTREAM_BUFFER_CREATE_FAILED
 	#define traceSTREAM_BUFFER_CREATE_FAILED( xIsMessageBuffer )
 #endif
diff -rupwN FreeRTOS/Source/include/task.h FreeRTOS_systemview/Source/include/task.h
--- FreeRTOS/Source/include/task.h	2021-03-22 00:27:05.253377700 +0800
+++ FreeRTOS_systemview/Source/include/task.h	2021-03-14 03:13:48.062250900 +0800
@@ -1468,25 +1468,6 @@ UBaseType_t uxTaskGetStackHighWaterMark(
  */
 configSTACK_DEPTH_TYPE uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) PRIVILEGED_FUNCTION;
 
-/**
- * task.h
- * <PRE>uint8_t* pxTaskGetStackStart( TaskHandle_t xTask);</PRE>
- *
- * INCLUDE_pxTaskGetStackStart must be set to 1 in FreeRTOSConfig.h for
- * this function to be available.
- *
- * Returns the start of the stack associated with xTask.  That is,
- * the highest stack memory address on architectures where the stack grows down
- * from high memory, and the lowest memory address on architectures where the
- * stack grows up from low memory.
- *
- * @param xTask Handle of the task associated with the stack returned.
- * Set xTask to NULL to return the stack of the calling task.
- *
- * @return A pointer to the start of the stack.
- */
-uint8_t* pxTaskGetStackStart( TaskHandle_t xTask) PRIVILEGED_FUNCTION;
-
 /* When using trace macros it is sometimes necessary to include task.h before
 FreeRTOS.h.  When this is done TaskHookFunction_t will not yet have been defined,
 so the following two prototypes will cause a compilation error.  This can be
diff -rupwN FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c FreeRTOS_systemview/Source/portable/GCC/ARM_CM4F/port.c
--- FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c	2021-03-22 00:29:26.581083000 +0800
+++ FreeRTOS_systemview/Source/portable/GCC/ARM_CM4F/port.c	2021-03-14 03:13:48.114111800 +0800
@@ -492,20 +492,14 @@ void xPortSysTickHandler( void )
 	save and then restore the interrupt mask value as its value is already
 	known. */
 	portDISABLE_INTERRUPTS();
-	traceISR_ENTER();
 	{
 		/* Increment the RTOS tick. */
 		if( xTaskIncrementTick() != pdFALSE )
 		{
-			traceISR_EXIT_TO_SCHEDULER();
 			/* A context switch is required.  Context switching is performed in
 			the PendSV interrupt.  Pend the PendSV interrupt. */
 			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
 		}
-		else
-		{
-			traceISR_EXIT();
-		}
 	}
 	portENABLE_INTERRUPTS();
 }
diff -rupwN FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h FreeRTOS_systemview/Source/portable/GCC/ARM_CM4F/portmacro.h
--- FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h	2021-03-22 00:30:13.377937500 +0800
+++ FreeRTOS_systemview/Source/portable/GCC/ARM_CM4F/portmacro.h	2021-03-14 03:13:48.115109400 +0800
@@ -89,7 +89,7 @@ typedef unsigned long UBaseType_t;
 
 #define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
 #define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )
-#define portEND_SWITCHING_ISR( xSwitchRequired ) { if( xSwitchRequired != pdFALSE ) { traceISR_EXIT_TO_SCHEDULER(); portYIELD(); } else { traceISR_EXIT(); } }
+#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired != pdFALSE ) portYIELD()
 #define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
 /*-----------------------------------------------------------*/
 
diff -rupwN FreeRTOS/Source/tasks.c FreeRTOS_systemview/Source/tasks.c
--- FreeRTOS/Source/tasks.c	2021-03-22 00:48:55.028709000 +0800
+++ FreeRTOS_systemview/Source/tasks.c	2021-03-14 03:13:48.213845800 +0800
@@ -220,16 +220,6 @@ count overflows. */
 	taskRECORD_READY_PRIORITY( ( pxTCB )->uxPriority );												\
 	vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); \
 	tracePOST_MOVED_TASK_TO_READY_STATE( pxTCB )
-/*
- * Place the task represented by pxTCB which has been in a ready list before
- * into the appropriate ready list for the task.
- * It is inserted at the end of the list.
- */
-#define prvReaddTaskToReadyList( pxTCB )															\
-	traceREADDED_TASK_TO_READY_STATE( pxTCB );														\
-	taskRECORD_READY_PRIORITY( ( pxTCB )->uxPriority );												\
-	vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); \
-	tracePOST_MOVED_TASK_TO_READY_STATE( pxTCB )
 /*-----------------------------------------------------------*/
 
 /*
@@ -1682,7 +1672,7 @@ static void prvAddNewTaskToReadyList( TC
 					{
 						mtCOVERAGE_TEST_MARKER();
 					}
-					prvReaddTaskToReadyList( pxTCB );
+					prvAddTaskToReadyList( pxTCB );
 				}
 				else
 				{
@@ -1744,7 +1734,6 @@ static void prvAddNewTaskToReadyList( TC
 				mtCOVERAGE_TEST_MARKER();
 			}
 
-			traceMOVED_TASK_TO_SUSPENDED_LIST(pxTCB);
 			vListInsertEnd( &xSuspendedTaskList, &( pxTCB->xStateListItem ) );
 
 			#if( configUSE_TASK_NOTIFICATIONS == 1 )
@@ -3893,20 +3882,6 @@ static void prvCheckTasksWaitingTerminat
 #endif /* INCLUDE_uxTaskGetStackHighWaterMark */
 /*-----------------------------------------------------------*/
 
-#if (INCLUDE_pxTaskGetStackStart == 1)
-	uint8_t* pxTaskGetStackStart( TaskHandle_t xTask)
-	{
-	    TCB_t *pxTCB;
-	    UBaseType_t uxReturn;
-        (void)uxReturn;
-
-		pxTCB = prvGetTCBFromHandle( xTask );
-		return ( uint8_t * ) pxTCB->pxStack;
-	}
-
-#endif /* INCLUDE_pxTaskGetStackStart */
-/*-----------------------------------------------------------*/
-
 #if ( INCLUDE_vTaskDelete == 1 )
 
 	static void prvDeleteTCB( TCB_t *pxTCB )
@@ -4081,7 +4056,7 @@ TCB_t *pxTCB;
 
 					/* Inherit the priority before being moved into the new list. */
 					pxMutexHolderTCB->uxPriority = pxCurrentTCB->uxPriority;
-					prvReaddTaskToReadyList( pxMutexHolderTCB );
+					prvAddTaskToReadyList( pxMutexHolderTCB );
 				}
 				else
 				{
@@ -4171,7 +4146,7 @@ TCB_t *pxTCB;
 					any other purpose if this task is running, and it must be
 					running to give back the mutex. */
 					listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) pxTCB->uxPriority ); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
-					prvReaddTaskToReadyList( pxTCB );
+					prvAddTaskToReadyList( pxTCB );
 
 					/* Return true to indicate that a context switch is required.
 					This is only actually required in the corner case whereby
@@ -5233,7 +5208,6 @@ const TickType_t xConstTickCount = xTick
 			/* Add the task to the suspended task list instead of a delayed task
 			list to ensure it is not woken by a timing event.  It will block
 			indefinitely. */
-			traceMOVED_TASK_TO_SUSPENDED_LIST(pxCurrentTCB);
 			vListInsertEnd( &xSuspendedTaskList, &( pxCurrentTCB->xStateListItem ) );
 		}
 		else
@@ -5250,14 +5224,12 @@ const TickType_t xConstTickCount = xTick
 			{
 				/* Wake time has overflowed.  Place this item in the overflow
 				list. */
-				traceMOVED_TASK_TO_OVERFLOW_DELAYED_LIST();
 				vListInsert( pxOverflowDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
 			}
 			else
 			{
 				/* The wake time has not overflowed, so the current block list
 				is used. */
-				traceMOVED_TASK_TO_DELAYED_LIST();
 				vListInsert( pxDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
 
 				/* If the task entering the blocked state was placed at the
@@ -5287,13 +5259,11 @@ const TickType_t xConstTickCount = xTick
 		if( xTimeToWake < xConstTickCount )
 		{
 			/* Wake time has overflowed.  Place this item in the overflow list. */
-			traceMOVED_TASK_TO_OVERFLOW_DELAYED_LIST();
 			vListInsert( pxOverflowDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
 		}
 		else
 		{
 			/* The wake time has not overflowed, so the current block list is used. */
-			traceMOVED_TASK_TO_DELAYED_LIST();
 			vListInsert( pxDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
 
 			/* If the task entering the blocked state was placed at the head of the

```

在命令行窗口中输入打补丁指令：

```
patch -p0 < patch
```

### 监听程序

在`STM32_Project\Inc\FreeRTOSConfig.h`下添加以下语句：

```
/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */

#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_pxTaskGetStackStart 1

#include "SEGGER_SYSVIEW_FreeRTOS.h"
/* USER CODE END Defines */
```

新建文件`STM32_Project\Modules\include\systemview_task.h`和`STM32_Project\Modules\src\systemview_task.c`[^3]：

```
#ifndef __SYSTEMVIEW_TASK_H
#define __SYSTEMVIEW_TASK_H

#include "FreeRTOS.h"

#include "config.h"
#include "utils.h"
#include "usart_com.h"

void SystemViewLaunch(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __SYSTEMVIEW_TASK_H */
```

```
#include "systemview_task.h"

#define SYSVIEW_SINGLE_TX 256

TaskHandle_t system_view_notify;

STATIC_MEM_TASK_ALLOC(systemview_task, SYSTEMVIEW_TASK_STACKSIZE);
static void SystemViewTask(void *param);

uint8_t hello_message[32] = {
    'S', 'E', 'G', 'G', 'E', 'R', ' ',
    'S', 'y', 's', 't', 'e', 'm', 'V', 'i', 'e', 'w',
    ' ', 'V', '0' + SEGGER_SYSVIEW_MAJOR,
    '.', '0' + (SEGGER_SYSVIEW_MINOR / 10),
    '0' + (SEGGER_SYSVIEW_MINOR % 10),
    '.', '0' + (SEGGER_SYSVIEW_REV / 10),
    '0' + (SEGGER_SYSVIEW_REV % 10),
    '\0', 0, 0, 0, 0, 0};

void SystemViewLaunch(void) {

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    SEGGER_SYSVIEW_Conf();

    STATIC_MEM_TASK_CREATE(systemview_task, SystemViewTask, "SystemViewTask", NULL, SYSTEMVIEW_TASK_PRI);
}

void SystemViewTask(void *param) {
    system_view_notify = xTaskGetCurrentTaskHandle();

    // 获取Channel ID
    int channel_id = SEGGER_SYSVIEW_GetChannelID();

    //发送HELLO包
    HAL_UART_Transmit_DMA(&huart1, hello_message, 32);

    uint8_t rx_buf;
    uint8_t tx_buf[SYSVIEW_SINGLE_TX];
    uint32_t notify_flag;
    bool dma_in_progress = true;
    TickType_t prev_send;

    //启动记录
    SEGGER_SYSVIEW_Start();

    while (1) {
        if (xTaskNotifyWait(0x00, 0x03, &notify_flag, pdMS_TO_TICKS(400)) == pdTRUE) {
            if (notify_flag & 0x01) {
                SEGGER_RTT_WriteDownBufferNoLock(channel_id, &rx_buf, 0x01);
                HAL_UART_Receive_IT(&huart1, &rx_buf, 0x01);
            } else if (notify_flag & 0x02) {
                if (dma_in_progress) {
                    dma_in_progress = false;
                    prev_send       = xTaskGetTickCount();
                }
            }
        }

        if (dma_in_progress == false && xTaskGetTickCount() - prev_send >= pdMS_TO_TICKS(400)) {
            unsigned int tx_length = SEGGER_RTT_GetBytesInBuffer(channel_id);
            if (tx_length >= SYSVIEW_SINGLE_TX) {
                uint32_t num = SEGGER_RTT_ReadUpBufferNoLock(channel_id, tx_buf, SYSVIEW_SINGLE_TX);
                HAL_UART_Transmit_DMA(&huart1, tx_buf, num);
            } else if (tx_length != 0) {
                uint32_t num = SEGGER_RTT_ReadUpBufferNoLock(channel_id, tx_buf, tx_length);
                HAL_UART_Transmit_DMA(&huart1, tx_buf, num);
            }
            dma_in_progress = true;
        }

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        BaseType_t higher_woken = pdFALSE;
        xTaskNotifyFromISR(system_view_notify, 0x01, eSetBits, &higher_woken);
        portYIELD_FROM_ISR(higher_woken);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        BaseType_t higher_woken = pdFALSE;
        xTaskNotifyFromISR(system_view_notify, 0x02, eSetBits, &higher_woken);
        portYIELD_FROM_ISR(higher_woken);
    }
}
```

修改`STM32_Project\Src\main.c`函数：

```
...
#include "systemview_task.h"
...

...
/* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  SystemViewLaunch();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();
...
```

修改`STM32_Project\.vscode\c_cpp_properties.json`，添加：

```
...
"____________________USER_FIELDS_CAN_BE_MODIFIED____________________": "",
        "user_cSources": [
            "Modules/src/usart_com.c",
            "Modules/src/motor.c",
            "Modules/src/mavlink_task.c",
            "Modules/src/utils.c",
            "Modules/src/mpu6050.c",
            "Modules/src/ms5611.c",
            "Modules/src/hmc5883l.c",
            "Modules/src/sensors.c",
            "Modules/src/i2c1.c",
            "Modules/src/dmp.c",
            "Modules/src/system_task.c",
            "Modules/src/systemview_task.c",
            "Middlewares/Third_Party/SystemView/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c",
            "Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.c",
            "Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.c",
            "Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c",
            "Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.c",
            "Middlewares/Third_Party/SystemView/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c"
        ],
        "user_asmSources": [
            "Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.s"
        ],
        "user_ldSources": [],
        "user_cIncludes": [
            "Modules/include",
            "Middlewares/Third_Party/SystemView/Config",
            "Middlewares/Third_Party/SystemView/SEGGER"
        ],
        "user_asmIncludes": [],
        "user_ldIncludes": [],
        "user_cDefines": [],
        "user_asmDefines": [],
        "user_cFlags": [],
        "user_asmFlags": [],
        "user_ldFlags": [
            "-u _printf_float"
        ],
...
```

编译并且烧录到单片机中。

### SystemView

打开`SystemView`软件，打开`Target->Recorder Configuration`，选择`UART`并且填好相应的串口和波特率。

点击`Start Recording`，重启单片机，成功监控FreeRTOS：



[^1]: https://github.com/nghiaphamsg/STM32F4_FreeRTOS/blob/master/002_SEGGER_Tools/README.md
[^2]: https://www.segger.com/downloads/systemview/UM08027
[^3]: https://blog.imi.moe/systemview-freertos/