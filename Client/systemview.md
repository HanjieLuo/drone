### 准备文件

我的STM32项目是经由`STM32CubeMX` v6.2.0生成的`Makefile`项目；使用[VS-Code-STM32-IDE](https://github.com/damogranlabs/VS-Code-STM32-IDE)来生成VS Code项目；使用v10.3.1的FreeRTOS系统；使用USART1作为输出端口，开启DMA传输。

下载[SystemView](https://www.segger.com/products/development-tools/systemview/) v3.30版本。在安装目录内的`SystemView_V330\Src`文件夹中，将`SEGGER`和`Sample`内的相关文件复制到STM32项目的文件夹`STM32_Project\Middlewares\Third_Party\SystemView`中[^1]。注意根据项目的需求，选择复制`Sample`中对应的系统和`SEGGER\Syscalls`中对应的编译器:

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

注意：`SEGGER_RTT_ASM_ARMv7M.S`需要改名为`SEGGER_RTT_ASM_ARMv7M.s`，不然后面编译时会出现`No rule to make target`错误。



修改`STM32_Project\.vscode\c_cpp_properties.json`，添加：

```
...
"____________________USER_FIELDS_CAN_BE_MODIFIED____________________": "",
        "user_cSources": [
            "Modules/src/usart2.c",
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

### 补丁

将`SystemView_V330\Src\Sample\FreeRTOSV10\Patch\FreeRTOSV10_Core.patch`复制到STM32项目下，



[^1]: https://blog.imi.moe/systemview-freertos/