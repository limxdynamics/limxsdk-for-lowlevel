# LimX SDK使用说明



## 1. Linux编译运行

- 安装依赖（`gcc版本9.4.0及以上`）

  ```
  sudo apt update
  sudo apt install build-essential cmake
  ```

- 编译运行

  - 进到源码目录，执行下面终端命令编译代码：

    ```
    limx@limx:/$ mkdir build 
    limx@limx:/$ cd build
    limx@limx:/build$ cmake -DEXAMPLE=ON ..
    limx@limx:/build$ make
    ```

  - 运行程序

    ```
    limx@limx:/build$ ./examples/wl_standup
    ```



## 2. Windows编译运行

- 安装依赖

  - 安装Windows包管理器

    ```
    > Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
    ```

  - 安装cmake

    ```
    > choco install -y cmake
    ```

  - 安装Visual Studio 2019

    下载安装文件：https://visualstudio.microsoft.com/zh-hans/thank-you-downloading-visual-studio/?sku=Community&rel=16&src=myvs&utm_medium=microsoft&utm_source=my.visualstudio.com&utm_campaign=download&utm_content=vs+community+2019 ，选择 ”Desktop development with C++” 进行安装

- 编译运行

  - 打开命令行终端：Developer Command Prompt for VS 2019

  - 进到源码目录，执行下面终端命令编译代码：

    ```
    D:\limx_robot_sdk> mkdir build
    D:\limx_robot_sdk> cd build
    D:\limx_robot_sdk\build> cmake -DEXAMPLE=ON ..
    D:\limx_robot_sdk\build> cmake --build . --config Release
    ```

  - 运行程序

    ```
    D:\limx_robot_sdk\build> .\Release\wl_standup.exe
    ```




# 4. 轮足接口介绍

```
/**
 * @file wheel_legged.h
 *
 * @brief This file contains the declarations of classes related to the control of wheeled-legged robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_WHEEL_LEGGED_H_
#define _LIMX_SDK_WHEEL_LEGGED_H_

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk {
  /**
   * @brief Class for controlling a wheel-legged robot using the LIMX SDK API.
   */
  class LIMX_SDK_API WheelLegged : public ApiBase {
    public:
      /**
       * @brief Get an instance of the WheelLegged class.
       * @return A pointer to a WheelLegged instance (Singleton pattern).
       */
      static WheelLegged* getInstance();

      /**
       * @brief Pure virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set   to "127.0.0.1",
       *                         while for a real robot, it may be set to "192.168.1.2".
       * @return True if init successfully, otherwise false.
       */
      bool init(const std::string& robot_ip_address = "127.0.0.1") override;

      /**
       * @brief Get the number of motors in the robot.
       * @return The total number of motors.
       */
      uint32_t getMotorNumber() override;

      /**
       * @brief Subscribe to receive updates about the robot state.
       * The motor order for the state data is as follows:
       *        0: LF_HAA,  1: LF_HFE,  2: LF_KFE,  3: LF_WHL
       *        4: LH_HAA,  5: LH_HFE,  6: LH_KFE,  7: LH_WHL
       *        8: RF_HAA,  9: RF_HFE,  10: RF_KFE, 11: RF_WH
       *        12: RH_HAA, 13: RH_HFE, 14: RH_KFE, 15: RH_WHL
       * 
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb) override;

      /**
       * @brief Publish a command to control the robot's actions.
       * The motor order for the commnd data is as follows:
       *        0: LF_HAA,  1: LF_HFE,  2: LF_KFE,  3: LF_WHL
       *        4: LH_HAA,  5: LH_HFE,  6: LH_KFE,  7: LH_WHL
       *        8: RF_HAA,  9: RF_HFE,  10: RF_KFE, 11: RF_WH
       *        12: RH_HAA, 13: RH_HFE, 14: RH_KFE, 15: RH_WHL
       * 
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      bool publishRobotCmd(const RobotCmd& cmd) override;

      /**
       * @brief Destructor for the WheelLegged class.
       *        Cleans up any resources used by the object.
       */
      virtual ~WheelLegged();

    private:
      /**
       * @brief Private constructor to prevent external instantiation of the WheelLegged class.
       */
      WheelLegged();
  };
};

#endif
```



# 5. 点足接口介绍

```
/**
 * @file pointfoot.h
 *
 * @brief This file contains the declarations of classes related to the control of pointfoot robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_POINTFOOT_H_
#define _LIMX_SDK_POINTFOOT_H_

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk {
  /**
   * @brief Class for controlling a pointfoot robot using the LIMX SDK API.
   */
  class LIMX_SDK_API PointFoot : public ApiBase {
    public:
      /**
       * @brief Get an instance of the PointFoot class.
       * @return A pointer to a PointFoot instance (Singleton pattern).
       */
      static PointFoot* getInstance();

      /**
       * @brief Pure virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set to "127.0.0.1",
       *                         while for a real robot, it may be set to "192.168.1.2".
       * @return True if init successfully, otherwise false.
       */
      bool init(const std::string& robot_ip_address = "127.0.0.1") override;

      /**
       * @brief Get the number of motors in the robot.
       * @return The total number of motors.
       */
      uint32_t getMotorNumber() override;

      /**
       * @brief Subscribe to receive updates about the robot state.
       * The motor order for the state data is as follows:
       *        0: abad_L_joint,  1: hip_L_joint,  2: knee_L_joint
       *        3: abad_R_joint,  4: hip_R_joint,  5: knee_R_joint
       * 
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb) override;

      /**
       * @brief Publish a command to control the robot's actions.
       * The motor order for the commnd data is as follows:
       *        0: abad_L_joint,  1: hip_L_joint,  2: knee_L_joint
       *        3: abad_R_joint,  4: hip_R_joint,  5: knee_R_joint
       * 
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      bool publishRobotCmd(const RobotCmd& cmd) override;

      /**
       * @brief Destructor for the PointFoot class.
       *        Cleans up any resources used by the object.
       */
      virtual ~PointFoot();

    private:
      /**
       * @brief Private constructor to prevent external instantiation of the PointFoot class.
       */
      PointFoot();
  };
};

#endif
```

