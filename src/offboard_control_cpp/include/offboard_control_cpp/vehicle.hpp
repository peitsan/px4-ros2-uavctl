/*
 * @file vehicle.hpp
 * @brief 无人机载具管理类头文件
 * @details 该文件定义了Vehicle类，用于管理ROS2中的无人机节点和执行器
 * @author [Author Name]
 * @date 2026-01-28
 */

#pragma once

// 标准库头文件
#include <memory>      // 用于智能指针（std::shared_ptr）
#include <thread>      // 用于多线程操作

// ROS2相关头文件
#include <rclcpp/rclcpp.hpp>                                  // ROS2客户端库主头文件
#include <rclcpp/executors/multi_threaded_executor.hpp>      // 多线程执行器，用于并发执行回调

/**
 * @class OffboardControl
 * @brief 前向声明OffboardControl类
 * @details 该类用于实现无人机的离板控制功能，具体定义在其他文件中
 */
class OffboardControl;

/**
 * @class Vehicle
 * @brief 无人机载具管理类
 * @details 该类负责管理无人机的ROS2节点生命周期、执行器和相关资源
 *          包括无人机控制对象、多线程执行器和自旋线程的管理
 */
class Vehicle {
public:
    /**
     * @brief Vehicle类的构造函数
     * @details 初始化无人机对象、ROS2执行器和自旋线程
     *          创建OffboardControl实例用于无人机控制
     */
    Vehicle();

    /**
     * @brief Vehicle类的析构函数
     * @details 自动清理资源，包括关闭执行器和停止自旋线程
     */
    ~Vehicle();

    /**
     * @brief 关闭无人机和相关资源
     * @details 停止ROS2执行器，等待自旋线程结束
     *          标记为已关闭状态，防止重复关闭
     * @return void
     */
    void close();

    /**
     * @brief 获取无人机控制对象指针
     * @details 返回OffboardControl的智能指针，用于调用无人机控制方法
     * @return std::shared_ptr<OffboardControl> 无人机控制对象的共享指针
     */
    std::shared_ptr<OffboardControl> drone() { return drone_; }

private:
    // ============ 成员变量 ============

    /**
     * @brief 无人机离板控制对象
     * @details 使用共享指针管理，负责实现无人机的各项控制功能
     */
    std::shared_ptr<OffboardControl> drone_;

    /**
     * @brief ROS2多线程执行器
     * @details 用于并发执行ROS2订阅、服务等回调函数
     *          支持多线程环境下的并发处理
     */
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    /**
     * @brief 自旋线程
     * @details 独立线程用于运行ROS2执行器的自旋循环
     *          使主程序线程不被阻塞
     */
    std::thread spin_thread_;

    /**
     * @brief 关闭状态标志
     * @details true表示已关闭，false表示正在运行
     *          用于防止重复关闭和异常关闭
     */
    bool closed_ = false;
};