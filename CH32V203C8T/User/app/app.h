#ifndef __APP_APP_H__
#define __APP_APP_H__

#include <stdint.h>

/**
 * @brief 初始化应用层模块
 */
void App_Init(void);

/**
 * @brief 应用主循环，永不返回
 */
void App_Loop(void);

#endif // ifndef __APP_APP_H__
