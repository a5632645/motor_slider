/**
 * @file    kfifo.h
 * @brief   字节型无锁环形缓冲区（KFIFO）
 *
 * 适用于单生产者/单消费者场景的无锁 FIFO 缓冲区。
 * 支持连续读取操作（ContinueRead）以最小化数据拷贝。
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/**
 * @brief KFIFO 环形缓冲区结构
 */
struct Kfifo {
    uint32_t wpos;  /**< 写指针（非取模） */
    uint32_t rpos;  /**< 读指针（非取模） */
    uint32_t mask;  /**< data 数组长度 - 1（须为 2^n - 1） */
    uint8_t data[]; /**< 柔性数组成员，实际缓冲区 */
};

/**
 * @brief 获取缓冲区中有效数据长度
 * @param f KFIFO 指针
 * @return 有效字节数
 */
static inline uint32_t Kfifo_Size(struct Kfifo* f) {
    return f->wpos - f->rpos;
}

/**
 * @brief 获取缓冲区剩余空闲空间
 * @param f KFIFO 指针
 * @return 空闲字节数
 */
static inline uint32_t Kfifo_FreeSpace(struct Kfifo* f) {
    return (f->mask + 1) - Kfifo_Size(f);
}

/**
 * @brief 检查缓冲区是否已满
 * @param f KFIFO 指针
 * @return true 已满, false 未满
 */
static inline bool Kfifo_IsFull(struct Kfifo* f) {
    return (f->wpos - f->rpos) == (f->mask + 1);
}

/**
 * @brief 检查缓冲区是否为空
 * @param f KFIFO 指针
 * @return true 为空, false 非空
 */
static inline bool Kfifo_IsEmpty(struct Kfifo* f) {
    return f->wpos == f->rpos;
}

/**
 * @brief 向缓冲区写入一个字节
 * @param f KFIFO 指针
 * @param v 待写入字节
 * @note 不检查空间是否充足，调用前请确保有空闲空间
 */
static inline void Kfifo_Push(struct Kfifo* f, uint8_t v) {
    f->data[f->wpos & f->mask] = v;
    asm volatile("" ::: "memory");
    ++f->wpos;
}

/**
 * @brief 从缓冲区读取一个字节
 * @param f KFIFO 指针
 * @return 读取到的字节
 * @note 不检查是否有数据，调用前请确认非空
 */
static inline uint8_t Kfifo_Pop(struct Kfifo* f) {
    uint8_t v = f->data[f->rpos & f->mask];
    asm volatile("" ::: "memory");
    ++f->rpos;
    return v;
}

/**
 * @brief 尝试向缓冲区写入多个字节
 * @param f     KFIFO 指针
 * @param src   源数据指针
 * @param count 待写入字节数
 * @return 实际写入字节数（可能小于 count 如果空间不足）
 */
static inline uint32_t Kfifo_TryPush(struct Kfifo* f, const uint8_t* src, uint32_t count) {
    uint32_t free = Kfifo_FreeSpace(f);
    if (count > free)
        count = free;

    uint32_t wpos = f->wpos & f->mask;
    uint32_t till_end = (f->mask + 1) - wpos;

    if (count <= till_end) {
        memcpy(&f->data[wpos], src, count);
    }
    else {
        memcpy(&f->data[wpos], src, till_end);
        memcpy(&f->data[0], src + till_end, count - till_end);
    }

    asm volatile("" ::: "memory");
    f->wpos += count;
    return count;
}

/**
 * @brief 从缓冲区读取多个字节
 * @param f     KFIFO 指针
 * @param dst   目标缓冲区指针
 * @param count 请求读取字节数
 * @return 实际读取字节数（可能小于 count）
 */
static inline uint32_t Kfifo_Read(struct Kfifo* f, uint8_t* dst, uint32_t count) {
    uint32_t size = Kfifo_Size(f);
    if (count > size)
        count = size;

    uint32_t rpos = f->rpos & f->mask;
    uint32_t till_end = (f->mask + 1) - rpos;

    if (count <= till_end) {
        memcpy(dst, &f->data[rpos], count);
    }
    else {
        memcpy(dst, &f->data[rpos], till_end);
        memcpy(dst + till_end, &f->data[0], count - till_end);
    }

    asm volatile("" ::: "memory");
    f->rpos += count;
    return count;
}

/**
 * @brief 开始连续读取（避免 memcpy 到中间缓冲区）
 * @param f     KFIFO 指针
 * @param count 输出参数：本次可连续读取的字节数
 * @return 指向可读数据的指针
 * @note 读取完成后必须调用 Kfifo_ContinueReadEnd
 */
static inline uint8_t* Kfifo_ContinueReadBegin(struct Kfifo* f, uint32_t* count) {
    uint32_t rpos = f->rpos & f->mask;
    uint32_t size = Kfifo_Size(f);
    uint32_t till_end = (f->mask + 1) - rpos;

    *count = (size < till_end) ? size : till_end;
    return &f->data[rpos];
}

/**
 * @brief 结束连续读取
 * @param f     KFIFO 指针
 * @param count 实际读取的字节数（应 <= 上一步获取的 count）
 */
static inline void Kfifo_ContinueReadEnd(struct Kfifo* f, uint32_t count) {
    asm volatile("" ::: "memory");
    f->rpos += count;
}
