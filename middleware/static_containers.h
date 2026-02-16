/**
 * @file static_containers.h
 * @brief Lock-free, heap-free containers for hard real-time.
 *
 * Every container here is fixed-capacity, stack/static allocated.
 * If you need std::vector, you're in the wrong codebase.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace bottling {

// ─────────────────────────────────────────────────────────────────────────────
// STATIC RING BUFFER — single-thread use (within one RT task)
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, size_t N>
class StaticRingBuffer {
    static_assert((N & (N - 1)) == 0, "Capacity must be power of 2 for fast modulo");
    static_assert(N >= 4, "Minimum capacity is 4");

    std::array<T, N> buffer_{};
    size_t head_ = 0;
    size_t tail_ = 0;

    static constexpr size_t mask() { return N - 1; }

public:
    [[nodiscard]] bool push(const T& item) {
        if (full()) return false;
        buffer_[head_ & mask()] = item;
        ++head_;
        return true;
    }

    [[nodiscard]] bool pop(T& item) {
        if (empty()) return false;
        item = buffer_[tail_ & mask()];
        ++tail_;
        return true;
    }

    [[nodiscard]] const T& peek() const    { return buffer_[tail_ & mask()]; }
    [[nodiscard]] bool empty() const       { return head_ == tail_; }
    [[nodiscard]] bool full() const        { return (head_ - tail_) >= N; }
    [[nodiscard]] size_t size() const      { return head_ - tail_; }
    [[nodiscard]] size_t capacity() const  { return N; }
    void clear()                           { head_ = tail_ = 0; }
};

// ─────────────────────────────────────────────────────────────────────────────
// SPSC QUEUE — wait-free, lock-free, cross-task communication
//
// Producer and consumer run on DIFFERENT RT tasks (potentially different cores).
// Cache-line aligned to prevent false sharing.
//
// Memory ordering:
//   Producer: store data → release-store head
//   Consumer: acquire-load head → load data
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, size_t N>
class SPSCQueue {
    static_assert((N & (N - 1)) == 0, "Capacity must be power of 2");
    static_assert(std::is_trivially_copyable_v<T>,
                  "SPSC elements must be trivially copyable for lock-free safety");

    static constexpr size_t kCacheLineSize = 64;
    static constexpr size_t mask() { return N - 1; }

    // Each index on its own cache line — eliminates false sharing
    alignas(kCacheLineSize) std::atomic<size_t> head_{0};
    alignas(kCacheLineSize) std::atomic<size_t> tail_{0};
    alignas(kCacheLineSize) std::array<T, N>    buffer_{};

public:
    /**
     * @brief Push from producer task. Wait-free, constant time.
     * @return true if enqueued, false if full (back-pressure signal).
     */
    [[nodiscard]] bool push(const T& item) {
        const size_t h = head_.load(std::memory_order_relaxed);
        const size_t t = tail_.load(std::memory_order_acquire);

        if ((h - t) >= N) return false;  // full

        buffer_[h & mask()] = item;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop from consumer task. Wait-free, constant time.
     * @return true if dequeued, false if empty.
     */
    [[nodiscard]] bool pop(T& item) {
        const size_t t = tail_.load(std::memory_order_relaxed);
        const size_t h = head_.load(std::memory_order_acquire);

        if (t == h) return false;  // empty

        item = buffer_[t & mask()];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    [[nodiscard]] size_t size_approx() const {
        const size_t h = head_.load(std::memory_order_acquire);
        const size_t t = tail_.load(std::memory_order_acquire);
        return h - t;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// STATIC VECTOR — fixed-capacity, variable-length, no heap
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, size_t N>
class StaticVector {
    alignas(T) uint8_t storage_[N * sizeof(T)];
    size_t size_ = 0;

    T*       data()       { return reinterpret_cast<T*>(storage_); }
    const T* data() const { return reinterpret_cast<const T*>(storage_); }

public:
    [[nodiscard]] bool push_back(const T& item) {
        if (size_ >= N) return false;
        new (&data()[size_]) T(item);
        ++size_;
        return true;
    }

    template<typename... Args>
    [[nodiscard]] bool emplace_back(Args&&... args) {
        if (size_ >= N) return false;
        new (&data()[size_]) T(std::forward<Args>(args)...);
        ++size_;
        return true;
    }

    void pop_back() {
        if (size_ > 0) {
            --size_;
            data()[size_].~T();
        }
    }

    void clear() {
        for (size_t i = 0; i < size_; ++i) data()[i].~T();
        size_ = 0;
    }

    T&       operator[](size_t i)       { return data()[i]; }
    const T& operator[](size_t i) const { return data()[i]; }

    [[nodiscard]] size_t size() const     { return size_; }
    [[nodiscard]] size_t capacity() const { return N; }
    [[nodiscard]] bool   empty() const    { return size_ == 0; }
    [[nodiscard]] bool   full() const     { return size_ >= N; }

    T*       begin()       { return data(); }
    T*       end()         { return data() + size_; }
    const T* begin() const { return data(); }
    const T* end() const   { return data() + size_; }

    ~StaticVector() { clear(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// ENCODER-DRIVEN BIT SHIFT REGISTER — bottle tracking by position
//
// Each bit = one encoder pulse. Set bit at detection → shift on each pulse →
// fire reject when bit reaches reject station position.
// ─────────────────────────────────────────────────────────────────────────────
template<size_t BitCount>
class BitShiftRegister {
    static_assert(BitCount <= 65536, "Shift register too large");
    static constexpr size_t kWordCount = (BitCount + 63) / 64;

    std::array<uint64_t, kWordCount> bits_{};
    size_t length_ = BitCount;

public:
    /**
     * @brief Shift all bits one position (called on each encoder pulse).
     *        Returns true if the MSB was set (bottle arrived at action point).
     */
    [[nodiscard]] bool shift() {
        bool carry_out = (bits_[kWordCount - 1] >> 63) & 1;

        for (size_t i = kWordCount - 1; i > 0; --i) {
            bits_[i] = (bits_[i] << 1) | (bits_[i - 1] >> 63);
        }
        bits_[0] <<= 1;

        return carry_out;
    }

    /**
     * @brief Set bit at position 0 (inject a new bottle/reject flag).
     */
    void inject(bool flag) {
        if (flag) bits_[0] |= 1ULL;
        else      bits_[0] &= ~1ULL;
    }

    void clear() { bits_.fill(0); }

    [[nodiscard]] bool test(size_t pos) const {
        if (pos >= BitCount) return false;
        return (bits_[pos / 64] >> (pos % 64)) & 1;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// MEMORY POOL — fixed-block allocator for alarm records, etc.
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, size_t N>
class MemoryPool {
    struct Block {
        alignas(T) uint8_t data[sizeof(T)];
    };

    std::array<Block, N> pool_{};
    std::array<bool, N>  used_{};
    size_t next_free_ = 0;

public:
    MemoryPool() { used_.fill(false); }

    T* allocate() {
        for (size_t i = next_free_; i < N; ++i) {
            if (!used_[i]) {
                used_[i] = true;
                next_free_ = i + 1;
                return reinterpret_cast<T*>(&pool_[i]);
            }
        }
        // Wrap-around search
        for (size_t i = 0; i < next_free_; ++i) {
            if (!used_[i]) {
                used_[i] = true;
                next_free_ = i + 1;
                return reinterpret_cast<T*>(&pool_[i]);
            }
        }
        return nullptr;  // pool exhausted
    }

    void deallocate(T* ptr) {
        auto* byte_ptr = reinterpret_cast<uint8_t*>(ptr);
        auto* base     = reinterpret_cast<uint8_t*>(&pool_[0]);
        const size_t index = static_cast<size_t>(byte_ptr - base) / sizeof(Block);
        if (index < N) {
            ptr->~T();
            used_[index] = false;
            if (index < next_free_) next_free_ = index;
        }
    }

    [[nodiscard]] size_t available() const {
        size_t count = 0;
        for (size_t i = 0; i < N; ++i) {
            if (!used_[i]) ++count;
        }
        return count;
    }
};

}  // namespace bottling
