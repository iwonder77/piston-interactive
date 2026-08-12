#pragma once
/**
 * RingWindow.h
 *
 * Lightweight circular buffer a.k.a. ring window class
 */

#include <Arduino.h>

template <typename T, size_t CAPACITY> class RingWindow {
public:
  RingWindow() : filled_(0), head_(0) {
    for (size_t i = 0; i < CAPACITY; ++i)
      buf_[i] = T();
  }
  RingWindow(const RingWindow &) = delete;
  RingWindow &operator=(const RingWindow &) = delete;

  void clear() {
    filled_ = 0;
    head_ = 0;
    for (size_t i = 0; i < CAPACITY; ++i)
      buf_[i] = T();
  }

  // write new sample to buffer
  void add(T s) {
    buf_[head_] = s;                // write sample to head
    head_ = (head_ + 1) % CAPACITY; // advance head (modulo % ensures we circle
                                    // back to 0 when we hit cap)
    if (filled_ < CAPACITY)
      ++filled_; // increment filled until buffer is full (then filled equals
                 // cap)
  }

  // checks if any samples have been written to buffer
  bool isFilled() const { return filled_ > 0; }

  size_t size() const { return filled_; }

  T getMin() const {
    if (filled_ == 0)
      return T();
    T m = buf_[0];
    for (size_t i = 0; i < filled_; ++i) {
      if (buf_[i] < m)
        m = buf_[i];
    }
    return m;
  }

  T getMax() const {
    if (filled_ == 0)
      return T();
    T m = buf_[0];
    for (size_t i = 0; i < filled_; ++i) {
      if (buf_[i] > m)
        m = buf_[i];
    }
    return m;
  }

  // return average
  float average() const {
    if (filled_ == 0)
      return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < filled_; ++i)
      sum += buf_[i];
    return sum / float(filled_);
  }

  T median() const {
    if (filled_ == 0)
      return T();

    // copy data to temporary fixed-size stack array for sorting
    T temp[CAPACITY];
    for (size_t i = 0; i < filled_; ++i) {
      temp[i] = buf_[i];
    }

    // insertion sort
    for (size_t i = 1; i < filled_; i++) {
      T v = temp[i];
      size_t j = i;
      while (j > 0 && temp[j - 1] > v) {
        temp[j] = temp[j - 1];
        j--;
      }
      temp[j] = v;
    }

    T result;
    if (filled_ % 2 == 0) {
      result = (temp[filled_ / 2 - 1] + temp[filled_ / 2]) / 2;
    } else {
      result = temp[filled_ / 2];
    }
    return result;
  }

private:
  T buf_[CAPACITY]; // fixed size array on stack
  size_t filled_;   // slots in buffer currently filled with valid data
  size_t head_;     // index where the NEXT write goes to
};
