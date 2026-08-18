#pragma once
/**
 * RingWindow.h
 *
 * Lightweight circular buffer a.k.a. ring window class
 */

#include <Arduino.h>

template <typename T, size_t CAPACITY> class RingWindow {
public:
  /**
   * @brief Constructs an empty window with every slot zeroed.
   */
  RingWindow() : filled_(0), head_(0) {
    for (size_t i = 0; i < CAPACITY; ++i)
      buf_[i] = T();
  }
  RingWindow(const RingWindow &) = delete;
  RingWindow &operator=(const RingWindow &) = delete;

  /**
   * @brief Empties the window, discarding every stored sample.
   *
   * Leaves the object in the same state as a freshly constructed one, so it is
   * safe to call when re-initialising the owner.
   */
  void clear() {
    filled_ = 0;
    head_ = 0;
    for (size_t i = 0; i < CAPACITY; ++i)
      buf_[i] = T();
  }

  /**
   * @brief Writes one sample, overwriting the oldest once the window is full.
   *
   * @param s sample to store
   */
  void add(T s) {
    buf_[head_] = s;                // write sample to head
    head_ = (head_ + 1) % CAPACITY; // advance head (modulo % ensures we circle
                                    // back to 0 when we hit cap)
    if (filled_ < CAPACITY)
      ++filled_; // increment filled until buffer is full (then filled equals
                 // cap)
  }

  /**
   * @brief Whether the window holds at least one sample.
   *
   * NOTE: this is not "is the window full" - it is true as soon as a single
   * sample has been added. Use it to guard the accessors below, which return
   * a default-constructed value on an empty window.
   *
   * @return true if one or more samples have been added since the last clear()
   */
  bool isFilled() const { return filled_ > 0; }

  /**
   * @brief Number of valid samples currently stored.
   *
   * @return sample count, never greater than CAPACITY
   */
  size_t size() const { return filled_; }

  /**
   * @brief Smallest of the stored samples.
   *
   * @return the minimum, or a default-constructed T if the window is empty
   */
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

  /**
   * @brief Largest of the stored samples.
   *
   * @return the maximum, or a default-constructed T if the window is empty
   */
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

  /**
   * @brief Mean of the stored samples.
   *
   * Averages over the samples actually present, not over CAPACITY, so the
   * result is meaningful before the window has filled.
   *
   * @return the mean as a float, or 0.0f if the window is empty
   */
  float average() const {
    if (filled_ == 0)
      return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < filled_; ++i)
      sum += buf_[i];
    return sum / static_cast<float>(filled_);
  }

  /**
   * @brief Median of the stored samples.
   *
   * Copies into a fixed-size stack array and insertion sorts it, so cost grows
   * with the square of the sample count - fine for the small windows used here.
   *
   * @return the median, or a default-constructed T if the window is empty
   */
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
