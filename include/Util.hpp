#pragma once

#include <Arduino.h>

template <typename T, size_t Size> class Array {
public:
  template <typename... Args> explicit Array(Args... args) : data{T(args)...} {}

  class Iterator {
  private:
    T *ptr;

  public:
    Iterator(T *ptr) : ptr(ptr) {}

    Iterator &operator++() {
      ptr++;
      return *this;
    }

    bool operator!=(const Iterator &other) const { return ptr != other.ptr; }

    // Overload the dereference operator (*iterator)
    T &operator*() const { return *ptr; }
  };

  T &operator[](size_t index) { return data[index]; }

  const T &operator[](size_t index) const { return data[index]; }

  constexpr static unsigned size() { return Size; }

  Iterator begin() { return Iterator(&data[0]); }

  Iterator end() { return Iterator(&data[Size]); }

private:
  T data[Size];
};
