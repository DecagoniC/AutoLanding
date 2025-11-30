#pragma once
#include "PrettyPrint.h"
#include <stdexcept>
#include <iostream>

template <typename T>
class DynamicArray {
private:
    T* data;
    int length;
    int capacity;
public:
    DynamicArray() : data(nullptr), length(0), capacity(0) {}
    DynamicArray(int size) : capacity(size), length(size) {
        if (size < 0) {
            throw std::out_of_range("Size cannot be negative");
        }
        if (size == 0) {
            data = nullptr;
        }
        else {
            data = new T[size];
            for (int i = 0; i < size; i++) {
                data[i] = T();
            }
        }
    }
    DynamicArray(T* items, int size) : data(new T[size]), capacity(size), length(0) {
        for (int i = 0; i < size; i++) {
            data[i] = items[i];
            length++;
        }
    }
    DynamicArray(const DynamicArray<T>& other) : data(new T[other.length]), length(other.length), capacity(other.capacity) {
        for (int i = 0; i < length; i++) {
            data[i] = other.Get(i);
        }
    }
    ~DynamicArray() {
        delete[] data;
    }
    T Get(int index) const {
        if (index < 0 || index >= length) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }
    T GetLast() const {
        return Get(length - 1);
    }
    T GetFirst() const {
        return Get(0);
    }
    int GetLength() const {
        return length;
    }
    int GetCapacity() const {
        return capacity;
    }
    void To_String() const {
        static_assert(
            std::is_same_v<decltype(std::declval<std::ostream&>() << std::declval<T>()), std::ostream&>,
            "T must be printable with std::ostream"
            );
        if (length != 0) std::cout << "[";
        else std::cout << "[]\n";
        for (int i = 0; i < GetLength(); i++) {
            if (i == GetLength() - 1) {
                std::cout << Get(i) << "]\n";
            }
            else {
                std::cout << Get(i) << "; ";
            }
        }
    }
    void To_String_S() const {
        if (length != 0) std::cout << "[";
        else std::cout << "[]";
        for (int i = 0; i < GetLength(); i++) {
            if (i == GetLength() - 1) {
                std::cout << Get(i) << "]";
            }
            else {
                std::cout << Get(i) << "; ";
            }
        }
    }
    DynamicArray<T>* GetSubArray(int startIndex, int endIndex) const {
        if (startIndex < 0 || endIndex >= GetLength() || startIndex > endIndex) {
            throw std::out_of_range("Invalid subsequence range");
        }
        DynamicArray<T>* SubArr = new DynamicArray<T>(endIndex - startIndex + 1);
        for (int i = startIndex; i <= endIndex; i++) {
            (*SubArr)[i - startIndex] = (*this)[i];
        }
        return SubArr;
    }
    void Set(T value, int index) {
        if (index < 0 || index >= length) {
            throw std::out_of_range("Index out of bounds");
        }
        data[index] = value;
    }
    void InsertAt(T item, int index) {
        if (index < 0 || index > length) {
            throw std::out_of_range("Index out of bounds");
        }
        if (capacity < length + 1) {
            Resize(std::max(1, 2 * capacity));
        }
        length++;
        T* newdata = new T[capacity];
        for (int i = length - 1; i >= 0; i--) {
            if (i > index) {
                newdata[i] = data[i - 1];
            }
            else {
                if (i == index) {
                    newdata[i] = item;
                }
                else newdata[i] = data[i];
            }
        }
        delete[] data;
        data = newdata;
    }
    void Resize(int newSize) {
        if (newSize < 0) {
            throw std::out_of_range("Size cannot be negative");
        }
        T* newData = new T[newSize];
        int copySize;
        if (newSize < length) copySize=newSize;
        else copySize=length;
        for (int i = 0; i < copySize; i++) {
            newData[i] = data[i];
        }
        for (int i = copySize; i < newSize; i++) {
            newData[i] = T();
        }
        delete[] data;
        data = newData;
        capacity = newSize;
        if (newSize < length) {
            length = newSize;
        }
    }
    void Append(T value) {
        if (capacity < length + 1) {
            if (capacity == 0) Resize(1);
            else Resize(2 * capacity);
        }
        data[length] = value;
        length++;
    }
    void Prepend(T value) {
        if (capacity < length + 1) {
            if (capacity == 0) Resize(1);
            else Resize(2 * capacity);
        }
        for (int i = length - 1; i >= 0; i--) {
            data[i + 1] = data[i];
        }
        data[0] = value;
        length++;
    }

    DynamicArray<T> Concat(const DynamicArray<T>& list) const {
        DynamicArray<T> result(*this);
        for (int i = 0; i < list.GetLength(); i++) {
            result.Append(list.Get(i));
        }
        return result;
    }
    T& operator[](int index) {
        if (index >= length) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }
    const T& operator[](int index) const {
        if (index >= length) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }
    DynamicArray<T>& operator=(const DynamicArray<T>& other) {
        if (this == &other) {
            return *this;
        }
        delete[] data;
        length = other.length;
        capacity = other.capacity;
        data = new T[capacity];
        for (int i = 0; i < length; i++) {
            data[i] = other[i];
        }
        return *this;
    }
    void To_PrettyString(const PrettyPrinter& printer=prettyprinter, std::ostream& os = std::cout) const {
        static_assert(
            std::is_same_v<decltype(std::declval<std::ostream&>() << std::declval<T>()), std::ostream&>,
            "T must be printable with std::ostream"
            );
        os << "[";
        if (length > 0) {
            for (int i = 0; i < length; ++i) {
                os << Get(i);
                if (i < length - 1) {
                    os << printer.getSeparator();
                }
            }
        }
        os << "]\n";
    }
    friend std::ostream& operator<<(std::ostream& os, const DynamicArray<T>& arr) {
        const PrettyPrinter& printer = prettyprinter;
        os << "[";
        for (int i = 0; i < arr.GetLength(); ++i) {
            os << arr.Get(i);
            if (i < arr.GetLength() - 1) {
                os << printer.getSeparator();
            }
        }
        os << "]";
        return os;
    }
};