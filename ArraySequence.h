#pragma once
#include "Sequence.h"
#include "DynamicArray.h"

template <typename T>
class ArraySequence : public Sequence<T> {
protected:
    DynamicArray<T> array;
    ArraySequence(DynamicArray<T>* arr) : array(arr) {}
public:
    ArraySequence() : array(DynamicArray<T>()) {}
    ArraySequence(int count) : array(DynamicArray<T>(count)) {}
    ArraySequence(T* items, int count) : array(DynamicArray<T>(items, count)) {}
    ArraySequence(const ArraySequence<T>& other) : array(DynamicArray<T> (other.array)) {}
    ArraySequence(const DynamicArray<T>& list) : array(list) {}
    ~ArraySequence() = default;
    T GetFirst() const override {
        return array.GetFirst();
    }
    T GetLast() const override {
        return array.GetLast();
    }
    T Get(int index) const override {
        return array.Get(index);
    }
    int GetLength() const override {
        return array.GetLength();
    }
    void To_String()const {
        array.To_String();
    }
    Sequence<T>* GetSubsequence(int start, int end) const override {
        DynamicArray<T>* subArray = array.GetSubArray(start, end);
        ArraySequence<T>* result = new ArraySequence<T>(*subArray);
        delete subArray;
        return result;
    }
    Sequence<T>* Concat(Sequence<T>* other) const {
        ArraySequence<T>* result = new ArraySequence<T>(*this);
        for (int i = 0; i < other->GetLength(); i++) {
            result->Append(other->Get(i));
        }
        return result;
    }
    int GetCapacity() {
        return array.GetCapacity();
    }
    Sequence<T>* Set(T value, int index)override {
        this->array.Set(value,index);
        return this;
    }
    Sequence<T>* Append(T item)override {
        this->array.Append(item);
        return this;
    }
    Sequence<T>* Prepend(T item)override {
        this->array.Prepend(item);
        return this;
    }
    Sequence<T>* InsertAt(T item, int index)override {
        this->array.InsertAt(item, index);
        return this;
    }
    Sequence<T>* Map(Func_T<T> func) const override{
        ArraySequence<T>* result = new ArraySequence<T>(*this);
        for (int i = 0; i < array.GetLength(); i++) {
            result->array[i] = func(array[i]);
        }
        return result;
    }
    Sequence<T>* Where(Func_T_bool<T> predicate) const override {
        int count = 0;
        for (int i = 0; i < array.GetLength(); i++) {
            if (predicate(array[i])) {
                count++;
            }
        }
        ArraySequence<T>* result = new ArraySequence<T>(count);
        int pos = 0;
        for (int i = 0; i < array.GetLength(); i++) {
            if (predicate(array[i])) {
                result->array.Set(array[i], pos);
                pos++;
            }
        }
        return result;
    }
    T Reduce(Func_T_T<T> func, T start) const override {
        T result=start;
        for (int i = 0; i < array.GetLength(); i++) {
            result = func(array[i], result);
        }
        return result;
    }
    T& operator[](int index) {
        if (index >= array.GetLength()) {
            throw std::out_of_range("Index out of bounds");
        }
        return array[index];
    }
    const T& operator[](int index) const{
        if (index >= array.GetLength()) {
            throw std::out_of_range("Index out of bounds");
        }
        return array[index];
    }
    void To_PrettyString(const PrettyPrinter& printer = prettyprinter, std::ostream& os = std::cout) const {
        array.To_PrettyString(printer, os);
    }
    friend std::ostream& operator<<(std::ostream& os, const ArraySequence<T>& arr) {
        os << arr.array;
        return os;
    }
};