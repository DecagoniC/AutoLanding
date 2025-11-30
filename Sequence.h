#pragma once
#include <cassert>
#include <functional>
#include <stdexcept>
template <typename T>
using Func_T = std::function<T(T)>;
template <typename T>
using Func_T_T = std::function<T(T, T)>;
template <typename T>
using Func_T_bool = std::function<bool(T)>;

template <typename T>
class Sequence {
public:
    virtual T GetFirst() const = 0;
    virtual T GetLast() const = 0;
    virtual T Get(int index) const = 0;
    virtual int GetLength() const = 0;

    virtual Sequence<T>* Set(T value, int index) = 0;
    virtual Sequence<T>* Append(T item) = 0;
    virtual Sequence<T>* Prepend(T item) = 0;
    virtual Sequence<T>* InsertAt(T item, int index) = 0;
    virtual Sequence<T>* GetSubsequence(int start, int end) const = 0;
    virtual Sequence<T>* Concat(Sequence<T>* other) const = 0;

    virtual Sequence<T>* Map(Func_T<T> func) const = 0;
    virtual Sequence<T>* Where(Func_T_bool<T> predicate) const = 0;
    virtual T Reduce(Func_T_T<T> func, T start) const = 0;
    struct IndexOutOfRange : public std::runtime_error {
        IndexOutOfRange(const std::string& msg = "Index out of range") : std::runtime_error(msg) {}
    };

    
};