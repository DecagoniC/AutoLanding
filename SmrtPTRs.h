#pragma once
#include <iostream>
#include <vector>
#include <chrono>
#include <type_traits>
#include <memory>
#include "DynamicArray.h"

template <typename T>
class ShrdPtr;

template <typename T>
class UnqPtr {
private:
    T* ptr_;
    size_t* ref_count_;

public:
    explicit UnqPtr(T* ptr = nullptr) : ptr_(ptr), ref_count_(new size_t(1)){}
    UnqPtr(UnqPtr&& other) : ptr_(other.ptr_), ref_count_(other.ref_count_) {
        other.ptr_ = nullptr;
        other.ref_count_ = nullptr;
    }
    template <typename U, typename = typename std::enable_if<std::is_base_of<T, U>::value>::type>
    UnqPtr(UnqPtr<U>&& other) : ptr_(other.ptr_), ref_count_(other.ref_count_) {
        other.ptr_ = nullptr;
        other.ref_count_ = nullptr;
    }
    UnqPtr& operator=(UnqPtr&& other) {
        if (this != &other){
            release(); ptr_ = other.ptr_;
            ref_count_ = other.ref_count_;
            other.ptr_ = nullptr;
            other.ref_count_ = nullptr;
        }
        return *this;
    }
    ~UnqPtr() { release(); }
    void release(){
        if(ref_count_ && --(*ref_count_) == 0){
            if constexpr (std::is_array_v<T>) {
                delete[] ptr_;
            } else {
                delete ptr_;
            }
            delete ref_count_;
        }
        ptr_ = nullptr;
        ref_count_ = nullptr;
    }
    T* get() const{
        return ptr_;
    }
    T& operator*() const{
        return *ptr_;
    }
    T* operator->() const {
        return ptr_;
    }
    explicit operator bool() const { 
        return ptr_ != nullptr; 
    }
    bool operator!() const {
        return ptr_ == nullptr;
    }
    friend class ShrdPtr<T>;
};

template <typename T>
class ShrdPtr {
private:
    T* ptr_;
    size_t* ref_count_;

public:
    ShrdPtr(UnqPtr<T>&& owner) : ptr_(owner.ptr_), ref_count_(owner.ref_count_) {
        owner.ptr_ = nullptr;
        owner.ref_count_ = nullptr;
    }
    ShrdPtr(const ShrdPtr& other) : ptr_(other.ptr_), ref_count_(other.ref_count_) {
        if (ref_count_) ++(*ref_count_);
    }
    template <typename U, typename = typename std::enable_if<std::is_base_of<T, U>::value>::type>
    ShrdPtr(const ShrdPtr<U>& other) : ptr_(static_cast<T*>(other.get())), ref_count_(other.get_ref_count()) {
        if (ref_count_) ++(*ref_count_);
    }
    ShrdPtr& operator=(const ShrdPtr& other) {
        if (this != &other) { release();
            ptr_ = other.ptr_; ref_count_ = other.ref_count_;
            if (ref_count_) ++(*ref_count_);
        }
        return *this;
    }

    ~ShrdPtr() {
        release();
    }
    void release() {
        if (ref_count_ && --(*ref_count_) == 0){
            if constexpr (std::is_array_v<T>) {
                delete[] ptr_;
            } else {
            delete ptr_;
            }
            delete ref_count_;
        }
    }
    T* get() const {
        return ptr_;
    }
    T& operator*() const {
        return *ptr_;
    }
    T* operator->() const {
        return ptr_;
    }
    size_t use_count() const {
        return ref_count_ ? *ref_count_ : 0;
    }
    size_t* get_ref_count() const {
        return ref_count_;
    }
    bool operator!() const {
        return ptr_ == nullptr;
    }
    explicit operator bool() const { 
        return ptr_ != nullptr; 
    }
};

struct Base {
    virtual void print()
    { 
    std::cout << "Base\n";
    }
    virtual ~Base() = default;
};
struct Derived : Base { 
    void print() override { 
        std::cout << "Derived\n";
    }
};