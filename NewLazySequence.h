#pragma once
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include "Sequence.h"
#include "ArraySequence.h"
#include "SmrtPTRs.h"

template<typename T>
class LazySequence : public Sequence<T> {
private:



    struct Cardinal {
        size_t value;
        bool is_infinite;

        Cardinal(size_t v) : value(v), is_infinite(false) {}
        Cardinal() : value(0), is_infinite(true) {}
    };    




    class Generator {
    private:
        std::function<T(const std::deque<T>&)> rule_;
        std::deque<T> window_;
        size_t window_size_;
    public:
        Generator() : rule_(nullptr), window_size_(0) {}
        Generator(std::function<T(const std::deque<T>&)> rule, size_t window_size)
            : rule_(rule), window_size_(window_size) {}
        Generator(const Generator& other)
            : rule_(other.rule_), window_(other.window_), window_size_(other.window_size_) {}

        void SetWindow(const std::deque<T>& w) { window_ = w; }
        void SetRule(std::function<T(const std::deque<T>&)> rule) { rule_ = rule; }
        bool HasRule() const { return (bool)rule_; }
        size_t WindowSize() const { return window_size_; }

        T Next() {
            if (!rule_) throw std::runtime_error("No generator rule");
            if (window_.size() < window_size_)
                throw std::runtime_error("Not enough elements in window");
            T next = rule_(window_);
            window_.push_back(next);
            if (window_.size() > window_size_) window_.pop_front();
            return next;
        }
    };




    
    ArraySequence<T> data_;
    UnqPtr<Generator> generator_;
    mutable size_t materialized_count_ = 0;

    
    ArraySequence<std::function<T(T)>> maps_;
    ArraySequence<std::function<bool(T)>> wheres_;
    
    Cardinal cardinal_;

    bool ApplyWheres(const T& value) const {
        for (int i = 0; i < wheres_.GetLength(); ++i)
            if (!wheres_.Get(i)(value)) return false;
        return true;
    }

    T ApplyMaps(T value) const {
        for (int i = 0; i < maps_.GetLength(); ++i)
            value = maps_.Get(i)(value);
        return value;
    }

    void EnsureGeneratedUntil(size_t index) const {
        if (!generator_ || !generator_->HasRule()) return;
        auto* gen = generator_.get();
        while ((size_t)data_.GetLength() <= index) {
            T next = gen->Next();
            next = ApplyMaps(next);
            if (ApplyWheres(next))
                const_cast<ArraySequence<T>&>(data_).Append(next);
        }
    }

public:
    LazySequence() 
        : data_(), maps_(), wheres_(), generator_(nullptr), cardinal_(0) {}

    LazySequence(T* items, int count) 
        : data_(items, count), maps_(), wheres_(), generator_(nullptr), cardinal_(count) {}

    LazySequence(Sequence<T>* seq)
        : maps_(), wheres_(), generator_(nullptr) 
    {
        if (!seq) throw std::invalid_argument("Base sequence is null");
        for (int i = 0; i < seq->GetLength(); ++i)
            data_.Append(seq->Get(i));
        cardinal_ = Cardinal(data_.GetLength());
    }

    LazySequence(T(*rule)(Sequence<T>*), Sequence<T>* base): maps_(), wheres_() {
        if (!base) throw std::invalid_argument("Base sequence is null");
        for (int i = 0; i < base->GetLength(); ++i)
            data_.Append(base->Get(i));

        generator_ = UnqPtr<Generator>(new Generator(
            [rule](const std::deque<T>& w) {
                ArraySequence<T> tmp;
                for (auto& e : w) tmp.Append(e);
                return rule(&tmp);
            },
            std::min<size_t>(base->GetLength(), 2)
        ));

        std::deque<T> window;
        int from = std::max(0, base->GetLength() - (int)generator_->WindowSize());
        for (int i = from; i < base->GetLength(); ++i)
            window.push_back(base->Get(i));
        generator_->SetWindow(window);

        cardinal_ = Cardinal();
    }

    LazySequence(std::function<T(Sequence<T>*)> rule, Sequence<T>* base)
        : maps_(), wheres_()
    {
        if (!base) throw std::invalid_argument("Base sequence is null");
        for (int i = 0; i < base->GetLength(); ++i)
            data_.Append(base->Get(i));

        generator_ = UnqPtr<Generator>(new Generator(
            [rule](const std::deque<T>& w) {
                ArraySequence<T> tmp;
                for (auto& e : w) tmp.Append(e);
                return rule(&tmp);
            },
            std::min<size_t>(base->GetLength(), 2)
        ));

        std::deque<T> window;
        int from = std::max(0, base->GetLength() - (int)generator_->WindowSize());
        for (int i = from; i < base->GetLength(); ++i)
            window.push_back(base->Get(i));
        generator_->SetWindow(window);

        cardinal_ = Cardinal();
    }

    LazySequence(std::function<T(const std::deque<T>&)> rule,
                 Sequence<T>* start,
                 size_t window_size)
        : maps_(), wheres_()
    {
        if (!start) throw std::invalid_argument("Base sequence is null");

        for (int i = 0; i < start->GetLength(); ++i)
            data_.Append(start->Get(i));

        generator_ = UnqPtr<Generator>( new Generator(rule, window_size));

        std::deque<T> window;
        int start_len = start->GetLength();
        int from = std::max(0, start_len - (int)window_size);
        for (int i = from; i < start_len; ++i)
            window.push_back(start->Get(i));
        generator_->SetWindow(window);

        cardinal_ = Cardinal();
    }

    LazySequence(const LazySequence<T>& other)
        : data_(other.data_),
          maps_(other.maps_),
          wheres_(other.wheres_),
          materialized_count_(other.materialized_count_),
          cardinal_(other.cardinal_)
    {
        if (other.generator_)
            generator_ = UnqPtr<Generator>( new Generator(*other.generator_));
    }
    T GetLast() const override {
        if (cardinal_.is_infinite) {
            throw std::out_of_range("Cannot get last element of an infinite sequence");
        }
        if (data_.GetLength() == 0)
            throw std::out_of_range("Empty sequence");
        return data_[data_.GetLength() - 1];
    }

    T GetFirst() const override {
        return data_.Get(0);
    }

    int GetLength() const override {
        if (cardinal_.is_infinite)
            return -1; 
        return static_cast<int>(data_.GetLength());
    }

    T Get(int index) const override {
        if (index < 0) throw std::out_of_range("Negative index");
        EnsureGeneratedUntil(index);
        if (index >= static_cast<int>(data_.GetLength()) && !cardinal_.is_infinite)
            throw std::out_of_range("Index too large");
        return data_.Get(index);
    }

    Sequence<T>* Append(T item) override {
        if (cardinal_.is_infinite)
            throw std::runtime_error("Cannot append to infinite sequence");
        LazySequence<T>* copy = new LazySequence<T>(*this);
        copy->data_.Append(item);
        copy->cardinal_.value++;
        return copy;
    }

    Sequence<T>* Prepend(T item) override {
        LazySequence<T>* copy = new LazySequence<T>(*this);
        copy->data_.Prepend(item);
        copy->cardinal_.value++;
        return copy;
    }

    Sequence<T>* InsertAt(T item, int index) override {
        LazySequence<T>* copy = new LazySequence<T>(*this);
        copy->data_.InsertAt(item, index);
        copy->cardinal_.value++;
        return copy;
    }

    Sequence<T>* Concat(Sequence<T>* seq) const override {
        LazySequence<T>* other = dynamic_cast<LazySequence<T>*>(seq);
        if (!other) throw std::runtime_error("Concat only supports LazySequence");

        if (!this->cardinal_.is_infinite && !other->cardinal_.is_infinite) {
            LazySequence<T>* copy = new LazySequence<T>(*this);
            for (int i = 0; i < other->GetLength(); ++i)
                copy->data_.Append(other->Get(i));
            copy->cardinal_.value = this->cardinal_.value + other->cardinal_.value;
            return copy;
        }

        if (!this->cardinal_.is_infinite && other->cardinal_.is_infinite) {
            LazySequence<T>* copy = new LazySequence<T>(*other);
            for (int i = this->GetLength() - 1; i >= 0; --i)
                copy->data_.Prepend(this->Get(i));
            copy->cardinal_ = other->cardinal_;
            return copy;
        }

        if (this->cardinal_.is_infinite)
            throw std::runtime_error("Cannot concat to an infinite sequence");

        return nullptr;
    }

    Sequence<T>* GetSubsequence(int start, int end) const override {
        Sequence<T>* sub = data_.GetSubsequence(start, end);
        LazySequence<T>* result = new LazySequence<T>();
        result->data_ = *dynamic_cast<ArraySequence<T>*>(sub);
        delete sub;
        return result;
    }

    Sequence<T>* Set(T value, int index) override {
        LazySequence<T>* copy = new LazySequence<T>(*this);
        copy->data_.Set(value, index);
        return copy;
    }

    Sequence<T>* Map(std::function<T(T)> f) const override {
        LazySequence<T>* copy = new LazySequence<T>(*this);
        ArraySequence<T>* mapped = dynamic_cast<ArraySequence<T>*>(data_.Map(f));
        copy->data_ = *mapped;
        delete mapped;
        copy->maps_.Append(f);
        return copy;
    }

    Sequence<T>* Where(std::function<bool(T)> f) const override {
        LazySequence<T>* copy = new LazySequence<T>(*this);
        ArraySequence<T>* filtered = dynamic_cast<ArraySequence<T>*>(data_.Where(f));
        copy->data_ = *filtered;
        delete filtered;
        copy->wheres_.Append(f);
        return copy;
    }

    T Reduce(std::function<T(T, T)> f, T start_value) const override {
        return data_.Reduce(f, start_value);
    }

    void To_String() const {
        data_.To_String();
    }
};
