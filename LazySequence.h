#pragma once
#include <iostream>
#include <map>
#include <deque> // Для std::deque
#include <stdexcept> // Для std::out_of_range
#include <functional> // Для std::function
#include "Sequence.h"
#include "ArraySequence.h"

// Forward declaration для разрешения циклической зависимости
template <typename T>
class LazySequence;

template <typename T>
class Generator {
private:
    LazySequence<T>* owner_; 
    std::function<T(Sequence<T>*)> rule_;
    std::deque<T> window_;
    size_t initial_size_ = 0; // Изначальная длина очереди

public:
    Generator(LazySequence<T>* owner, std::function<T(Sequence<T>*)> rule, const Sequence<T>* initial = nullptr)
        : owner_(owner), rule_(rule) {
        if (initial) {
            for (int i = 0; i < initial->GetLength(); ++i) {
                // Предполагаем, что Get в Sequence.h должен быть const
                window_.push_back(const_cast<Sequence<T>*>(initial)->Get(i)); // Временное решение
            }
            initial_size_ = initial->GetLength();
        }
    }

    T GetNext() {
        if (!HasNext()) {
            throw std::out_of_range("End of sequence");
        }
        T next = rule_(owner_);
        // Сдвиг очереди
        if (!window_.empty() && window_.size() >= initial_size_) {
            window_.pop_front();
        }
        window_.push_back(next);
        // Мемоизация через owner
        owner_->materialized_.Append(next);
        ++owner_->materialized_count_;

        return next;
    }

    bool HasNext() const {
        return !window_.empty() || (owner_->GetLengthCardinal().is_infinite);
    }

    std::function<T(Sequence<T>*)> GetRule() const { return rule_; } // Публичный метод для доступа к rule_
};

struct Cardinal {
    size_t value;
    bool is_infinite;
    Cardinal(size_t v) : value(v), is_infinite(false) {}
    Cardinal() : value(0), is_infinite(true) {}
};

template <typename T>
class LazySequence : public Sequence<T> {
public:
    ArraySequence<T> materialized_;           // Материализованная часть последовательности
    Generator<T> generator_;                  // Генератор для ленивого вычисления
    LazySequence<T>* base_= nullptr;                   // Для concat/map и т.д.
    Cardinal length_;                         // Лениво вычисленная длина
    size_t materialized_count_ = 0;           // Количество материализованных элементов
    size_t length_shift_=0;
    std::map<int, T> somepended_;               // Словарь для хранения вставленных элементов
    int index_shift_ = 0;                     // Сдвиг индексов для коррекции

    friend class Generator<T>; // Дружественный класс для доступа к приватным членам

    // Приватный метод материализации
    void Materialize(size_t up_to) {
        size_t adjusted_up_to = up_to - index_shift_;
        while (materialized_.GetLength() <= adjusted_up_to && generator_.HasNext()) {

            T next = generator_.GetNext();
        }
        if (materialized_.GetLength() <= adjusted_up_to) {
            throw std::out_of_range("Index out of range");
        }

    }
    void GetInfo(){
        std::cout<<"\nmaterialized_: ";
        materialized_.To_String();
        std::cout<<"\nlength_.is_infinite: "<<length_.is_infinite<<"\nlength_.value: "<<length_.value<<
        "\nmaterialized_count_: "<<materialized_count_<<"\nindex_shift_: "<<index_shift_<<"\n";
    }
public:
    LazySequence(): generator_(this, [](Sequence<T>* seq) -> T { throw std::out_of_range("No rule"); }), 
          materialized_() {
        length_ = Cardinal(); // Бесконечная по умолчанию
    }
    LazySequence(T* items, int count): generator_(this, [](Sequence<T>* seq) -> T { throw std::out_of_range("No rule"); }),   materialized_(items, count) {
        materialized_count_ = count;
        length_ = Cardinal(count);
    }
    LazySequence(Sequence<T>* seq): generator_(this, [](Sequence<T>* seq) -> T { throw std::out_of_range("No rule"); }),   materialized_(seq->GetLength()) {
        materialized_(seq);
        for (int i = 0; i < seq->GetLength(); ++i) {
            materialized_.Append(seq->Get(i)); // Предполагаем const Get
        }
        materialized_count_ = seq->GetLength();
        length_ = Cardinal(seq->GetLength());
    }
    LazySequence(std::function<T(Sequence<T>*)> rule, Sequence<T>* initial = nullptr): generator_(this, rule), materialized_(initial ? initial->GetLength() : 0) {
        if (initial) {
            for (int i = 0; i < initial->GetLength(); ++i) {
                materialized_.Append(initial->Get(i));        // Предполагаем const Get
            }
            materialized_count_ = initial->GetLength();
            length_ = Cardinal(initial->GetLength());
        } else {
            length_ = Cardinal();         // Бесконечная, если initial = nullptr
        }
    }
    LazySequence(const LazySequence<T>& other) 
        : materialized_(other.materialized_), generator_(this, other.generator_.GetRule(), &other.materialized_),
          base_(other.base_ ? new LazySequence<T>(*other.base_) : nullptr), length_(other.length_), 
          materialized_count_(other.materialized_count_), somepended_(other.somepended_), index_shift_(other.index_shift_) {}

    // Константные методы для соответствия Sequence<T>
    T GetFirst() const override {
        if (materialized_.GetLength() == 0 && somepended_.empty()) {
            // Преобразуем const объект к неконстантному для материализации
            LazySequence<T>* nonConstThis = const_cast<LazySequence<T>*>(this);
            nonConstThis->Materialize(0);
        }
        if (!somepended_.empty() && somepended_.find(-index_shift_) != somepended_.end()) {
            return somepended_.at(-index_shift_);
        }
        return materialized_.Get(0);
        }

    T GetLast() const override {
        if (length_.is_infinite) {
            throw std::out_of_range("Infinite sequence has no last");
        }
        if (materialized_.GetLength() < length_.value) {
            // Преобразуем const объект к неконстантному для материализации
            LazySequence<T>* nonConstThis = const_cast<LazySequence<T>*>(this);
            return nonConstThis->GetLastNonConst();
        }
        return materialized_.GetLast();
    }

    T Get(int index) const override {
        if (index < 0) {
            throw std::out_of_range("Index out of range");
        }
        int adjusted_index = index - index_shift_;
        
        if (somepended_.find(adjusted_index) != somepended_.end()) {
            return somepended_.at(adjusted_index);
        }
        if (materialized_.GetLength() <= static_cast<size_t>(adjusted_index)) {
            // Преобразуем const объект к неконстантному для материализации
            LazySequence<T>* nonConstThis = const_cast<LazySequence<T>*>(this);
            nonConstThis->Materialize(index);
        }
        
        return materialized_.Get(adjusted_index);
    }
    int GetLength() const override {
        if(length_.is_infinite){
            return -1;
        }
        else if(!somepended_.empty()&&(static_cast<int>(somepended_.rbegin()->first)>0)){
            return static_cast<int>(somepended_.rbegin()->first)+1;
        }
        else return static_cast<int>(materialized_count_) + index_shift_;
    }
    int GetValidLength(){
        return materialized_count_ + index_shift_;
    }
    Cardinal GetLengthCardinal() const { return length_; }

    size_t GetMaterializedCount() const { return materialized_count_; }

    // Неконстантные обертки для материализации
    T GetNonConst(int index) {
        if (index < 0) throw std::out_of_range("Index out of range");
        int adjusted_index = index - index_shift_;
        if (somepended_.find(adjusted_index) != somepended_.end()) {
            return somepended_.at(adjusted_index);
        }
        Materialize(static_cast<size_t>(adjusted_index));
        return materialized_.Get(adjusted_index);
    }

    T GetFirstNonConst() {
        if (materialized_.GetLength() == 0 && somepended_.empty()) Materialize(0);
        if (!somepended_.empty() && somepended_.find(-index_shift_) != somepended_.end()) {
            return somepended_.at(-index_shift_);
        }
        return materialized_.Get(0);
    }

    T GetLastNonConst() {
        if (!length_.is_infinite) {
            Materialize(length_.value - 1 - index_shift_);
            return materialized_.GetLast();
        }
        throw std::out_of_range("Infinite sequence has no last");
    }

    // Операции
    Sequence<T>* Append(T item) {
        LazySequence<T>* new_seq = new LazySequence<T>(*this);
        if(this->length_.is_infinite){
            throw std::logic_error("Cannot append in infitite sequence");
        }
        // Вместо Materialize, просто добавляем в somepended_
        int append_index = new_seq->materialized_.GetLength() + new_seq->index_shift_;
        new_seq->somepended_[append_index] = item;
        
        // Обновляем длину
        if (!new_seq->length_.is_infinite) {
            new_seq->length_ = Cardinal(new_seq->length_.value + 1);
        }
        
        return new_seq;
    }

    Sequence<T>* Prepend(T item) {
        LazySequence<T>* new_seq = new LazySequence<T>(*this);
        
        // Используем отрицательные индексы для prepend
        new_seq->somepended_[-1 - new_seq->index_shift_] = item;
        new_seq->index_shift_++;
        // Обновляем длину
        if (!new_seq->length_.is_infinite) {
            new_seq->length_ = Cardinal(new_seq->length_.value + 1);
        }
        
        return new_seq;
    }

    Sequence<T>* InsertAt(T item, int index) {
        LazySequence<T>* new_seq = new LazySequence<T>(*this);
        
        // Проверяем валидность индекса
        if (!new_seq->length_.is_infinite && index > new_seq->length_.value + new_seq->index_shift_) {
            throw std::out_of_range("Index out of range");
        }
        
        int adjusted_index = index - new_seq->index_shift_;
        new_seq->somepended_[adjusted_index] = item;
        
        // Обновляем длину
        if (!new_seq->length_.is_infinite) {
            new_seq->length_ = Cardinal(new_seq->length_.value + 1);
        }
        
        return new_seq;
    }

    Sequence<T>* Concat(Sequence<T>* other) const override {
        if (this->GetLength() == -1) { // Бесконечная последовательность
            return new LazySequence<T>(*this); // Возвращаем копию
        } else { // Конечная последовательность
            LazySequence<T>* new_seq = new LazySequence<T>(new int[0],0);
            // Копируем данные, используя только материализованные значения
            for (int i = 0; i < this->GetLength() && i < static_cast<int>(materialized_.GetLength()); ++i) {
                new_seq->Append(this->Get(i)); // Используем const Get
            }
            auto concat_rule = [this, other](Sequence<T>* seq) -> T {
                int len = this->GetLength();
                if (len == -1) throw std::out_of_range("Should not reach here for infinite");
                int current_index = seq->GetLength() - this->index_shift_;
                if (current_index < len && current_index < static_cast<int>(this->materialized_.GetLength())) {
                    return this->Get(current_index);
                } else {
                    int other_index = current_index - len;
                    if (other_index < other->GetLength()) {
                        return other->Get(other_index);
                    }
                    throw std::out_of_range("Index out of range");
                }
            };
            new_seq->generator_ = Generator<T>(new_seq, concat_rule, this);
            new_seq->length_ = Cardinal(this->length_.value + (other->GetLength() == -1 ? 0 : other->GetLength()));
            return new_seq;
        }
    }

    Sequence<T>* Set(T value, int index) override {
        throw std::out_of_range("Set not supported in LazySequence");
        return this;
    }

    Sequence<T>* GetSubsequence(int start, int end) const override {
        throw std::out_of_range("GetSubsequence not supported in LazySequence");
        return nullptr;
    }

    Sequence<T>* Where(std::function<bool(T)> predicate) const override {
        LazySequence<T>* new_seq = new LazySequence<T>();
        // Фильтруем только материализованные данные
        for (int i = 0; i < static_cast<int>(materialized_.GetLength()); ++i) {
            if (predicate(materialized_.Get(i))) {
                new_seq->Append(materialized_.Get(i));
            }
        }
        // Создаем правило для ленивой фильтрации, но без немедленной материализации
        auto where_rule = [this, predicate](Sequence<T>* seq) -> T {
            int index = seq->GetLength() - this->index_shift_;
            // Проверяем только материализованные данные в константном контексте
            if (index < static_cast<int>(this->materialized_.GetLength())) {
                T value = this->Get(index);
                if (predicate(value)) {
                    return value;
                }
                throw std::out_of_range("No matching element at this index");
            }
            throw std::logic_error("Cannot materialize in const context");
        };
        new_seq->generator_ = Generator<T>(new_seq, where_rule, this);
        // Длина остается такой же, так как точное количество неизвестно
        new_seq->length_ = this->length_;
        return new_seq;
    }

    Sequence<T>* Map(std::function<T(T)> func) const override {
        LazySequence<T>* new_seq = new LazySequence<T>();
        auto map_rule = [this, func](Sequence<T>* seq) -> T {
            int index = seq->GetLength() - this->index_shift_;
            if (index < this->GetLength() && index < static_cast<int>(this->materialized_.GetLength())) {
                return func(this->Get(index));
            }
            throw std::out_of_range("Index out of range");
        };
        new_seq->generator_ = Generator<T>(new_seq, map_rule, this);
        new_seq->length_ = this->length_;
        return new_seq;
    }

    T Reduce(std::function<T(T, T)> func, T start) const override {
        if(length_.is_infinite){
            throw std::logic_error("Cannot reduce infitite sequence");
        }
        T acc = start;
        size_t i = 0;
        while (true) {
            try {
                if (i >= materialized_.GetLength()) {
                    throw std::out_of_range("Cannot materialize in const context");
                }
                acc = func(acc, Get(i++));
            } catch (const std::out_of_range&) {
                break;
            }
        }
        return acc;
    }
};