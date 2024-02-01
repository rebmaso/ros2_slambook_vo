#pragma once
#ifndef THREADAFE_QUEUE_H
#define THREADAFE_QUEUE_H

#include <queue>

namespace myslam {

template<typename Data>
class threadsafe_queue
{
private:

    std::queue<Data> the_queue;
    std::mutex the_mutex;
    std::condition_variable the_condition_variable;
    long unsigned int max_size;

public:

    threadsafe_queue(const long unsigned int & max_size_) {
        if(max_size_ <= 0) throw std::invalid_argument( "received <= 0 value");
        max_size = max_size_;
    }

    void push_and_drop(Data const& data)
    {
        std::unique_lock<std::mutex> lock(the_mutex);

        // push
        the_queue.push(data);

        // if queue full, drop oldest el
        if(the_queue.size() > max_size) {
            the_queue.pop();
        }

        lock.unlock();

        the_condition_variable.notify_one();
    }

    bool empty() const
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        return the_queue.empty();
    }

    unsigned int size() 
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        return the_queue.size();
    }

    bool try_pop(Data& popped_value)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }
        
        popped_value=the_queue.front();
        the_queue.pop();
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }
        
        popped_value=the_queue.front();
        the_queue.pop();
    }

};

} // namespace myslam

#endif  // THREADAFE_QUEUE_H