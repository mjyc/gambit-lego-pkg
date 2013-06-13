#ifndef THREAD_BASE_H
#define THREAD_BASE_H

#include <boost/thread.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/thread/locks.hpp>

class ThreadBase {
private:
    boost::shared_ptr<boost::thread> m_thread_ptr;

public:
    ThreadBase() : m_thread_ptr() { }
    virtual ~ThreadBase() { }

    virtual void run() = 0;

    void start() {
        if (m_thread_ptr == NULL)
            m_thread_ptr.reset(
                        new boost::thread(
                            boost::lambda::bind(&ThreadBase::run, this)));
        else
            throw std::runtime_error("multiple start");
    }

    void join() {
        if (m_thread_ptr)
            m_thread_ptr->join();
    }

    void interrupt() {
        if (m_thread_ptr)
            m_thread_ptr->interrupt();
    }
};

#endif // THREAD_BASE_H