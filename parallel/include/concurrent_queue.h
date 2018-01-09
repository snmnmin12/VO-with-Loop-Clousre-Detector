#pragma once
#include <mutex>
#include <condition_variable>
#include <queue>
template<typename T>
class Queue {
public:
	T pop() {
		std::unique_lock<mutex> lck(mutex_);
		while(queue_.empty()) {
			cond_.wait(lck);
		}
		auto item = queue_.front();
		queue_.pop();
		return item;
	}

	void pop(T& item) {
	std::unique_lock<mutex> lck(mutex_);
		while(queue_.empty()) {
			cond_.wait(lck);
		}
		item = queue_.front();
		queue_.pop();
	}

	void push(T& item) {
		std::unique_lock<mutex> lck(mutex_);
		queue_.push(item);
		lck.unlock();
		cond_.notify_one();
	}

private:
	std::queue<T> queue_;
	std::mutex mutex_;
	std::condition_variable cond_;
};