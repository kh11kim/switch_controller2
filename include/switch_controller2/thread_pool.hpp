#pragma once
#include "switch_controller2/core.hpp"

class ThreadPool
{
protected:
  size_t num_threads;
  bool stop_all;
  std::vector<std::thread> worker_threads_;
  std::mutex m_job_q;
  std::queue<std::function<void()>> jobs;
  std::condition_variable cv_job_q;

  ThreadPool()
    : num_threads(2)
    , stop_all(false)
  {
    worker_threads_.reserve(num_threads);
    for (size_t i=0; i<num_threads; i++){
      worker_threads_.emplace_back([this](){this->WorkerThread();});
    }
  }
  
  ~ThreadPool(){
    stop_all = true;
    cv_job_q.notify_all();
    for (auto& t : worker_threads_){
      t.join();
    }
  }

  void EnqueueJob(std::function<void()> job){
    if (stop_all) {
      throw std::runtime_error("Stop All Thread in this pool!");
    }
    {
      std::lock_guard<std::mutex> lock(m_job_q);
      jobs.push(std::move(job));
    }
    cv_job_q.notify_one();
  }

  void WorkerThread(){
    while (true){
      std::unique_lock<std::mutex> lock(m_job_q);
      cv_job_q.wait(lock, [this](){ return !this->jobs.empty() || stop_all; });
      if (stop_all && this->jobs.empty()){
        return;
      }
      // pop first job
      std::function<void()> job = std::move(jobs.front());
      jobs.pop();
      lock.unlock();

      // do job
      job();
    }
  }
};