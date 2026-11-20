#include "helios_obstacle_detection/thread_pool.hpp"

#include <algorithm>

namespace helios_obstacle_detection
{

ThreadPool::ThreadPool(std::size_t threads)
{
  if (threads == 0) {
    const std::size_t hw = std::max<std::size_t>(std::thread::hardware_concurrency(), 2);
    threads = std::min<std::size_t>(hw - 1, 16);
  }
  workers_.reserve(threads);
  for (std::size_t i = 0; i < threads; ++i) {
    workers_.emplace_back([this] {workerLoop();});
  }
}

ThreadPool::~ThreadPool()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  for (auto & t : workers_) {
    if (t.joinable()) {
      t.join();
    }
  }
}

void ThreadPool::enqueue(std::function<void()> job)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(job));
  }
  cv_.notify_one();
}

void ThreadPool::workerLoop()
{
  while (true) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] {return stop_ || !queue_.empty();});
      if (stop_ && queue_.empty()) {
        return;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
    }
    job();  // packaged_task / parallelFor helpers never throw out of here
  }
}

namespace
{
struct ParallelState
{
  std::function<void(std::size_t, std::size_t)> fn;
  std::size_t n{0};
  std::size_t chunk{1};
  std::size_t chunks{0};
  std::atomic<std::size_t> next{0};
  std::atomic<std::size_t> done{0};
  std::mutex mutex;
  std::condition_variable cv;
  std::exception_ptr error;

  void run()
  {
    while (true) {
      const std::size_t c = next.fetch_add(1, std::memory_order_relaxed);
      if (c >= chunks) {
        return;
      }
      const std::size_t b = c * chunk;
      const std::size_t e = std::min(n, b + chunk);
      try {
        fn(b, e);
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!error) {
          error = std::current_exception();
        }
      }
      if (done.fetch_add(1, std::memory_order_acq_rel) + 1 == chunks) {
        std::lock_guard<std::mutex> lock(mutex);
        cv.notify_all();
      }
    }
  }
};
}  // namespace

void ThreadPool::parallelFor(
  std::size_t n, const std::function<void(std::size_t, std::size_t)> & fn, std::size_t min_chunk)
{
  if (n == 0) {
    return;
  }
  min_chunk = std::max<std::size_t>(min_chunk, 1);
  const std::size_t participants = workers_.size() + 1;
  // ~4 chunks per participant for load balancing, but never smaller than min_chunk.
  std::size_t chunk = std::max(min_chunk, (n + 4 * participants - 1) / (4 * participants));
  const std::size_t chunks = (n + chunk - 1) / chunk;
  if (chunks == 1 || workers_.empty()) {
    fn(0, n);
    return;
  }
  auto st = std::make_shared<ParallelState>();
  st->fn = fn;
  st->n = n;
  st->chunk = chunk;
  st->chunks = chunks;
  const std::size_t helpers = std::min(workers_.size(), chunks - 1);
  for (std::size_t i = 0; i < helpers; ++i) {
    enqueue([st] {st->run();});
  }
  st->run();
  {
    std::unique_lock<std::mutex> lock(st->mutex);
    st->cv.wait(lock, [&] {return st->done.load(std::memory_order_acquire) == st->chunks;});
  }
  if (st->error) {
    std::rethrow_exception(st->error);
  }
}

}  // namespace helios_obstacle_detection
