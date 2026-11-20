// Small fixed-size thread pool with a deadlock-free, work-sharing parallelFor.
//
// parallelFor() splits [0, n) into chunks that are pulled from a shared atomic counter by the
// calling thread AND by helper tasks queued on the pool. The caller only waits until every chunk
// has *completed*; helper tasks that start late find no work and return. Therefore parallelFor()
// never waits for a worker to become free and cannot deadlock, even when all workers are busy
// with long-running tasks (e.g. ego-motion ICP submitted with submit()).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace helios_obstacle_detection
{

class ThreadPool
{
public:
  /// `threads == 0` -> hardware_concurrency() - 1 (at least 1), capped at 16.
  explicit ThreadPool(std::size_t threads = 0);
  ~ThreadPool();
  ThreadPool(const ThreadPool &) = delete;
  ThreadPool & operator=(const ThreadPool &) = delete;

  std::size_t size() const noexcept {return workers_.size();}

  template<class F>
  auto submit(F && f) -> std::future<std::invoke_result_t<std::decay_t<F>>>
  {
    using R = std::invoke_result_t<std::decay_t<F>>;
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
    std::future<R> fut = task->get_future();
    enqueue([task] {(*task)();});
    return fut;
  }

  /// Runs fn(begin, end) over [0, n) in chunks of at least `min_chunk` items. Blocks until all
  /// chunks are done; rethrows the first exception thrown by fn. Safe to call concurrently.
  void parallelFor(
    std::size_t n, const std::function<void(std::size_t, std::size_t)> & fn,
    std::size_t min_chunk = 1);

private:
  void enqueue(std::function<void()> job);
  void workerLoop();

  std::vector<std::thread> workers_;
  std::deque<std::function<void()>> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_{false};
};

}  // namespace helios_obstacle_detection
