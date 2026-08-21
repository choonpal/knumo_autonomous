#ifndef UBLOX_GPS_ASYNC_WORKER_HPP
#define UBLOX_GPS_ASYNC_WORKER_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <asio/buffer.hpp>
#include <asio/error.hpp>
#include <asio/error_code.hpp>
#include <asio/io_service.hpp>
#include <asio/placeholders.hpp>
#include <asio/write.hpp>
#include <asio/ip/udp.hpp>

#include <rclcpp/rclcpp.hpp>

#include "worker.hpp"

namespace ublox_gps {

/**
 * @brief Handles Asynchronous I/O reading and writing.
 */
template <typename StreamT>
class AsyncWorker final : public Worker {
 public:
  /**
   * @brief Construct an Asynchronous I/O worker.
   * @param stream the stream for th I/O service
   * @param io_service the I/O service
   * @param buffer_size the size of the input and output buffers
   */
  explicit AsyncWorker(std::shared_ptr<StreamT> stream,
                       std::shared_ptr<asio::io_service> io_service,
                       std::size_t buffer_size,
                       int debug,
                       const rclcpp::Logger& logger);
  ~AsyncWorker() override;

  AsyncWorker(AsyncWorker &&c) = delete;
  AsyncWorker &operator=(AsyncWorker &&c) = delete;
  AsyncWorker(const AsyncWorker &c) = delete;
  AsyncWorker &operator=(const AsyncWorker &c) = delete;

  /**
   * @brief Set the callback function which handles input messages.
   * @param callback the read callback which handles received messages
   */
  void setCallback(const WorkerCallback& callback) override;

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  void setRawDataCallback(const WorkerRawCallback& callback) override;

  /**
   * @brief Send the data bytes via the I/O stream.
   * @param data the buffer of data bytes to send
   * @param size the size of the buffer
   */
  bool send(const unsigned char* data, const unsigned int size) override;
  /**
   * @brief Wait for incoming messages.
   * @param timeout the maximum time to wait
   */
  void wait(const std::chrono::milliseconds& timeout) override;

  bool isOpen() const override {
    return !stopping_.load(std::memory_order_acquire);
  }

 private:
  /**
   * @brief Read the input stream.
   */
  void doRead();

  /**
   * @brief Process messages read from the input stream.
   * @param error_code an error code for read failures
   * @param bytes_received the number of bytes received
   */
  void readEnd(const asio::error_code& error, std::size_t bytes_transferred);

  /**
   * @brief Send all the data in the output buffer.
   */
  void doWrite();

  /**
   * @brief Complete an asynchronous output operation.
   */
  void writeEnd(const asio::error_code& error, std::size_t bytes_transferred);

  /**
   * @brief Close the I/O stream.
   */
  void doClose();

  std::shared_ptr<StreamT> stream_; //!< The I/O stream
  std::shared_ptr<asio::io_service> io_service_; //!< The I/O service

  std::mutex read_mutex_; //!< Lock for the input buffer
  std::condition_variable read_condition_;
  std::vector<unsigned char> in_; //!< The input buffer
  std::size_t in_buffer_size_; //!< number of bytes currently in the input
                               //!< buffer

  std::mutex write_mutex_; //!< Lock for the output buffer
  std::condition_variable write_condition_;
  std::vector<unsigned char> out_; //!< The output buffer
  bool write_in_progress_{false};

  std::shared_ptr<std::thread> background_thread_; //!< thread for the I/O
                                                       //!< service
  std::mutex callback_mutex_;
  WorkerCallback read_callback_; //!< Callback function to handle received messages
  WorkerRawCallback raw_callback_; //!< Callback function to handle raw data

  std::atomic_bool stopping_; //!< Whether or not the I/O service is closed

  int debug_; //!< Used to determine which debug messages to display

  rclcpp::Logger logger_;
};

template <typename StreamT>
AsyncWorker<StreamT>::AsyncWorker(std::shared_ptr<StreamT> stream,
        std::shared_ptr<asio::io_service> io_service,
        std::size_t buffer_size,
        int debug,
        const rclcpp::Logger& logger)
    : stream_(stream), io_service_(io_service), in_buffer_size_(0), stopping_(false), debug_(debug), logger_(logger) {
  in_.resize(buffer_size);

  out_.reserve(buffer_size);

  io_service_->post(std::bind(&AsyncWorker<StreamT>::doRead, this));
  background_thread_ = std::make_shared<std::thread>([this] {
    try {
      io_service_->run();
    } catch (const std::exception& error) {
      stopping_.store(true, std::memory_order_release);
      RCLCPP_ERROR(
        logger_, "U-Blox I/O worker stopped after an exception: %s",
        error.what());
      asio::error_code close_error;
      stream_->close(close_error);
      read_condition_.notify_all();
      write_condition_.notify_all();
    } catch (...) {
      stopping_.store(true, std::memory_order_release);
      RCLCPP_ERROR(logger_, "U-Blox I/O worker stopped after an unknown exception");
      asio::error_code close_error;
      stream_->close(close_error);
      read_condition_.notify_all();
      write_condition_.notify_all();
    }
  });
}

template <typename StreamT>
AsyncWorker<StreamT>::~AsyncWorker() {
  stopping_.store(true, std::memory_order_release);
  io_service_->post(std::bind(&AsyncWorker<StreamT>::doClose, this));
  if (background_thread_ && background_thread_->joinable()) {
    background_thread_->join();
  }
  // run() may already have returned after an I/O failure, in which case a
  // newly posted close handler will not execute. Close once more after join.
  asio::error_code error;
  stream_->close(error);
}

template <typename StreamT>
void AsyncWorker<StreamT>::setCallback(const WorkerCallback& callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  read_callback_ = callback;
}

template <typename StreamT>
void AsyncWorker<StreamT>::setRawDataCallback(
    const WorkerRawCallback& callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  raw_callback_ = callback;
}

template <typename StreamT>
bool AsyncWorker<StreamT>::send(const unsigned char* data,
                                const unsigned int size) {
  if (stopping_.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (stopping_.load(std::memory_order_relaxed)) {
    return false;
  }
  if (data == nullptr || size == 0) {
    RCLCPP_ERROR(logger_, "Ublox AsyncWorker::send: Size of message to send is 0");
    return false;
  }

  if (out_.capacity() - out_.size() < size) {
    RCLCPP_ERROR(logger_, "Ublox AsyncWorker::send: Output buffer too full to send message");
    return false;
  }
  out_.insert(out_.end(), data, data + size);

  io_service_->post(std::bind(&AsyncWorker<StreamT>::doWrite, this));
  return true;
}

template <typename StreamT>
void AsyncWorker<StreamT>::doWrite() {
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (stopping_.load(std::memory_order_acquire) ||
      out_.empty() || write_in_progress_) {
    return;
  }
  write_in_progress_ = true;
  try {
    asio::async_write(
      *stream_, asio::buffer(out_.data(), out_.size()),
      std::bind(
        &AsyncWorker<StreamT>::writeEnd, this,
        std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception& error) {
    write_in_progress_ = false;
    out_.clear();
    stopping_.store(true, std::memory_order_release);
    RCLCPP_ERROR(
      logger_, "U-Blox failed to start asynchronous write: %s",
      error.what());
    asio::error_code close_error;
    stream_->close(close_error);
    write_condition_.notify_all();
  }
}

template <>
inline void AsyncWorker<asio::ip::udp::socket>::doWrite() {
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (stopping_.load(std::memory_order_acquire) ||
      out_.empty() || write_in_progress_) {
    return;
  }
  write_in_progress_ = true;
  try {
    stream_->async_send(
      asio::buffer(out_.data(), out_.size()),
      std::bind(
        &AsyncWorker<asio::ip::udp::socket>::writeEnd, this,
        std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception& error) {
    write_in_progress_ = false;
    out_.clear();
    stopping_.store(true, std::memory_order_release);
    RCLCPP_ERROR(
      logger_, "U-Blox failed to start asynchronous UDP write: %s",
      error.what());
    asio::error_code close_error;
    stream_->close(close_error);
    write_condition_.notify_all();
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::writeEnd(
    const asio::error_code& error,
    std::size_t bytes_transferred) {
  bool write_more = false;
  bool write_failed = static_cast<bool>(error);
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    write_in_progress_ = false;
    if (error) {
      if (error != asio::error::operation_aborted ||
          !stopping_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR(
          logger_, "U-Blox ASIO output write error: %s (%zu bytes)",
          error.message().c_str(), bytes_transferred);
      }
      out_.clear();
      stopping_.store(true, std::memory_order_release);
    } else if (bytes_transferred == 0) {
      RCLCPP_ERROR(logger_, "U-Blox ASIO output transferred zero bytes");
      out_.clear();
      stopping_.store(true, std::memory_order_release);
      write_failed = true;
    } else {
      const std::size_t consumed =
        std::min(bytes_transferred, out_.size());
      if (debug_ >= 2) {
        std::ostringstream oss;
        for (auto it = out_.begin(); it != out_.begin() + consumed; ++it) {
          oss << std::hex << static_cast<unsigned int>(*it) << " ";
        }
        RCLCPP_DEBUG(
          logger_, "U-Blox sent %zu bytes: \n%s",
          consumed, oss.str().c_str());
      }
      out_.erase(out_.begin(), out_.begin() + consumed);
      write_more =
        !out_.empty() && !stopping_.load(std::memory_order_acquire);
    }
    write_condition_.notify_all();
  }

  if (write_failed) {
    asio::error_code close_error;
    stream_->close(close_error);
    read_condition_.notify_all();
  } else if (write_more) {
    io_service_->post(std::bind(&AsyncWorker<StreamT>::doWrite, this));
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::doRead() {
  if (stopping_.load(std::memory_order_acquire)) {
    return;
  }
  std::lock_guard<std::mutex> lock(read_mutex_);
  if (stopping_.load(std::memory_order_relaxed)) {
    return;
  }
  if (in_.size() - in_buffer_size_ == 0) {
    // In some circumstances, it is possible that there is no room left in the
    // buffer.  This can happen, for instance, if one of the UBlox messages
    // has a value in the Length field that is much larger than this buffer
    // can accomodate.  We definitely don't want to ask for a 0-byte read (as
    // we will get into an endless loop of asking for, and then receiving,
    // 0 bytes), so we just throw away all of the data in the buffer.
    in_buffer_size_ = 0;
  }

  try {
    stream_->async_read_some(
        asio::buffer(in_.data() + in_buffer_size_,
                     in_.size() - in_buffer_size_),
        std::bind(&AsyncWorker<StreamT>::readEnd, this,
                  std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception& error) {
    stopping_.store(true, std::memory_order_release);
    RCLCPP_ERROR(
      logger_, "U-Blox failed to start asynchronous read: %s", error.what());
    asio::error_code close_error;
    stream_->close(close_error);
    read_condition_.notify_all();
  }
}
template <>
inline void AsyncWorker<asio::ip::udp::socket>::doRead() {
  if (stopping_.load(std::memory_order_acquire)) {
    return;
  }
  std::lock_guard<std::mutex> lock(read_mutex_);
  if (stopping_.load(std::memory_order_relaxed)) {
    return;
  }
  if (in_.size() - in_buffer_size_ == 0) {
    // In some circumstances, it is possible that there is no room left in the
    // buffer.  This can happen, for instance, if one of the UBlox messages
    // has a value in the Length field that is much larger than this buffer
    // can accomodate.  We definitely don't want to ask for a 0-byte read (as
    // we will get into an endless loop of asking for, and then receiving,
    // 0 bytes), so we just throw away all of the data in the buffer.
    in_buffer_size_ = 0;
  }

  try {
    stream_->async_receive(
        asio::buffer(in_.data() + in_buffer_size_,
                     in_.size() - in_buffer_size_),
        std::bind(&AsyncWorker<asio::ip::udp::socket>::readEnd, this,
                  std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception& error) {
    stopping_.store(true, std::memory_order_release);
    RCLCPP_ERROR(
      logger_, "U-Blox failed to start asynchronous UDP read: %s",
      error.what());
    asio::error_code close_error;
    stream_->close(close_error);
    read_condition_.notify_all();
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::readEnd(const asio::error_code& error,
                                   std::size_t bytes_transferred) {
  bool read_more = false;
  {
    std::lock_guard<std::mutex> lock(read_mutex_);
    if (error) {
      if (error != asio::error::operation_aborted ||
          !stopping_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR(
          logger_, "U-Blox ASIO input read error: %s (%zu bytes)",
          error.message().c_str(), bytes_transferred);
      }
      stopping_.store(true, std::memory_order_release);
    } else if (bytes_transferred > 0) {
      in_buffer_size_ += bytes_transferred;

      unsigned char* raw_data_start =
        in_.data() + (in_buffer_size_ - bytes_transferred);
      WorkerRawCallback raw_callback;
      WorkerCallback read_callback;
      {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        raw_callback = raw_callback_;
        read_callback = read_callback_;
      }
      if (raw_callback) {
        raw_callback(raw_data_start, bytes_transferred);
      }

      if (debug_ >= 4) {
        std::ostringstream oss;
        for (auto it = in_.begin() + in_buffer_size_ - bytes_transferred;
             it != in_.begin() + in_buffer_size_; ++it) {
          oss << std::hex << static_cast<unsigned int>(*it) << " ";
        }
        RCLCPP_DEBUG(
          logger_, "U-Blox received %zu bytes \n%s", bytes_transferred,
          oss.str().c_str());
      }

      if (read_callback) {
        const std::size_t consumed =
          std::min(read_callback(in_.data(), in_buffer_size_), in_buffer_size_);
        in_buffer_size_ -= consumed;
      }

      read_condition_.notify_all();
      read_more = !stopping_.load(std::memory_order_acquire);
    } else {
      RCLCPP_ERROR(logger_, "U-Blox ASIO transferred zero bytes");
      stopping_.store(true, std::memory_order_release);
    }
  }

  if (error || bytes_transferred == 0) {
    asio::error_code close_error;
    stream_->close(close_error);
    write_condition_.notify_all();
  } else if (read_more) {
    io_service_->post(std::bind(&AsyncWorker<StreamT>::doRead, this));
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::doClose() {
  stopping_.store(true, std::memory_order_release);
  asio::error_code error;
  stream_->close(error);
  if (error) {
    RCLCPP_ERROR(logger_, "Error while closing the AsyncWorker stream: %s",
                 error.message().c_str());
  }
  read_condition_.notify_all();
  write_condition_.notify_all();
}

template <typename StreamT>
void AsyncWorker<StreamT>::wait(
    const std::chrono::milliseconds& timeout) {
  std::unique_lock<std::mutex> lock(read_mutex_);
  read_condition_.wait_for(lock, timeout);
}

}  // namespace ublox_gps

#endif  // UBLOX_GPS_ASYNC_WORKER_HPP
