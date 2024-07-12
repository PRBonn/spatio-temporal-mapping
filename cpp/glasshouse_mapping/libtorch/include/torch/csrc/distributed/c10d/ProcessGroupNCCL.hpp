#pragma once

#if defined(__linux__)
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#ifdef USE_C10D_NCCL

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <torch/csrc/distributed/c10d/Backend.hpp>
#include <torch/csrc/distributed/c10d/NCCLUtils.hpp>
#include <torch/csrc/distributed/c10d/PrefixStore.hpp>
#include <torch/csrc/distributed/c10d/Store.hpp>
#include <torch/csrc/distributed/c10d/intra_node_comm.hpp>

#include <ATen/DynamicLibrary.h>
#include <ATen/cuda/CUDAContext.h>
#include <ATen/cuda/CUDAEvent.h>
#include <c10/core/Stream.h>
#include <c10/core/StreamGuard.h>
#include <c10/cuda/CUDACachingAllocator.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>

#include <torch/custom_class.h>

namespace c10d {

// Control whether or not wait() is blocking or non-blocking.
static std::vector<std::string> TORCH_NCCL_BLOCKING_WAIT = {
    "TORCH_NCCL_BLOCKING_WAIT",
    "NCCL_BLOCKING_WAIT"};

// Control whether or not we perform Async Error Handling with NCCL.
static std::vector<std::string> TORCH_NCCL_ASYNC_ERROR_HANDLING = {
    "TORCH_NCCL_ASYNC_ERROR_HANDLING",
    "NCCL_ASYNC_ERROR_HANDLING"};

// Control whether dumping debug info on watchdog
// timeout is enabled. This variable must be set together with
// TORCH_NCCL_ENABLE_MONITORING=1 and TORCH_NCCL_TRACE_BUFFER_SIZE > 0.
static std::vector<std::string> TORCH_NCCL_DUMP_ON_TIMEOUT = {
    "TORCH_NCCL_DUMP_ON_TIMEOUT"};

// Control whether Desync Debug is enabled. This variable must be set
// together with TORCH_NCCL_ASYNC_ERROR_HANDLING.
static std::vector<std::string> TORCH_NCCL_DESYNC_DEBUG = {
    "TORCH_NCCL_DESYNC_DEBUG",
    "NCCL_DESYNC_DEBUG"};

// Enable recording start-events for all ProcessGroupNCCL collectives, and
// compute accurate collective timing per-collective. (Note: end-events are
// recorded by default. Turn on this flag can increase chances of a watchdog
// hang due to performing a CUDA event query which eventually calls
// cudaEventElapsedTime() API.
static std::vector<std::string> TORCH_NCCL_ENABLE_TIMING = {
    "TORCH_NCCL_ENABLE_TIMING",
    "NCCL_ENABLE_TIMING"};

// Enable monitoring thread which aborts the process when the ProcessGroupNCCL
// Watchdog thread gets stuck and no heartbeat is detected after
// TORCH_NCCL_HEARTBEAT_TIMEOUT_SEC. This can happen due to calling CUDA/NCCL
// APIs that may hang. It is Useful to prevent jobs being stuck for a prolonged
// time than necessary tying up cluster resources.
static std::vector<std::string> TORCH_NCCL_ENABLE_MONITORING = {
    "TORCH_NCCL_ENABLE_MONITORING"};

// Control the watchdog heartbeat timeout period after which the monitoring
// thread will abort the process.
static std::vector<std::string> TORCH_NCCL_HEARTBEAT_TIMEOUT_SEC = {
    "TORCH_NCCL_HEARTBEAT_TIMEOUT_SEC"};

// The maximum number of events we store in the flight recorder's ring buffer.
// (One event could be the start or end of a collective, for example).
static std::vector<std::string> TORCH_NCCL_TRACE_BUFFER_SIZE = {
    "TORCH_NCCL_TRACE_BUFFER_SIZE"};

// Control how much extra time we will wait for dumping the debugging info
// before we exit and throws timeout exception.
static std::vector<std::string> TORCH_NCCL_WAIT_TIMEOUT_DUMP_MILSEC = {
    "TORCH_NCCL_WAIT_TIMEOUT_DUMP_MILSEC"};

// Control the interval inside the watchdog thread to check the coordinated
// signal from other ranks, e.g. to dump the debugging information.
static std::vector<std::string> TORCH_NCCL_COORD_CHECK_MILSEC = {
    "TORCH_NCCL_COORD_CHECK_MILSEC"};

// Whether to abort the communicators when users call destroy_process_group().
// If yes, communicators will be aborted when destroy_process_group is called,
// but not in destructor.
static std::vector<std::string> TORCH_NCCL_ABORT_IN_DESTROY_PG = {
    "TORCH_NCCL_ABORT_IN_DESTROY_PG"};

constexpr const char* NCCL_BACKEND_NAME = "nccl";

constexpr const char* TIMEOUT_DUMP = "timeout_dump";

constexpr const int kWorkStatusUpdatePeriodMs = 10 * 1000; // 10 seconds

constexpr auto kProcessGroupNCCLDefaultTimeout =
    std::chrono::milliseconds(10 * 60 * 1000);

// NoHandling: do not handle asynchronous NCCL errors
// TearDown: tear down process upon error, see `WorkNCCL::handleException`
// CleanUpOnly: just clean up collectives and abort communicators without
// tearing down process SkipCleanUp: (this is a temporary option and can be
// removed in future) tear down process without cleaning up NCCL communicators.
// This should be used as a last resort in case `ncclCommAbort` itself is
// hanging
enum ErrorHandlingMode {
  NoHandling = 0,
  TearDown = 1,
  CleanUpOnly = 2,
  SkipCleanUp = 3
};

#define SHOULD_CLEAN_UP(a) (a != NoHandling && a != SkipCleanUp)

#define SHOULD_TEAR_DOWN(a) (a != NoHandling && a != CleanUpOnly)

#define PRINT_COLLECTIVE_HASH_SIGNATURE(phase, opType, numel, hashValue)      \
  LOG(WARNING) << logPrefix() << "Hash of " << phase << " to NCCL " << opType \
               << " with size " << numel << " is " << hashValue;

// If set, ProcessGroupNCCL doesn't use recordStream calls to ensure
// caching allocator safety for tensors used on both user-facing and
// internal comm streams.
// Instead, it stashes live references to those tensors until after
// user-facing streams are synced with comm streams.
// See stashed_for_allocator_safety_ below.
static std::vector<std::string> TORCH_NCCL_AVOID_RECORD_STREAMS = {
    "TORCH_NCCL_AVOID_RECORD_STREAMS"};

// If set, ProcessGroupNCCL registers postAlloc and preFree hooks to cuda cache
// allocator so that whenever a tensor is allocated or freed, ProcessGroupNCCL
// can register/deregister the tensor on all available NCCL communicators.
static std::vector<std::string> TORCH_NCCL_USE_TENSOR_REGISTER_ALLOCATOR_HOOK =
    {"TORCH_NCCL_USE_TENSOR_REGISTER_ALLOCATOR_HOOK",
     "NCCL_USE_TENSOR_REGISTER_ALLOCATOR_HOOK"};

#if defined(__linux__)
struct DumpPipe {
  DumpPipe(int rank) {
    std::string fileStem =
        getCvarString({"TORCH_NCCL_DEBUG_INFO_PIPE_FILE"}, "");
    if (fileStem.empty() ||
        getCvarInt({"TORCH_NCCL_TRACE_BUFFER_SIZE"}, 0) <= 0) {
      return;
    }
    TORCH_CHECK(!fileStem.empty(), "TORCH_NCCL_DEBUG_INFO_TEMP_FILE is empty");
    std::string filename = c10::str(fileStem, rank, ".pipe");
    TORCH_CHECK(
        unlink(filename.c_str()) != -1 || errno == ENOENT,
        "Error removing existing named pipe ",
        filename);
    TORCH_CHECK(
        mkfifo(filename.c_str(), 0666) != -1,
        "Error creating named pipe ",
        filename);
    fd_ = open(filename.c_str(), O_RDONLY | O_NONBLOCK);
    LOG(INFO) << "Pipe file " << filename
              << " has been opened, write to it to trigger NCCL Debug Dump.";
    TORCH_CHECK(fd_ != -1, "Error opening named pipe ", filename);
  }
  bool shouldDump() {
    if (fd_ == -1) {
      return false;
    }
    char buf[128];
    // non-blocking from O_NONBLOCK above.
    // Ignore EINTR because we already will poll this
    // again later.
    ssize_t bytesRead = read(fd_, &buf, 128);
    return bytesRead > 0;
  }
  ~DumpPipe() {
    if (fd_ != -1) {
      close(fd_);
    }
  }

 private:
  int fd_ = -1;
};
#else
struct DumpPipe {
  DumpPipe(int rank) {}
  bool shouldDump() {
    return false;
  }
};
#endif

// ProcessGroupNCCL implements NCCL bindings for c10d.
//
// All functions of the class are expected to be called in the same order
// across all processes in the process group.  This is the only way that we
// can guarantee to match up the same calls among all processes.
//
// All NCCL functions provided by this class are asynchronous functions. More
// specifically, each NCCL call is scheduled on a separate CUDA stream that is
// different from the current CUDA stream. This is for the purpose of
// achieving potentially concurrency and better performance. As a result,
// it is the callers' responsibility to make sure that the CUDA stream their
// code works on needs to wait for the NCCL operation from
// this class.
//
// This can be done by calling:
//
// either WorkNCCL::wait() or WorkNCCL::synchronize(), both achieves the same
// functionality and are synonyms.
//
// Also note that WorkNCCL::finishedGPUExecution() is a helper function only
// provided by ProcessGroupNCCL to check if the NCCL operation of WorkNCCL has
// finished execution on the GPU (not just scheduled).
//
// Example on using the NCCL process group
//
//   ProcessGroupNCCL pg(store, rank, size);
//   std::shared_ptr<WorkNCCL> work = pg.allreduce(tensors);
//
//   // At this point, NCCL kernel has already by queued successfully
//   // Now, let current stream wait for the NCCL to finish, this function is
//   // async operation as well
//
//   work->wait()
//
//   // Now continue on other work in the current stream.
class TORCH_API ProcessGroupNCCL : public Backend {
 public:
  class WorkNCCL : public Work, public std::enable_shared_from_this<WorkNCCL> {
   public:
    friend struct WorkInfo;

    // Constructor takes a list of CUDA devices
    WorkNCCL(
        at::Device& device,
        int rank,
        OpType opType,
        uint64_t seq,
        const char* profilingTitle = nullptr,
        const c10::optional<std::vector<at::Tensor>>& inputs = c10::nullopt,
        bool desyncDebug = false,
        bool enableTiming = false,
        DebugLevel distDebugLevel = DebugLevel::Off);
    // Copy constructor doing partial copy without outputs_. Cleanup thread
    // monitors and removes finished works. However it will deadlock when
    // destructs outputs_ tensors who are view tensors in autograd graph.
    WorkNCCL(const WorkNCCL& w);

    ~WorkNCCL() override;

    // Checks if the NCCL kernel has started to execute.
    bool isStarted();

    // Checks if request has completed. In this specific case of NCCL, it checks
    // if the NCCL operation has completed on the GPU in its own NCCL stream.
    // Non-blocking operation.
    bool isCompleted() override;

    bool isSuccess() const override;

    // Same as calling synchronize() for NCCL work.
    bool wait(std::chrono::milliseconds timeout = kNoTimeout) override;

    void abort() override;

    // Let current stream wait on the completing of the NCCL work
    // Throws on exceptions. Blocking operation, which will wait for work
    // completion.
    void synchronize() override;

    // Synchronize streams by blocking each on the NCCL stream
    void synchronizeStream();

    // Helper function to handle exception (throw if needed).
    void handleException(ErrorHandlingMode asyncErrorHandling);

    // Helper function that checks if the NCCL kernels have finished
    // execution on the GPUs
    bool finishedGPUExecution();

    // Get a Future object that will be marked as completed internally.
    c10::intrusive_ptr<c10::ivalue::Future> getFuture() override;

    float getDuration() const override;

    uint64_t getSequencenumber() const override;

    const std::string& logPrefix() const;

    // Helper function that sets an exception_ptr on the WorkNCCL object.
    void setException(std::exception_ptr exception_ptr);

    // Helper function that returns True if the WorkNCCL object has timed out
    // and False otherwise.
    // In case of timeout, set exception on the WorkNCCL object.
    bool checkTimeout(
        c10::optional<std::chrono::milliseconds> timeout = c10::nullopt);

    std::vector<at::Tensor> result() override;

   protected:
    // The cached list of CUDA devices to operate on
    at::Device device_;

    // The start CUDA event of NCCL operator tracking this work item. These
    // start CUDA events are needed by desync debugging if enabled.
    std::shared_ptr<at::cuda::CUDAEvent> ncclStartEvent_;

    // The end CUDA event of NCCL operator tracking this work item.
    std::shared_ptr<at::cuda::CUDAEvent> ncclEndEvent_;

    // The NCCL communicator used for this work item.
    std::shared_ptr<NCCLComm> ncclComm_;

    // Tensors used for barrier op
    at::Tensor barrierTensor_;

    // Clone of blockingWait_ from ProcessGroupNCCL.
    bool blockingWait_ = false;

    // Clone of avoidRecordStreams_ from ProcessGroupNCCL.
    bool avoidRecordStreams_ = false;

    // Clone of opTimeout_ from ProcessGroupNCCL.
    std::chrono::milliseconds opTimeout_;

    // Time point representing when the work started.
    std::chrono::time_point<std::chrono::steady_clock> workStartTime_;

    // Record the collective sequential number.
    uint64_t seq_;

    // Indicates if the nccl start event has been updated to the store trace.
    // This will be used by desync debug.
    bool startTraceUpdated_{false};

    // Record collective sizes for debug. We only record the size on the first
    // device as multi-device per process is deprecated
    size_t numelIn_ = -1;
    size_t numelOut_ = -1;

    // Wrapper method for the static checkForNCCLErrors which can be overridden
    // for tests.
    virtual std::exception_ptr checkForNCCLErrors();

    friend std::ostream& operator<<(
        std::ostream& output,
        const WorkNCCL& workNCCL);

   private:
    // Helper function for synchronize
    void synchronizeInternal(std::chrono::milliseconds timeout);

    // Checks for NCCL errors and sets an appropriate exception_ptr.
    void checkAndSetException();

    // Just checks whether GPU execution has started, without modifying
    // exception_ptr.
    bool startedGPUExecutionInternal() const;

    // Just checks whether GPU execution has completed, without modifying
    // exception_ptr.
    bool finishedGPUExecutionInternal() const;

    // Reference to the store so that we can write aborted communicators
    // to the store.
    c10::intrusive_ptr<Store> store_;

    // Store a reference to NCCL collective's outputs, used by result and to
    // give a more descriptive message when representing the Work as a string.
    std::shared_ptr<std::vector<at::Tensor>> outputs_;

    // TORCH_NCCL_AVOID_RECORD_STREAMS implementation helper.
    // Stores references to participating non-output tensors (ie inputs,
    // flattened intermediates).
    // We'll clear this list in synchronizeStream, just after user-facing
    // stream(s) are synced with the nccl work stream(s).
    // By keeping these refs (as well as outputs_) alive until after the
    // collective's work rejoins the user-facing streams, we achieve
    // caching allocator safety without any recordStream calls.
    // For in-place collectives, some refs stashed here may alias outputs_,
    // but that doesn't do any harm.
    std::shared_ptr<std::vector<at::Tensor>> stashed_for_allocator_safety_;

    // The future returned by getFuture.
    c10::intrusive_ptr<at::ivalue::Future> future_;

    bool timingEnabled_;
    // unique id used to tell the trace buffer that this
    // work has completed
    c10::optional<uint64_t> trace_id_;
    DebugLevel distDebugLevel_;
    friend class ProcessGroupNCCL;
  };

  struct Options : Backend::Options {
    // NOTE: timeout in ProcessGroupNCCL::Options denote the timeout for
    // operations. This is only used when blockingWait_ is enabled.
    explicit Options(bool is_high_priority_stream = false);

    // return intrusive_ptr of the object
    static c10::intrusive_ptr<Options> create(
        bool is_high_priority_stream = false) {
      return c10::make_intrusive<Options>(is_high_priority_stream);
    }

    // Schedule NCCL operations on high priority CUDA streams
    bool is_high_priority_stream;

#ifdef NCCL_HAS_COMM_NONBLOCKING
    // Configure ranks
    ncclConfig_t config = NCCL_CONFIG_INITIALIZER;
#endif

    // Optional "parent" backend and color to create communicators from
    // via `ncclCommSplit`
    std::shared_ptr<ProcessGroupNCCL> split_from;
    int64_t split_color{0};
    std::vector<uint64_t> global_ranks_in_group;
  };

  // If you wish to create multiple process groups, each with a potentially
  // different rank and size, you can do so by passing a new store instance
  // to each one. If you have only a single store object, you can
  // use the `c10d::PrefixStore` to derive scoped instances.
  // This is also what the Python API in torch.distributed does.
  //
  // The process group instance keeps a reference to the store because
  // it may be used long after the constructor runs. In fact, the constructor
  // doesn't create any NCCL communicators. A single NCCL communicator can
  // only be used on a specific set of devices, and are therefore created
  // on-demand when a collective runs. If another collective is executed later,
  // against a different set of devices, the process group creates another NCCL
  // communicator. These NCCL communicators are cached and reused if possible.
  //
  ProcessGroupNCCL(
      const c10::intrusive_ptr<Store>& store,
      int rank,
      int size,
      c10::intrusive_ptr<Options> options = Options::create());

  // This constructor includes the deprecated `groupName` argument.
  // If you have existing code that uses the `groupName`, you can replace
  // it by specifying a `c10d::PrefixStore(groupName, store)` for store.
  C10_DEPRECATED ProcessGroupNCCL(
      const c10::intrusive_ptr<Store>& store,
      int rank,
      int size,
      const std::string& groupName,
      c10::intrusive_ptr<Options> options = Options::create())
      : ProcessGroupNCCL(store, rank, size, options) {}

  ~ProcessGroupNCCL() override;

  uint64_t getUid() {
    return static_cast<uint64_t>(uid_);
  }

  c10::intrusive_ptr<Options> getOptions() {
    return options_;
  }

  const std::string getBackendName() const override {
    return std::string(NCCL_BACKEND_NAME);
  }

  bool supportsSplitting() const override {
    return true;
  }

  void startCoalescing() override;

  c10::intrusive_ptr<Work> endCoalescing() override;

  // For specifying a composite optype, such as ALLGATHER and REDUCE_SCATTER
  c10::intrusive_ptr<Work> endCoalescing(OpType optype);

  c10::intrusive_ptr<Work> broadcast(
      std::vector<at::Tensor>& tensors,
      const BroadcastOptions& opts = BroadcastOptions()) override;

  c10::intrusive_ptr<Work> _broadcast_oop(
      at::Tensor& outputTensors,
      at::Tensor& inputTensors,
      const BroadcastOptions& opts = BroadcastOptions());

  c10::intrusive_ptr<Work> allreduce_sparse(
      std::vector<at::Tensor>& tensors,
      const AllreduceOptions& opts = AllreduceOptions()) override;

  c10::intrusive_ptr<Work> allreduce(
      std::vector<at::Tensor>& tensors,
      const AllreduceOptions& opts = AllreduceOptions()) override;

  c10::intrusive_ptr<Work> allreduce_coalesced(
      std::vector<at::Tensor>& tensors,
      const AllreduceCoalescedOptions& opts =
          AllreduceCoalescedOptions()) override;

  c10::intrusive_ptr<Work> reduce(
      std::vector<at::Tensor>& tensors,
      const ReduceOptions& opts = ReduceOptions()) override;

  c10::intrusive_ptr<Work> _reduce_oop(
      at::Tensor& outputTensors,
      at::Tensor& inputTensors,
      const ReduceOptions& opts = ReduceOptions());

  c10::intrusive_ptr<Work> allgather(
      std::vector<std::vector<at::Tensor>>& outputTensors,
      std::vector<at::Tensor>& inputTensors,
      const AllgatherOptions& opts = AllgatherOptions()) override;

  c10::intrusive_ptr<Work> _allgather_base(
      at::Tensor& outputbuffer,
      at::Tensor& inputbuffer,
      const AllgatherOptions& opts = AllgatherOptions()) override;

  c10::intrusive_ptr<Work> allgather_coalesced(
      std::vector<std::vector<at::Tensor>>& outputTensorLists,
      std::vector<at::Tensor>& inputTensors,
      const AllgatherOptions& opts = AllgatherOptions()) override;

  c10::intrusive_ptr<Work> allgather_into_tensor_coalesced(
      std::vector<at::Tensor>& outputs,
      std::vector<at::Tensor>& inputs,
      const AllgatherOptions& opts = AllgatherOptions()) override;

  c10::intrusive_ptr<Work> reduce_scatter(
      std::vector<at::Tensor>& outputTensors,
      std::vector<std::vector<at::Tensor>>& inputTensors,
      const ReduceScatterOptions& opts = ReduceScatterOptions()) override;

  c10::intrusive_ptr<Work> _reduce_scatter_base(
      at::Tensor& outputTensor,
      at::Tensor& inputTensor,
      const ReduceScatterOptions& opts = ReduceScatterOptions()) override;

  c10::intrusive_ptr<Work> reduce_scatter_tensor_coalesced(
      std::vector<at::Tensor>& outputs,
      std::vector<at::Tensor>& inputs,
      const ReduceScatterOptions& opts = ReduceScatterOptions()) override;

  c10::intrusive_ptr<Work> barrier(
      const BarrierOptions& opts = BarrierOptions()) override;

  c10::intrusive_ptr<Work> alltoall_base(
      at::Tensor& outputTensor,
      at::Tensor& inputTensor,
      std::vector<int64_t>& outputSplitSizes,
      std::vector<int64_t>& inputSplitSizes,
      const AllToAllOptions& opts = AllToAllOptions()) override;

  c10::intrusive_ptr<Work> alltoall(
      std::vector<at::Tensor>& outputTensors,
      std::vector<at::Tensor>& inputTensors,
      const AllToAllOptions& opts = AllToAllOptions()) override;

  c10::intrusive_ptr<Work> send(
      std::vector<at::Tensor>& tensors,
      int dstRank,
      int tag) override;

  c10::intrusive_ptr<Work> recv(
      std::vector<at::Tensor>& tensors,
      int srcRank,
      int tag) override;

  void groupStart();

  void groupEnd();

  void groupEndNonblocking(std::shared_ptr<NCCLComm> comm);

  c10::intrusive_ptr<Work> gather(
      std::vector<std::vector<at::Tensor>>& outputTensors,
      std::vector<at::Tensor>& inputTensors,
      const GatherOptions& opts = GatherOptions()) override;

  c10::intrusive_ptr<Work> scatter(
      std::vector<at::Tensor>& outputTensors,
      std::vector<std::vector<at::Tensor>>& inputTensors,
      const ScatterOptions& opts = ScatterOptions()) override;

  // Unsupported Ops
  c10::intrusive_ptr<Work> recvAnysource(
      std::vector<at::Tensor>& tensors,
      int tag) override;

  // Agrees on an initial sequence number for the whole group by having rank 0
  // create it and broadcast it to other ranks using the store.
  void setSequenceNumberForGroup() override;

  // Retrieves the current sequence number for the whole group, which should be
  // in sync. If the returned number is not consistent across the group, it
  // may indicate that there is some sort of collective desynchronization.
  uint64_t getSequenceNumberForGroup() override;

  // Return the total number of splits the communicators held by this process
  // group have performed.
  uint64_t getCommSplitCounter() const;

  void registerOnCompletionHook(
      std::function<void(std::shared_ptr<WorkInfo>)>&& hook) override;
  void waitForPendingWorks() override;

  void enableCollectivesTiming() override;

  // Helper function for iteratively aborting communicators in the provided map
  void abortCommsFromMap(
      std::unordered_map<std::string, std::shared_ptr<NCCLComm>>& ncclCommsMap,
      c10::optional<std::string> abortReason);

  c10::intrusive_ptr<intra_node_comm::IntraNodeComm> initIntraNodeComm();

  // Provides an API to abort the ProcessGroup (similar to ncclCommAbort)
  // instead of relying on ProcessGroupNCCL destructor.
  // return true if abort is successful, otherwise false
  bool abort(c10::optional<std::string> abortReason = c10::nullopt);

  void shutdown(c10::optional<std::string> reason = c10::nullopt);

  void eagerConnectSingleDevice(at::Device device) override;

  void performNocolorSplit(at::Device device);

 protected:
  // Helper that broadcasts nccl unique ID to all ranks through the store
  void broadcastUniqueNCCLID(
      ncclUniqueId* ncclID,
      bool isSingleP2POp,
      const std::string& devicesKey,
      int p2pRank);

  // Helper that either looks up the cached NCCL communicators or creates
  // a new set of NCCL communicators as a cache entry
  std::shared_ptr<NCCLComm> getNCCLComm(
      const std::string& deviceKey,
      at::Device& device,
      OpType opType,
      int p2pRank = 0,
      bool isSendRecvSelf = false);

  // Wrapper method which can be overridden for tests.
  virtual std::exception_ptr checkForNCCLErrors(
      std::shared_ptr<NCCLComm>& ncclComm);

  // Ensure thaht if record is True, the work obj will be enqueued via
  // workEnqueue
  virtual c10::intrusive_ptr<ProcessGroupNCCL::WorkNCCL> initWork(
      at::Device& device,
      int rank,
      OpType opType,
      const char* profilingTitle = nullptr,
      const std::vector<at::Tensor>& inputs = {},
      const std::vector<at::Tensor>& outputs = {},
      bool record = false);

  // In the timeout case and we will dump debug info such as the NCCL flight
  // recorder to storage. Down the road, if we have more complicated or blocking
  // operations, we might need to use a side thread to do it.
  bool dumpDebuggingInfo();

 private:
  int globalRankStart;
  int globalRankStride;

  // Helper that encapsulates work shared across all collective communication
  // primitives.  The callbacks have the following signatures:
  //
  //    ncclResult_t fn(at::Tensor& input, at::Tensor& output,
  //                    ncclComm_t, at::cuda::CUDAStream&);
  //    void {pre,post}(std::vector<at::cuda::CUDAStream&>);
  template <typename Fn>
  c10::intrusive_ptr<Work> collective(
      at::Tensor& input,
      at::Tensor& output,
      Fn fn,
      OpType opType,
      const char* profilingTitle = nullptr,
      bool avoidRecordStreams = false);

  template <typename Fn, typename PreProcess, typename PostProcess>
  c10::intrusive_ptr<Work> collective(
      at::Tensor& input,
      at::Tensor& output,
      Fn fn,
      PreProcess pre,
      PostProcess post,
      OpType opType,
      const char* profilingTitle = nullptr,
      bool avoidRecordStreams = false);

  template <typename Fn>
  c10::intrusive_ptr<Work> collectiveCoalesced(
      std::vector<at::Tensor>& input,
      std::vector<at::Tensor>& output,
      Fn fn,
      OpType opType,
      const char* profilingTitle = nullptr,
      bool avoidRecordStreams = false);

  // Helper that encapsulates work shared across point-to-point communication
  // primitives. It is the same structure as the helper used for collective
  // communication primitives.
  template <typename Fn>
  c10::intrusive_ptr<Work> pointToPoint(
      at::Tensor& tensor,
      Fn fn,
      int peer,
      OpType opType,
      const char* profilingTitle = nullptr);

  template <typename Fn, typename PreProcess, typename PostProcess>
  c10::intrusive_ptr<Work> pointToPoint(
      at::Tensor& tensor,
      Fn fn,
      int peer,
      OpType opType,
      PreProcess pre,
      PostProcess post,
      const char* profilingTitle);

  c10::intrusive_ptr<Work> allreduce_impl(
      at::Tensor& tensor,
      const AllreduceOptions& opts = AllreduceOptions());

  // Checks for NCCL errors on each of the communicators and returns an
  // appropriate exception_ptr (nullptr if no errors).
  static std::exception_ptr checkForNCCLErrorsInternal(
      std::shared_ptr<NCCLComm>& ncclComm);

  // Function that runs as part of a separate thread and checks for errors on
  // NCCL communicators. We need a separate thread to check for NCCL errors
  // since we can't rely on the user calling certain methods like wait(),
  // isCompleted() etc. to detect and remediate errors. In addition to this, we
  // need a mechanism to safely abort and remove NCCL communicators from our
  // cache. This can be done cleanly by having a thread for the ProcessGroupNCCL
  // class. Attempting to modify the communicator cache from the WorkNCCL class
  // might run into issues with object lifetime since the ProcessGroupNCCL
  // object might get destroyed before the WorkNCCL object.
  void ncclCommWatchdog();

  // Return the CUDA device most likely associated with this backend.
  // If we aren't bound to a specific device, there is no strict
  // guarantee that this heuristic is the correct assignment of ranks
  // to GPUs that Python layers use, but in practice it tends to be.
  // Fortunately we don't rely on this for correctness of any tensor
  // operations, just for ancillary uses like barriers.
  at::Device guessDeviceForRank() const;

  // Destroys initialized NCCL communicators in devNCCLComMap_ given by input
  // key. Throws if there are no communicators to destroy. Also removes
  // communicators from the cache and clears used device indices.
  void destroyNCCLComms(const std::string& devNCCLCommMapKey);

  // Watchdog's inside loop.
  // Takes care of cleaning up completed work, and aborting upon failure or
  // timeout.
  void watchdogHandler();

  void runHookLoop();

  // Desync debug helper
  void logWorkStart(WorkNCCL& work);

  // Desync debug helper
  void logWorkEnd(WorkNCCL& work);

  // Generates a prefix that is unique to this process group and rank, for
  // disambiguating logs
  std::string createLogPrefix() const;

  // Returns the unique prefix created in createLogPrefix
  const std::string& logPrefix() const;

  // Returns the global rank of the device. This function assumes that users
  // always create a default global process group(PG) which includes all
  // devices. It is called in the constructor of ProcessGroupNCCL, so it always
  // return the rank_ of the the very first PG created, aka, default global PG.
  const int& globalRank() const;

  // Returns the global ranks of a PG.
  const std::vector<uint64_t>& groupRanks() const;

 protected:
  // Function that runs as part of a separate thread aside from watchdog
  // thread because we need to check the heartbeat from watchdog thread
  // so that when we get stuck in some NCCL/CUDA calls,
  // we can dump the debugging information and abort the process.
  virtual void heartbeatMonitor();

  // Function that directly trigger std::abort so that the whole process
  // gets terminated.
  virtual void terminateProcess(std::string errMsg);

  // A helper function to wait for a future to complete or timeout.
  void waitForFutureOrTimeout(
      std::future<bool>& fut,
      const std::chrono::milliseconds& timeOutMilSec,
      const std::string& futDescription,
      bool throwException = false);

  // When watchdog timeout, this function will be called and return debug info
  // for users. For now we only get information from retrieveDesyncReport.
  // We are working on enabling more useful debug information for watchdog
  // timeout.
  virtual std::string getNCCLWatchdogDebugInfo();

  static const int64_t kWatchdogThreadSleepMillis;

  // The store is used to broadcast the NCCL unique ID of rank 0. This store
  // comes with prefix and it is different across ProcessGroup NCCL instances
  // (aka, different ProcessGroups).
  c10::intrusive_ptr<Store> store_;

  // Reference to the store without prefix so that keys are same across all
  // ProcessGroup NCCL instances and (key, value) pairs written to the store are
  // global.
  c10::intrusive_ptr<Store> globalStore_;

  bool storeError_{false};

  const c10::intrusive_ptr<Options> options_;

  // The number of NCCL communicators that have been created during
  // the lifetime of this process group. This sequence number is
  // used to scope keys used in the store.
  uint64_t ncclCommCounter_{0};

  // The store keys to trace the last NCCL collective kernel CUDA events - start
  // event and end event respectively. These are used to do desync root cause
  // analysis.
  const std::string traceKeyStart_;
  const std::string traceKeyEnd_;

  // The NCCL communicator that the process group has cached.
  //
  // For collective operations:
  // The key is a list of GPU devices that an operation is operating on
  // The GPU devices are stored in a device sequence and the cache NCCL
  // communicator is associated with this GPU device sequence
  //
  // e.g. If the process group op only uses device 0, then the value of
  // the used device string stored (value of the hashmap) would be "0".
  //
  //      If the process group op uses device 0 - 7 and the each tensor of the
  //      input tensor list is on device, 0, 1, 2, 3, 4, 5, 6, 7 separately,
  //      then the value of the used device string (key) stored would be
  //      "0,1,2,3,4,5,6,7"
  //
  //      If the process group op uses device 0 - 7 and the each tensor of the
  //      input tensor list is on device, 0, 4, 5, 6, 7, 1, 2, 3 separately,
  //      then the value of the used device string stored would be
  //      "0,4,5,6,7,1,2,3"
  //
  //      Note that the order of the device for the tensor list matters.
  //
  // For point-to-point operations:
  // The key is a string of my current rank and the peer process rank.
  // e.g. If process 1 and process 2 are involved in a point-to-point
  // communication, the key will be "1:2" on both processes. Note: this is for
  // the scenario where there is only 1 GPU per process. When it comes to
  // multiple GPUs per process, this part may need to redesigned.
  std::unordered_map<std::string, std::shared_ptr<NCCLComm>> devNCCLCommMap_;

  // The NCCL communicators currently in process of being initialized.
  std::unordered_map<std::string, std::shared_ptr<NCCLComm>>
      inInitializationCommMap_;

  // Map from ncclUniqueId to appropriate communicator.
  std::unordered_map<std::string, std::shared_ptr<NCCLComm>> ncclIdToCommMap_;

  // Mutex to guard maps like devNCCLCommMap_ and ncclIdToCommMap_.
  std::mutex mutex_;

  // Heartbeat of watchdog thread.
  std::atomic_uint64_t heartbeat_;

  // The time interval used for deciding whether there is no watchdog heartbeat.
  int heartbeatTimeoutInSec_;

  // timeout for the dump to finish.
  int waitTimeoutDumpInMilSec_;

  // Interval of check coordinated signals in ProcessGroupNCCL from other ranks
  // e.g., trigger the dump of the debugging info for timeout when notified.
  int coordCheckIntervalMilSec_;

  // Size of ring buffer where we store NCCL Traces for debugging.
  int ncclTraceBufferSize_;

  // We gate the heartbeat monitor thread so that we can roll it out gradually.
  std::atomic<bool> monitorThreadEnabled_;

  // Monitor thread which checks the heartbeat of Watchdog thread.
  // If the monitor thread finds there is no heartbeat, it will dump debug info
  // and then kill the watchdog thread to avoid hang.
  std::thread ncclHeartbeatMonitorThread_;

  // Watchdog thread which looks for errors on the cached NCCL communicators.
  std::thread ncclCommWatchdogThread_;

  std::thread onCompletionHookThread_;

  // Whether or not we should terminate the watchdog and workCleanup threads.
  std::atomic<bool> terminateProcessGroup_;

  // Whether or not we should terminate the heartbeat monitoring threads.
  std::atomic<bool> terminateHeartbeatMonitorThread_;

  // Whether we are in the shutdown mode when we are trying to get debug info,
  // such as desync report.
  std::atomic<bool> collectiveDebugInfoMode_;

  // Whether there are hooks pending to be fired
  std::atomic<bool> hasPendingHooks_;

  // This is the signal from watchdog threads to indicate whether the monitor
  // thread should dump. Making it static so that it is accessiable from all the
  // PGs. With this flag, monitor thread would dump debug info under any one of
  // the 3 conditions: 1: this flag is set to true by the watchdog thread when
  // it detects a timeout. 2: timeout signal is received from
  // other ranks through tcpstore 3: no heartbeat of watchdog Note that only the
  // monitor thread from PG0 should dump the debug info and only once
  static std::atomic<bool> shouldDump_;

  // Mutex to Guard workMetaList_
  std::mutex workMetaListMutex_;

  // Mutex to Guard monitorWakeUpCV_
  std::mutex monitorMutex_;

  bool writeDebugInfo_ = false;

  // Condition Variable for watchdog thread sleep
  std::condition_variable workMetaListCV_;

  // Condition Variable for monitor thread to wake up early
  std::condition_variable monitorWakeUpCV_;

  // Vector to Store WorkNCCL pointers
  std::list<ProcessGroupNCCL::WorkNCCL> workMetaList_;

  std::chrono::time_point<std::chrono::steady_clock> lastWorkListUpdateTime_;

  // Mutex to Guard workMetaList_
  std::mutex completedWorkListMutex_;

  // Condition Variable for watchdog thread sleep
  std::condition_variable completedWorkListCV_;

  std::list<ProcessGroupNCCL::WorkNCCL> completedWorkList_;

  // Add Work Pointer to workVector
  void workEnqueue(c10::intrusive_ptr<ProcessGroupNCCL::WorkNCCL>);

  // The CUDA streams used by NCCL kernels
  std::unordered_map<std::string, at::cuda::CUDAStream> ncclStreams_;

  // The CUDA events used to sync NCCL streams
  std::unordered_map<std::string, at::cuda::CUDAEvent> ncclEvents_;

  // Device Indexes used for all collectives in this group
  std::set<int> usedDeviceIdxs_;

  // Flag to denote if a coalescing groupStart/groupEnd block is active
  int coalescing_state_ = 0;

  // Stores device indexes for all collectives run inside a coalescing block
  std::vector<at::Device> coalescedDevices_;

  // Stores communicators for all collectives run inside a coalescing block
  std::vector<std::shared_ptr<NCCLComm>> coalescedComms_;

  // map from the key: "group name + pg counter (ID)" to the
  // unique NCCL ID count. This needs to be group and pg specific
  //
  // For each process group, we need a uniform unique NCCL ID counter to ensure
  // that NCCL operation in this process group can be completed successfully.
  // Since each process group ID belongs to a group name, the key to this map
  // is a combination of group name and ProcessGroupNCCL ID.
  static std::unordered_map<std::string, ssize_t> pgUniqueNCCLIDCnt_;

  // map from group name to the pg counter (ID) within that group
  //
  // For each group with the "group name" (which is the key), we need to
  // keep track of a unique process group ID when creating a new
  // ProcessGroupNCCL for this "group name". Therefore, the value of this
  // map keeps the unique ProcessGroupNCCL's ID for a specific group with
  // the "group name". The reason we need a per-group process group ID counter
  // is that different group can have different ranks and we need ensure that
  // each group has its own uniform process group ID for all its ranks.
  static std::unordered_map<std::string, ssize_t> processGroupCounterMap_;

  // Whether or not wait() and synchronize() are blocking operations that wait
  // for the operation to complete.
  bool blockingWait_ = false;

  // Whether to abort the communicators when users call destroy_process_group().
  // If yes, communicators will be aborted when destroy_process_group is called,
  // but not in destructor.
  bool abortInDestroyProcessGroup_ = false;

  // Whether or not to hook the cache allocator to register all allocated
  // tensors
  bool useTensorRegisterAllocatorHook_ = false;

  // Whether or not the workCleanupThread is used to perform async error
  // handling.
  ErrorHandlingMode asyncErrorHandling_ = NoHandling;

  // Whether or not to enable timeout root cause analysis.
  bool desyncDebug_;

  // Whether or not to dump debug info on timeout
  bool dumpOnTimeout_;

  // Whether or not to create start CUDAEvent and enable timing for start
  // and end events. Note that enableTiming_ is always true if desyncDebug_
  // is set to true.
  std::atomic<bool> enableTiming_;

  // Flag to enable the print of hash value of input/output of collectives for
  // verification.
  std::atomic<bool> enableCollecticeHashDebug_;

  // Whether or not TORCH_NCCL_AVOID_RECORD_STREAMS was set
  bool avoidRecordStreams_ = false;

  // Set of communicators that this process group has aborted and their
  // ncclUniqueId has been written to the store. We don't need a lock
  // for this map since only the watchdog thread accesses this set. The
  // set contains the string representation of ncclUniqueId.
  std::unordered_set<std::string> abortedComms_;

  // The number of active ncclGroupStart() calls. This counter will be increased
  // by 1 when ncclGroupStart() is called and decreased by 1 when ncclGroupEnd()
  // is called.
  static thread_local uint64_t ncclActiveGroupCounter_;

  // Counting for the sequential number of NCCL collective call.
  // (specifically, how many actual kernels we launched, which differs from
  // op_id_ when coalescing is enabled)
  uint64_t seq_{0};

  // Incrementing counter for logical operations (collective or p2p) issued on
  // the ProcessGroup
  uint64_t op_id_{0};

  // the sequential number of the last colletive enqueued into workMetaList_
  // This is useful for indentifying a rank that has not join a collective
  uint64_t lastEnqueuedSeq_;

  // the sequential number of the last colletive completed marked by
  // the watchdog thread
  uint64_t lastCompletedSeq_;

  std::exception_ptr watchDogException_ = nullptr;

  size_t uid_;

  std::string logPrefix_;

  c10::intrusive_ptr<intra_node_comm::IntraNodeComm> intraNodeComm_;
};

TORCH_API std::string dump_nccl_trace();

// Gets a mutable reference to a global optional function.  Heartbeat Monitor
// will query this function and if available, call it to dump traces. Inside
// fbcode, we store a function here that uses an internal tool for process
// tracing
TORCH_API c10::optional<std::function<std::string()>>& get_cpp_trace_dumper();

// Similar to get_cpp_trace_dumper, this stores a function defined in
// torch-python layer that lets us check whether the GIL can be acquired,
// helpful for instrumenting in cases where a hang was observed.
typedef bool (*gil_checker_t)();

TORCH_API gil_checker_t& get_gil_checker();
} // namespace c10d

#endif // USE_C10D_NCCL
