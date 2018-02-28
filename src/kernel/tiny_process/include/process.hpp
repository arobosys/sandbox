#ifndef TINY_PROCESS_LIBRARY_HPP_
#define TINY_PROCESS_LIBRARY_HPP_

#include <string>
#include <functional>
#include <vector>
#include <mutex>
#include <thread>
#include <memory>
#include <algorithm>
#include <list>
#include <sys/wait.h>
#include <sys/prctl.h>

///Create a new process given command and run path.
class Process {
public:
    typedef pid_t id_type;
    typedef int fd_type;
    typedef std::string string_type;
private:
    struct Data {
        id_type id;
        Data();
    };

public:
    Process(const string_type &command, const string_type &path = string_type(),
            std::function<void(int)> process_finished = nullptr,
            std::function<void(const char *bytes, size_t n)> read_stdout = nullptr,
            std::function<void(const char *bytes, size_t n)> read_stderr = nullptr,
            bool open_stdin = false,
            size_t buffer_size = 131072);

    ~Process();

    ///Get the process id of the started process.
    id_type get_id();

    ///Wait until process is finished, and return exit status.
    int get_exit_status();

    ///Write to stdin.
    bool write(const char *bytes, size_t n);

    ///Write to stdin. Convenience function using write(const char *, size_t).
    bool write(const std::string &data);

    ///Close stdin. If the process takes parameters from stdin, use this to notify that all parameters have been sent.
    void close_stdin();

    ///Kill the process.
    void kill(bool force = false);

    ///Kill a given process id. Use kill(bool force) instead if possible.
    static void kill(id_type id, bool force = false);

private:
    Data data;
    bool closed;
    std::mutex close_mutex;
    std::function<void(int)> process_finished;
    std::function<void(const char *bytes, size_t n)> read_stdout;
    std::function<void(const char *bytes, size_t n)> read_stderr;
    std::thread stdout_thread, stderr_thread;
    bool open_stdin;
    std::mutex stdin_mutex;
    size_t buffer_size;

    std::unique_ptr<fd_type> stdout_fd, stderr_fd, stdin_fd;

    id_type open(const string_type &command, const string_type &path);

    void async_read();

    void close_fds();

    static std::list<Process *> instances;

    static void callHandlers(int signum);

    void sigChld();
};

#endif  // TINY_PROCESS_LIBRARY_HPP_
