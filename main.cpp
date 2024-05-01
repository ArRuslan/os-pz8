#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <thread>
#include <unistd.h>
#include <windows.h>

struct Task1ThreadParam {
    uint32_t thread_id;
    std::mutex* thread_mtx;
    std::mutex* queue_mtx;
    std::queue<uint32_t>* queue;
    double time_ms;
};

bool access_queue(Task1ThreadParam* thread, uint32_t& result) {
    void* threadHandle = GetCurrentThread();

    timespec timesp{};
    clock_gettime(CLOCK_MONOTONIC, &timesp);
    double time = (timesp.tv_sec + timesp.tv_nsec / 1000000000.0) * 1000;
    bool increase_priority = time - thread->time_ms > 0.1;
    thread->time_ms = time;

    int currentPrio = GetThreadPriority(threadHandle);
    if (increase_priority && currentPrio <= THREAD_PRIORITY_NORMAL) {
        SetThreadPriority(threadHandle, THREAD_PRIORITY_ABOVE_NORMAL);
        printf("Increased priority for thread %d to ABOVE_NORMAL\n", thread->thread_id);
    } else if (increase_priority && currentPrio == THREAD_PRIORITY_ABOVE_NORMAL) {
        SetThreadPriority(threadHandle, THREAD_PRIORITY_HIGHEST);
        printf("Increased priority for thread %d to HIGHEST\n", thread->thread_id);
    } else {
        SetThreadPriority(threadHandle, THREAD_PRIORITY_NORMAL);
    }

    thread->queue_mtx->lock();
    if (thread->queue->empty()) {
        thread->queue_mtx->unlock();
        return false;
    }
    result = thread->queue->front();
    thread->queue->pop();
    thread->queue_mtx->unlock();

    return true;
}

uint32_t WINAPI task1_thread(void* param) {
    auto par = static_cast<Task1ThreadParam*>(param);
    par->thread_mtx->lock();

    uint32_t result = 1;
    uint32_t val;
    while (true) {
        if (!access_queue(par, val))
            break;
        result *= val;
    }

    par->thread_mtx->unlock();
    free(param);
    return 0;
}

void task1() {
    constexpr uint32_t threadCount = 8;
    void** threads = new void*[threadCount];
    std::queue<uint32_t> queue;
    std::mutex queue_mtx;

    for (uint32_t i = 0; i < 1024; i++) {
        queue.push(i);
    }

    queue_mtx.lock();
    for (int i = 0; i < threadCount; i++) {
        auto threadInfo = static_cast<Task1ThreadParam*>(malloc(sizeof(Task1ThreadParam)));
        auto thread_mtx = std::mutex();
        threadInfo->thread_mtx = &thread_mtx;
        threadInfo->queue = &queue;
        threadInfo->queue_mtx = &queue_mtx;

        thread_mtx.lock();
        threads[i] = CreateThread(nullptr, 0, &task1_thread, threadInfo, 0, &threadInfo->thread_id);
        SetThreadPriority(threads[i], THREAD_PRIORITY_NORMAL);

        timespec time{};
        clock_gettime(CLOCK_MONOTONIC, &time);
        threadInfo->time_ms = (time.tv_sec + time.tv_nsec / 1000000000.0) * 1000;

        thread_mtx.unlock();
    }
    queue_mtx.unlock();

    WaitForMultipleObjects(threadCount, threads, true, INFINITE);

    for (int i = 0; i < threadCount; i++) {
        CloseHandle(threads[i]);
    }

    delete[] threads;
}

uint32_t WINAPI task2_thread(void* param) {
    uint64_t n = 1024 * 1024 * 512, a = 0, b = 1, c = 0;

    for (int i = 1; i <= n; ++i) {
        if(i == 1 || i == 2)
            continue;
        c = a + b;
        a = b;
        b = c;
    }

    return c;
}

void task2() {
    constexpr uint32_t threadCount = 8;
    std::vector Qs = {std::deque<void*>(), std::deque<void*>(), std::deque<void*>(), std::deque<void*>()};

    for (int i = 0; i < threadCount; i++) {
        uint32_t threadId;
        Qs[0].push_back(CreateThread(nullptr, 0, &task2_thread, nullptr, CREATE_SUSPENDED, &threadId));
        SetThreadPriority(Qs[0][Qs[0].size()-1], THREAD_PRIORITY_BELOW_NORMAL);
    }

    while(!Qs[0].empty() || !Qs[1].empty() || !Qs[2].empty() || !Qs[3].empty()) {
        for(int i = 0; i < Qs.size(); i++) {
            for(auto& thread : Qs[i])
                ResumeThread(thread);
            std::this_thread::sleep_for(std::chrono::milliseconds(200 * (int)std::pow(2, i)));
            for(auto& thread : Qs[i])
                SuspendThread(thread);
            int j = 0;
            while(j < Qs[i].size()) {
                void* thread = Qs[i][j];
                uint32_t tmp;
                GetExitCodeThread(thread, &tmp);
                if(tmp != STILL_ACTIVE) {
                    printf("Thread %p is finished in queue %d\n", thread, i);
                    CloseHandle(thread);
                    Qs[i].erase(Qs[i].begin()+j);
                    continue;
                }
                if(i == Qs.size()-1) {
                    j++;
                    continue;
                }
                Qs[i].erase(Qs[i].begin()+j);
                Qs[i+1].push_back(thread);
                SetThreadPriority(thread, i - 1);
                printf("Thread %p moved from queue %d to %d, priority increased\n", thread, i, i+1);
            }
        }
    }
}

int main() {
    task1();
    task2();

    return 0;
}
