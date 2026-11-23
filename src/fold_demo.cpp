#include "tcs/communication/Communications.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <thread>

namespace {

volatile std::sig_atomic_t running = 1;

void signalHandler(int) {
    running = 0;
}

} // namespace

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::atomic<int> latest_fold{-1};
    std::puts("等待 MCU 折叠状态数据，Ctrl+C 退出。");
    std::fflush(stdout);
    tcs::McuCommunication serial([&latest_fold](const tcs::mcu::ReceivePacket& packet) {
        if (packet.data_size != 33) {
            throw std::runtime_error("MCU 接收包长度错误，fold 需要 33 字节载荷");
        }
        if (packet.fold > 1) {
            throw std::runtime_error("MCU fold 字段非法，允许值为 0 或 1");
        }
        latest_fold.store(packet.fold);
    });

    int displayed_fold = -1;
    while (running) {
        const int fold = latest_fold.load();
        // 首次接收及状态变化时显示，避免高频报文重复输出。
        if (fold != displayed_fold) {
            std::printf("fold=%d，当前状态：%s\n", fold,
                        fold == 0 ? "云台折叠" : "云台升起（未折叠）");
            std::fflush(stdout);
            displayed_fold = fold;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    serial.stopWorker();
    return 0;
}
