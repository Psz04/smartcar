#include "vehicle_comm.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/system/error_code.hpp>

#include "protocol.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

typedef struct {
    std::vector<uint8_t> data;
    int text;
} OutMessage;

struct CommContext {
    net::io_context ioc;
    tcp::resolver resolver;
    std::unique_ptr<websocket::stream<tcp::socket> > ws;
    std::mutex connect_mutex;

    CommContext() : resolver(ioc) {}
};

static std::unique_ptr<CommContext> g_context;
static std::mutex g_queue_mutex;
static std::condition_variable g_queue_cv;
static std::queue<OutMessage> g_queue;
static std::atomic<int> g_running(0);
static std::atomic<int> g_initialized(0);
static std::string g_host;
static uint16_t g_port = 0;
static VehicleComm_CommandCallback g_callback = NULL;
static std::thread g_send_thread;
static std::thread g_receive_thread;

static void push_out_message(OutMessage *message)
{
    {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        g_queue.push(std::move(*message));
    }
    g_queue_cv.notify_one();
}

static int ensure_connected(void)
{
    if (!g_context) {
        return 0;
    }
    if (g_context->ws && g_context->ws->is_open()) {
        return 1;
    }

    std::lock_guard<std::mutex> lock(g_context->connect_mutex);
    if (g_context->ws && g_context->ws->is_open()) {
        return 1;
    }

    try {
        typedef websocket::stream<tcp::socket> WsStream;
        g_context->ws.reset(new WsStream(g_context->ioc));
        std::string port_text = std::to_string((int)g_port);
        auto results =
            g_context->resolver.resolve(g_host, port_text);
        net::connect(g_context->ws->next_layer(), results.begin(), results.end());
        g_context->ws->handshake(g_host + ":" + port_text, "/");
        std::cout << "[Comm] connected to " << g_host << ":" << port_text << std::endl;
        return 1;
    } catch (const std::exception &e) {
        std::cerr << "[Comm] connect failed: " << e.what() << std::endl;
        g_context->ws.reset();
        return 0;
    }
}

static void send_loop(void)
{
    while (g_running.load() != 0) {
        OutMessage message;
        {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            g_queue_cv.wait(lock, [] {
                return g_running.load() == 0 || !g_queue.empty();
            });
            if (g_running.load() == 0 && g_queue.empty()) {
                break;
            }
            message = std::move(g_queue.front());
            g_queue.pop();
        }

        if (!ensure_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        try {
            g_context->ws->text(message.text != 0);
            g_context->ws->write(net::buffer(message.data));
        } catch (const std::exception &e) {
            std::cerr << "[Comm] send failed: " << e.what() << std::endl;
            g_context->ws.reset();
        }
    }
}

static void receive_loop(void)
{
    while (g_running.load() != 0) {
        if (!ensure_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        beast::flat_buffer buffer;
        try {
            g_context->ws->read(buffer);
            std::string text = beast::buffers_to_string(buffer.data());
            if (g_callback != NULL) {
                g_callback(text.c_str());
            }
        } catch (const std::exception &e) {
            if (g_running.load() != 0) {
                std::cerr << "[Comm] receive failed: " << e.what() << std::endl;
            }
            g_context->ws.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

/**
 * @brief 初始化车辆通信模块。
 * @param host 上位机 IP 地址。
 * @param port 上位机端口。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int VehicleComm_Init(const char *host, uint16_t port)
{
    if (host == NULL || port == 0) {
        return -1;
    }
    g_host = host;
    g_port = port;
    g_initialized.store(1);
    return 0;
}

/**
 * @brief 启动车辆通信收发线程。
 * @param callback 上位机文本指令回调。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int VehicleComm_Start(VehicleComm_CommandCallback callback)
{
    if (g_initialized.load() == 0 || g_running.load() != 0) {
        return -1;
    }

    g_callback = callback;
    g_context.reset(new CommContext());
    g_running.store(1);

    g_send_thread = std::thread(send_loop);
    g_receive_thread = std::thread(receive_loop);
    return 0;
}

/**
 * @brief 停止车辆通信并释放资源。
 * @retval 无。
 */
void VehicleComm_Stop(void)
{
    if (g_running.load() == 0) {
        return;
    }

    g_running.store(0);
    g_queue_cv.notify_all();

    if (g_context && g_context->ws) {
        boost::system::error_code ec;
        g_context->ws->next_layer().close(ec);
    }

    if (g_send_thread.joinable()) {
        g_send_thread.join();
    }
    if (g_receive_thread.joinable()) {
        g_receive_thread.join();
    }

    g_context.reset();
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    while (!g_queue.empty()) {
        g_queue.pop();
    }
}

/**
 * @brief 发送一帧图像。
 * @param jpeg JPEG 数据。
 * @param jpeg_len JPEG 数据长度。
 * @param meta 图像 meta 文本。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int VehicleComm_SendImage(const uint8_t *jpeg, size_t jpeg_len, const char *meta)
{
    if (jpeg == NULL || meta == NULL || g_initialized.load() == 0) {
        return -1;
    }

    size_t total_size = 1 + 4 + strlen(meta) + jpeg_len;
    std::vector<uint8_t> frame(total_size);
    int written = Protocol_BuildImageFrame(jpeg, jpeg_len, meta,
                                           frame.data(), frame.size());
    if (written < 0) {
        return -1;
    }
    frame.resize((size_t)written);

    OutMessage message;
    message.data = std::move(frame);
    message.text = 0;
    push_out_message(&message);
    return 0;
}

/**
 * @brief 发送文本消息。
 * @param text 待发送文本。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int VehicleComm_SendText(const char *text)
{
    if (text == NULL || g_initialized.load() == 0) {
        return -1;
    }

    OutMessage message;
    const uint8_t *begin = (const uint8_t *)text;
    const uint8_t *end = begin + strlen(text);
    message.data.assign(begin, end);
    message.text = 1;
    push_out_message(&message);
    return 0;
}