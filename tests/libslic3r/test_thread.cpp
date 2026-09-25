#include <catch2/catch_all.hpp>

#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Thread.hpp"

#include <atomic>
#include <chrono>
#include <clocale>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

using namespace Slic3r;

TEST_CASE("Naming the TBB pool returns when the scheduler grants the calling thread no workers", "[Thread]")
{
    // The calling thread's arena still reports every hardware thread as its concurrency, but no worker may join it.
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);

    auto        named = std::make_shared<std::promise<void>>();
    auto        done  = named->get_future();
    std::thread caller([named] {
        name_tbb_thread_pool_threads_set_locale();
        named->set_value();
    });
    const bool returned = done.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
    // A caller stuck waiting for workers never returns, so it cannot be joined.
    if (returned)
        caller.join();
    else
        caller.detach();
    REQUIRE(returned);
}

TEST_CASE("TBB workers print a decimal point under a comma locale once a thread names the pool", "[Thread]")
{
    const std::string original = std::setlocale(LC_NUMERIC, nullptr);
    bool              comma    = false;
    for (const char *name : {"de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "fr_FR.utf8", "de-DE"})
        if (std::setlocale(LC_NUMERIC, name) != nullptr && !is_decimal_separator_point()) {
            comma = true;
            break;
        }
    if (!comma) {
        std::setlocale(LC_NUMERIC, original.c_str());
        SKIP("No locale with a comma decimal separator is installed");
    }

    std::atomic<size_t> worker_tasks{0};
    std::atomic<size_t> comma_tasks{0};
    std::thread([&worker_tasks, &comma_tasks] {
        name_tbb_thread_pool_threads_set_locale();
        const auto caller = std::this_thread::get_id();
        // Tasks that sleep give every worker time to steal one.
        tbb::parallel_for(tbb::blocked_range<size_t>(0, 256, 1), [&](const tbb::blocked_range<size_t> &range) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (std::this_thread::get_id() != caller) {
                ++worker_tasks;
                if (!is_decimal_separator_point())
                    ++comma_tasks;
            }
        });
    }).join();
    std::setlocale(LC_NUMERIC, original.c_str());

    REQUIRE(worker_tasks > 0);
    CHECK(comma_tasks == 0);
}
