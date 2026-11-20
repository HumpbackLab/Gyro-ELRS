#include "RxScanOrder.h"
#include <unity.h>

// SX128 RF table: F1000, F500, D500, D250, 500, 333 Full, 250, 150, 100 Full, 50.
static const uint32_t intervals[] = {1000, 2000, 1000, 1000, 2000, 3003, 4000, 6666, 10000, 20000};
static uint32_t window(uint8_t rate)
{
    const uint8_t hop = rate < 4 || rate == 9 ? 2 : 4;
    return rxScanWindowMs(80, hop, intervals[rate]);
}

void setUp() {}
void tearDown() {}

static void fresh_scan_orders_by_window_and_never_repeats_startup()
{
    RxScanOrder<10> scan;
    TEST_ASSERT_EQUAL(0, scan.begin(0, window));
    const uint8_t expected[] = {2, 3, 1, 4, 5, 6, 7, 8, 9, 0};
    for (auto rate : expected) TEST_ASSERT_EQUAL(rate, scan.next());
}

static void saved_rate_is_first_then_all_other_modes_by_window()
{
    const uint8_t sorted[] = {0, 2, 3, 1, 4, 5, 6, 7, 8, 9};
    for (uint8_t saved = 0; saved < 10; ++saved)
    {
        RxScanOrder<10> scan;
        TEST_ASSERT_EQUAL(saved, scan.begin(saved, window));
        for (auto rate : sorted)
            if (rate != saved) TEST_ASSERT_EQUAL(rate, scan.next());
        TEST_ASSERT_EQUAL(saved, scan.next());
    }
}

static void excludes_unsupported_and_handles_invalid_saved_index()
{
    RxScanOrder<10> scan;
    auto supported = [](uint8_t rate) { return rate == 0 || rate == 2 || rate == 7 ? uint32_t(0) : window(rate); };
    TEST_ASSERT_EQUAL(3, scan.begin(255, supported));
    const uint8_t expected[] = {1, 4, 5, 6, 8, 9, 3};
    for (auto rate : expected) TEST_ASSERT_EQUAL(rate, scan.next());
    TEST_ASSERT_EQUAL(3, scan.begin(2, supported));
}

static void binding_restart_and_single_mode_do_not_skip_the_first_rate()
{
    RxScanOrder<10> scan;
    TEST_ASSERT_EQUAL(6, scan.begin(6, window));
    scan.next();
    scan.restart();
    TEST_ASSERT_EQUAL(6, scan.next());
    TEST_ASSERT_EQUAL(0, scan.next());
    TEST_ASSERT_EQUAL(4, scan.begin(0, [](uint8_t rate) { return rate == 4 ? window(rate) : uint32_t(0); }));
    for (int i = 0; i < 30; ++i) TEST_ASSERT_EQUAL(4, scan.next());
    scan.restart();
    TEST_ASSERT_EQUAL(4, scan.next());
}

static void windows_preserve_the_existing_hopping_margin()
{
    TEST_ASSERT_EQUAL(176, window(0));
    TEST_ASSERT_EQUAL(352, window(1));
    TEST_ASSERT_EQUAL(176, window(2));
    TEST_ASSERT_EQUAL(176, window(3));
    TEST_ASSERT_EQUAL(704, window(4));
    TEST_ASSERT_EQUAL(1057, window(5));
    TEST_ASSERT_EQUAL(1408, window(6));
    TEST_ASSERT_EQUAL(3520, window(8));
    TEST_ASSERT_EQUAL(3520, window(9));
    // A different band's channel count must affect ordering, not just Hz.
    TEST_ASSERT_EQUAL(228, rxScanWindowMs(13, 4, 4000));
    TEST_ASSERT_EQUAL(1408, rxScanWindowMs(80, 4, 4000));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(fresh_scan_orders_by_window_and_never_repeats_startup);
    RUN_TEST(saved_rate_is_first_then_all_other_modes_by_window);
    RUN_TEST(excludes_unsupported_and_handles_invalid_saved_index);
    RUN_TEST(binding_restart_and_single_mode_do_not_skip_the_first_rate);
    RUN_TEST(windows_preserve_the_existing_hopping_margin);
    return UNITY_END();
}
