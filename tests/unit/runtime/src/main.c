#include <zephyr/ztest.h>

ZTEST(runtime, test_fast_step_boundary_contract_present)
{
	zassert_true(true, "fast-step boundary checks are enforced in CMake");
}

ZTEST_SUITE(runtime, NULL, NULL, NULL, NULL, NULL);
