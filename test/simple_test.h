#ifndef SIMPLE_TEST_H
#define SIMPLE_TEST_H

#include <iostream>
#include <vector>
#include <functional>
#include <string>
#include <cstring>

// Minimal Test Framework
struct Test {
    std::string name;
    std::function<void()> func;
};

static std::vector<Test>& get_tests() {
    static std::vector<Test> tests;
    return tests;
}

struct TestRegistrar {
    TestRegistrar(std::string name, std::function<void()> func) {
        get_tests().push_back({name, func});
    }
};

struct TestFailedException { std::string msg; };

#define TEST(suite, name) \
    void test_##suite##_##name(); \
    static TestRegistrar registrar_##suite##_##name(#suite "." #name, test_##suite##_##name); \
    void test_##suite##_##name()

#define EXPECT_TRUE(cond) if (!(cond)) throw TestFailedException{"Expected true: " #cond}
#define EXPECT_FALSE(cond) if (cond) throw TestFailedException{"Expected false: " #cond}
#define EXPECT_STREQ(a, b) if (strcmp(a, b) != 0) throw TestFailedException{std::string("Strings not equal: ") + a + " != " + b}

inline int run_all_tests() {
    int passed = 0;
    int failed = 0;
    for (const auto& t : get_tests()) {
        try {
            t.func();
            std::cout << "[PASS] " << t.name << std::endl;
            passed++;
        } catch (const TestFailedException& e) {
            std::cout << "[FAIL] " << t.name << ": " << e.msg << std::endl;
            failed++;
        } catch (...) {
            std::cout << "[FAIL] " << t.name << ": Unknown exception" << std::endl;
            failed++;
        }
    }
    std::cout << "\nResult: " << passed << " passed, " << failed << " failed." << std::endl;
    return failed > 0 ? 1 : 0;
}

#define RUN_ALL_TESTS() run_all_tests()

#endif
