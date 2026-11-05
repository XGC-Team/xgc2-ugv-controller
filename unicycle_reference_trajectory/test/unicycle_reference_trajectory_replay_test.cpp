// The original runtime supplies the numerical/publication oracle for the same explicit-future
// request fixture. Only retired event source/category/sequence metadata is normalized. The
// original start-now hash remains reference_replay.sha256; receipt-time correction is tested
// separately, so the candidate never supplies its own expected numerical values.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "replay/reference_replay_core.h"
#include "replay/sha256.h"

namespace {

std::string trace() {
    char* buffer = nullptr;
    size_t size = 0;
    FILE* stream = open_memstream(&buffer, &size);
    reference_replay_core::writeTrace(stream);
    std::fclose(stream);
    std::string text(buffer, size);
    std::free(buffer);
    return text;
}

std::string hashOf(const std::string& text) {
    reference_replay::Sha256 sha;
    sha.update(text.data(), text.size());
    return sha.hex();
}

TEST(ReferenceReplay, Sha256ImplementationMatchesKnownDigests) {
    EXPECT_EQ(hashOf(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(hashOf("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(hashOf(std::string(1000, 'a')),
              "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
}

TEST(ReferenceReplay, AbsoluteTimeCoreReproducesTheOriginalRuntime) {
    std::ifstream file(REPLAY_SHA256_FILE);
    std::string expected;
    file >> expected;
    ASSERT_EQ(expected.size(), 64U) << REPLAY_SHA256_FILE;
    const std::string text = trace();
    EXPECT_EQ(hashOf(text), expected);
    // The scenario ran every branch it scripts.
    EXPECT_NE(text.find("run default\n"), std::string::npos);
    EXPECT_NE(text.find("run limits\n"), std::string::npos);
    EXPECT_NE(text.find("run default_analytic\n"), std::string::npos);
    EXPECT_NE(text.find("  rejected\n"), std::string::npos);
    EXPECT_NE(text.find(" polynomial hdr"), std::string::npos);
    EXPECT_NE(text.find(" sampled hdr"), std::string::npos);
}

TEST(ReferenceReplay, TraceIsDeterministic) {
    EXPECT_EQ(trace(), trace());
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
