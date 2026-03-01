#include "simple_test.h"
#include "../src/misc/MD5.h"

TEST(MD5Test, BasicHash) {
    char *data = (char*)"hello world";
    unsigned char *hash = MD5::make_hash(data);
    char *digest = MD5::make_digest(hash, 16);

    EXPECT_STREQ(digest, "5eb63bbbe01eeed093cb22bb8f5acdc3");

    free(hash);
    free(digest);
}

TEST(MD5Test, EmptyString) {
    char *data = (char*)"";
    unsigned char *hash = MD5::make_hash(data);
    char *digest = MD5::make_digest(hash, 16);

    EXPECT_STREQ(digest, "d41d8cd98f00b204e9800998ecf8427e");

    free(hash);
    free(digest);
}

int main() {
    return RUN_ALL_TESTS();
}
