// Copyright (c) 2024 Andrei Gramakov. All rights reserved.

#include <stdio.h>
#include "ulog.h"

int main(int argc, char *argv[])
{
    log_info("main() started");
    if (argc > 1) {
        printf("Hello, %s!\n", argv[1]);
    } else {
    printf("Hello!\n");
    }
    return 0;
}
