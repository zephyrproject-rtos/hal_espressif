#!/bin/bash
for dir in esp32 esp32s2 esp32c3 esp32s3 esp32c2 esp32c6 esp32h2 esp32c5 esp32c61 esp32s31 esp32h4 esp32h21; do
    if [ $dir = esp32 ]; then
        TOOLCHAIN="xtensa-esp32-elf"
    elif [ $dir = esp32s2 ]; then
        TOOLCHAIN="xtensa-esp32s2-elf"
    elif [ $dir = esp32c3 -o $dir = esp32c2 -o $dir = esp32c6 -o $dir = esp32h2 -o $dir = esp32c5 -o $dir = esp32c61 -o $dir = esp32s31 -o $dir = esp32h4 -o $dir = esp32h21 ]; then
        TOOLCHAIN="riscv32-esp-elf"
    elif [ $dir = esp32s3 ]; then
        TOOLCHAIN="xtensa-esp32s3-elf"
    else
        echo "$dir does not exist"
    fi
    if [ -d "$dir" ]; then
        chmod -x $dir/*;
        cd $dir

        git status libcoexist.a | grep "modified" >/dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo $dir/libcoexist.a fixed
            $TOOLCHAIN-objcopy --redefine-sym ets_printf=coexist_printf libcoexist.a
            $TOOLCHAIN-objcopy --redefine-sym printf=coexist_printf libcoexist.a
        fi

        cd ..
    else
        echo "$dir does not exist"
    fi
done;
