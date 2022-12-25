    nix develop github:BBBSnowball/nixcfg#purethermal
    OPTIONS="MLX90614 MLX90614_OVERLAY OVERLAY_DEFAULT_ON"
    make SYSTEM=arm-none-eabihf- OPTIONS="$OPTIONS" STRICT=1 -j
    make SYSTEM=arm-none-eabihf- OPTIONS="$OPTIONS" STRICT=1 flash-dfu

Determine include path, e.g. for VS Code cpptools (but also set `compilerPath`):

    arm-none-eabihf-gcc -xc++ -E -v /dev/null |& sed -n '/^#include .* search starts here:/,/End of search list/ s/^\s*\(\/.*\)/\1/p'
