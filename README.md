# RDM630 Library for Pico using PIO's and FreeRTOS

```bash
rm -rvf build
cmake -B build -S . -DEXAMPLE_BUILD=ON -DFREERTOS_DIR=/Users/matt/Dropbox/Work/LWK/Pico-RP2040/github/FreeRTOS-Kernel
cd build
make
```

Likely need to increase the number of shared irq handlers if using more than one instance

```cmake
target_compile_definitions(${NAME} PRIVATE
    PICO_MAX_SHARED_IRQ_HANDLERS=8
)
```
